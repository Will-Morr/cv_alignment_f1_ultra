import numpy as np
import random
import math
import tarfile
import io
import ssl
import json
import time
import hashlib
import threading
from PIL import Image
from websockets.sync.client import connect as ws_connect

header = """
G90
G0 F3000
G4M1
M9064 B2
M9039 C2
# GS002 HEAD
G0 F180000
M4 S0
G1 F180000
G0 X0 Y0

G102
G91
#G0Z4F600
G103
G90
G0 F180000
"""

footer = """
# END
# GS002 TAIL



G102
#G91
#G0Z-4F600
G103




G90
G0 S0
G0 F180000
G1 F180000
M536 U0
M6 P1

"""

def make_cut_gcode(paths,
        Z = 0.0,   # mm
        power = 60.0, # %
        speed = 50.0, #mm/s
    ):
    # G0 move XY
    # G1 cut, XY s=power, f=feedrate
    # G0Q30 frequency 30-60

    # Put your gcode here

    contents = f"""
# Do this when the laser changes:
# GS002 VECTOR HEAD
# motion_start
G4M1
#G21=Blue or G22=fiber for which laser to use
G21
G90 # Absolute moves for laser
G0Q30 # 30 kHz for fiber laser

G4M1
M523P40

# Z move
# G102
# G91 #incremental
# G0Z{Z}F600
# G90 #absolute
# G103
# G0F180000
"""

    def round3(v):
        return round(v, 3)

    parts = []

    for fooPath in paths:
        x0, y0 = fooPath[0]
        parts.append(f"G0X{round3(x0)}Y{round3(y0)}")

        for x, y in fooPath[1:]:
            parts.append(f"G1X{round3(x)}Y{round3(y)}S{power*10.0}F{speed}") # power is in %*10

    parts.append("")

    contents += '\n'.join(parts)

    # // TODO: Return z to start
    # result.push("#".to_string());
    # result.push(format!("G0Z{}", round3(23.0)));

    return contents

def make_xf(contents):
    filename = "F1-Ultra-template.xf"
    t = tarfile.open(filename, 'r')
    files = t.getmembers()

    tar_fileobj = io.BytesIO()

    #output = tarfile.open('the_test_file.xf','w')
    output = tarfile.open(fileobj=tar_fileobj, mode='w')

    # tarfile.TarInfo("preview.jpg")
    # tarfile.TarInfo("motion.gcode")
    # tarfile.TarInfo("description.json")
    # tarfile.TarInfo("border.gcode")

    for part in files:
        f = t.extractfile(part)
        data = f.read()
        print(part.name, data[:100])

        if part.name == 'motion.gcode':
            info = tarfile.TarInfo("motion.gcode")
            data = header+contents+footer
            data = data.replace('\n', '\r\n')
            info.size = len(data)
            output.addfile(info, io.BytesIO(data.encode()))
            #print(data.encode())
        else:
            info = tarfile.TarInfo(part.name)
            info.size = len(data)
            output.addfile(info, io.BytesIO(data))
    output.close()
    print("Done")
    tar_fileobj.seek(0)
    return tar_fileobj.read()


# ---------------------------------------------------------------------------
# xTool "V2" LAN protocol (firmware >= 40.52 on the F1 Ultra).
# The old plain-HTTP API on :8080/:8329 is gone. Everything now goes over a TLS
# WebSocket on :28900. JSON requests ride in a CRC16-framed binary envelope on
# the "instruction" channel; bulk bytes (camera JPEGs, job files) go over a
# separate "file_stream" channel with a sliding-window transfer protocol.
# Reverse-engineered by https://github.com/thecodingdad/ha-xtool (docs/PROTOCOL.md).
# ---------------------------------------------------------------------------

HOST = "192.168.1.210"

_SSL = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
_SSL.check_hostname = False
_SSL.verify_mode = ssl.CERT_NONE

PT_JSON = 4
PT_FILE = 33
FILE_REQUEST = 1
FILE_DATA = 129
PING_TXN = 65510


def _crc16(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _frame(payload, ptype=PT_JSON, crc=True):
    h = bytearray(10)
    h[0:2] = b"\xba\xbe"
    h[2:5] = len(payload).to_bytes(3, "big")
    h[5] = (ptype & 0x7F) | (0 if crc else 0x80)
    h[6:8] = (_crc16(payload) if crc else 0).to_bytes(2, "big")
    h[8:10] = _crc16(bytes(h[:8])).to_bytes(2, "big")
    return bytes(h) + payload


def _unframe(buf):
    """Return (frames, remainder). Frames are (ptype, payload)."""
    frames, pos = [], 0
    while pos + 10 <= len(buf):
        if buf[pos:pos+2] != b"\xba\xbe":
            pos += 1
            continue
        n = int.from_bytes(buf[pos+2:pos+5], "big")
        if pos + 10 + n > len(buf):
            break
        if _crc16(buf[pos:pos+8]) != int.from_bytes(buf[pos+8:pos+10], "big"):
            pos += 1
            continue
        payload = buf[pos+10:pos+10+n]
        if not (buf[pos+5] & 0x80) and _crc16(payload) != int.from_bytes(buf[pos+6:pos+8], "big"):
            pos += 1
            continue
        frames.append((buf[pos+5] & 0x7F, payload))
        pos += 10 + n
    return frames, buf[pos:]


class F1Ultra:
    def __init__(self, host=HOST, verbose=True):
        self.host = host
        self.verbose = verbose
        self.session_id = int(time.time() * 1000)
        self.txn = 0
        self.channel = 0
        self.ws = None
        self.rx = b""
        self.events = []

    def _url(self, function):
        return f"wss://{self.host}:28900/websocket?id={self.session_id}&function={function}"

    def _open(self, function, timeout=15):
        return ws_connect(self._url(function), ssl=_SSL, max_size=None, open_timeout=timeout,
                          additional_headers={"Origin": "atomm://renderer"})

    def connect(self):
        self.ws = self._open("instruction")
        self.request("/v1/user/parity", "GET", data={
            "userID": "mk-guest", "userKey": "bWFrZWJsb2NrLXh0b29s", "timezone": "America/New_York"})
        self._alive = True
        threading.Thread(target=self._heartbeat, daemon=True).start()
        return self

    def close(self):
        self._alive = False
        if self.ws:
            self.ws.close()
            self.ws = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *a):
        self.close()

    def _send_json(self, obj):
        self.ws.send(_frame(json.dumps(obj, separators=(",", ":")).encode()))

    def _heartbeat(self):
        while self._alive:
            time.sleep(3)
            if not self._alive:
                break
            try:
                self._send_json({"type": "request", "method": "GET", "url": "/v1/user/ping",
                                 "transactionId": PING_TXN, "data": {}, "params": {},
                                 "timestamp": int(time.time() * 1000)})
            except Exception:
                break

    def _read_json(self, timeout):
        """Block until one JSON frame arrives on the instruction channel."""
        deadline = time.time() + timeout
        while True:
            frames, self.rx = _unframe(self.rx)
            for ptype, payload in frames:
                if ptype == PT_JSON:
                    return json.loads(payload)
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError("no response from laser")
            msg = self.ws.recv(timeout=remaining)
            self.rx += msg if isinstance(msg, bytes) else msg.encode()

    def request(self, url, method="GET", params=None, data=None, timeout=10):
        self.txn = self.txn % 65000 + 1
        txn = self.txn
        self._send_json({"type": "request", "method": method, "url": url, "params": params or {},
                         "data": data or {}, "timestamp": int(time.time() * 1000), "transactionId": txn})
        deadline = time.time() + timeout
        while True:
            ev = self._read_json(deadline - time.time())
            ev_txn = ev.get("transactionId", (ev.get("data") or {}).get("transactionId") if isinstance(ev.get("data"), dict) else None)
            if ev.get("type") == "response" and ev_txn == PING_TXN:
                continue
            if ev.get("type") == "response" and ev_txn == txn:
                if ev.get("code", 0) != 0:
                    raise RuntimeError(f"{method} {url} -> code {ev.get('code')}: {ev.get('msg')}")
                if self.verbose:
                    print(f"[laser] {method} {url} -> {ev.get('data')}")
                return ev.get("data") or {}
            self.events.append(ev)
            if self.verbose:
                print(f"[laser] event: {ev.get('url')} {ev.get('data')}")

    # ---- file transfer ----------------------------------------------------

    def _next_channel(self):
        self.channel = (self.channel + 1) & 0xFF
        return self.channel

    def download(self, filename, filetype=5, timeout=30):
        ch = self._next_channel()
        hs = self.request("/v1/filetransfer/download", "PUT", data={
            "filetype": filetype, "filename": filename, "digesttype": 1,
            "channel": ch, "packetsize": 1024 * 1024}, timeout=timeout)
        size = int(hs["filesize"])
        window = min(5 * 1024 * 1024, int(hs.get("packetsize") or 5 * 1024 * 1024))
        buf = bytearray(size)
        got = 0
        with self._open("file_stream") as fs:
            def req(offset):
                n = min(window, size - offset)
                pkt = bytes([FILE_REQUEST, ch]) + offset.to_bytes(5, "big") + n.to_bytes(3, "big")
                fs.send(_frame(pkt, PT_FILE, crc=False))
                return n
            want = req(0)
            win_got = 0
            scan = b""
            deadline = time.time() + timeout
            while got < size:
                scan += fs.recv(timeout=max(0.1, deadline - time.time()))
                frames, scan = _unframe(scan)
                for ptype, p in frames:
                    if ptype != PT_FILE or len(p) < 7 or p[0] != FILE_DATA or p[1] != ch:
                        continue
                    off = int.from_bytes(p[2:7], "big")
                    content = p[7:]
                    buf[off:off+len(content)] = content
                    got += len(content)
                    win_got += len(content)
                if win_got >= want and got < size:
                    win_got = 0
                    want = req(got)
        if hashlib.md5(buf).hexdigest().lower() != str(hs.get("digestdata", "")).lower():
            raise RuntimeError("download md5 mismatch")
        self._finish(ch)
        return bytes(buf)

    def _finish(self, ch):
        # Firmware answers code -1 for camera snaps even though the transfer is complete.
        try:
            self.request("/v1/filetransfer/finish", "PUT", data={"code": 0, "message": "file transfer finish", "channel": ch})
        except RuntimeError as e:
            if self.verbose:
                print(f"[laser] finish ignored: {e}")

    def upload(self, blob, filename, filetype=1, timeout=120):
        """Push a blob to the device. The device drives the transfer by sending
        FILE_REQUEST windows; we answer each with FILE_DATA packets."""
        ch = self._next_channel()
        hs = self.request("/v1/filetransfer/upload", "PUT", data={
            "filetype": filetype, "filename": filename, "filesize": len(blob), "digesttype": 1,
            "digestdata": hashlib.md5(blob).hexdigest(), "channel": ch, "packetsize": 1024 * 1024}, timeout=timeout)
        packet = int(hs.get("packetsize") or 64 * 1024)
        sent_to = 0
        with self._open("file_stream") as fs:
            scan = b""
            deadline = time.time() + timeout
            while sent_to < len(blob):
                scan += fs.recv(timeout=max(0.1, deadline - time.time()))
                frames, scan = _unframe(scan)
                for ptype, p in frames:
                    if ptype != PT_FILE or len(p) < 10 or p[0] != FILE_REQUEST or p[1] != ch:
                        if self.verbose:
                            print(f"[laser] file_stream: ignoring frame ptype={ptype} {p[:12].hex()}")
                        continue
                    off = int.from_bytes(p[2:7], "big")
                    win = int.from_bytes(p[7:10], "big")
                    end = min(off + win, len(blob))
                    if self.verbose:
                        print(f"[laser] FILE_REQUEST offset={off} window={win}")
                    while off < end:
                        chunk = blob[off:min(off + packet, end)]
                        fs.send(_frame(bytes([FILE_DATA, ch]) + off.to_bytes(5, "big") + chunk, PT_FILE))
                        off += len(chunk)
                    sent_to = max(sent_to, end)
        self._finish(ch)

    # ---- high level -------------------------------------------------------

    def info(self):
        return self.request("/v1/device/machineInfo")

    def status(self):
        return self.request("/v1/device/runtime-infos")

    def snap(self):
        r = self.request("/v1/camera/snap", "GET", params={"name": "main"}, timeout=30)
        return self.download(r["filename"], filetype=5)

    def go_to_z(self, z):
        return self.request("/v1/laser-head/focus/control", "POST",
                            data={"action": "goTo", "Z": z, "stopFirst": 1, "F": 5000}, timeout=60)

    def autofocus(self, timeout=60):
        """Run the built-in height measurement. Returns the measured Z (mm)."""
        self.request("/v1/device/mode", "PUT", data={"mode": "P_AUTOFOCUS"})
        self.request("/v1/laser-head/focus/control", "POST", data={"action": "auto_start", "stopFirst": 1}, timeout=timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.status()  # drains push events
            if any(e["data"].get("type") == "FOCUS_FINISHED" for e in self.events):
                zs = [e["data"]["info"]["z"] for e in self.events if e.get("url") == "/laser_head/value"]
                return zs[-1] if zs else None
            time.sleep(0.5)
        raise TimeoutError("autofocus did not finish")

    def autofocus(self, timeout=60):
        """Run the built-in height measurement. Returns the measured Z (mm)."""
        self.request("/v1/device/mode", "PUT", data={"mode": "P_AUTOFOCUS"})
        self.request("/v1/laser-head/focus/control", "POST", data={"action": "auto_start", "stopFirst": 1}, timeout=timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.status()  # drains push events
            done = [e for e in self.events if e["data"].get("type") == "FOCUS_FINISHED"]
            if done:
                zs = [e["data"]["info"]["z"] for e in self.events if e.get("url") == "/laser_head/value"]
                return zs[-1] if zs else None
            time.sleep(0.5)
        raise TimeoutError("autofocus did not finish")


# Get the camera image (a jpg) with the same settings that xtool uses
def getPhoto(outPath = None):
    with F1Ultra() as laser:
        data = laser.snap()

    if outPath != None:
        with open(outPath+".jpg", "wb") as f:
            f.write(data)

    return Image.open(io.BytesIO(data))

def runLines(
        inputLines,
        Z = 0.0,   # mm, or None to leave Z where it is
        power = 60.0, # %
        speed = 50.0, #mm/s
        autoStart = 1,
        ):
    xf_data = make_xf(make_cut_gcode(inputLines, Z, power, speed))
    with F1Ultra() as laser:
        if Z is not None:  # None keeps the current (e.g. autofocused) height
            laser.go_to_z(Z)
            time.sleep(3)
        laser.upload(xf_data, "tmp.xf", filetype=1)
        taskId = f"PC_F1Ultra_MXFK002B2024072307949AB_{int(time.time()*1000)}"
        laser.request("/v1/processing/upload/config", "PUT",
                      data={"fileType": "xf", "gcodeType": "processing", "autoStart": autoStart, "taskId": taskId})

if __name__ == '__main__':
    getPhoto('tmp')
