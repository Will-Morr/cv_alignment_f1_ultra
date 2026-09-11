"""LAN driver for the xTool F1 Ultra on "V2" firmware (>= 40.52).

The old plain-HTTP API on ports 8080/8329 is gone. Everything now goes over a
TLS WebSocket on port 28900:

* ``function=instruction`` carries JSON requests/responses plus push events,
  each wrapped in a CRC16 binary envelope (magic 0xBABE).
* ``function=file_stream`` carries bulk bytes (camera JPEGs down, job files up)
  with a sliding-window transfer protocol driven by FILE_REQUEST/FILE_DATA packets.

Protocol reference (reverse engineered): https://github.com/thecodingdad/ha-xtool
"""
import io
import ssl
import json
import time
import tarfile
import hashlib
import threading
from websockets.sync.client import connect as ws_connect

HOST = "192.168.1.210"
PORT = 28900
XF_TEMPLATE = "F1-Ultra-template.xf"

# ---------------------------------------------------------------------------
# G-code / .xf job packaging
# ---------------------------------------------------------------------------

# Job prologue/epilogue as emitted by xTool Creative Space for this machine.
GCODE_HEADER = """G90
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
G103
G90
G0 F180000
"""

GCODE_FOOTER = """# END
# GS002 TAIL
G102
G103
G90
G0 S0
G0 F180000
G1 F180000
M536 U0
M6 P1
"""


def make_cut_gcode(paths, power=60.0, speed=50.0):
    """Vector G-code for a list of polylines in laser mm.

    paths: iterable of [(x, y), ...]; power in %, speed in mm/s.
    Uses the blue diode (G21). Z is not touched: focus before running.
    """
    lines = [
        "# GS002 VECTOR HEAD",
        "# motion_start",
        "G4M1",
        "G21",      # blue diode laser (G22 = infrared)
        "G90",
        "G0Q30",    # pulse frequency kHz (only matters for the IR source)
        "G4M1",
        "M523P40",
    ]
    for path in paths:
        x0, y0 = path[0]
        lines.append(f"G0X{x0:.3f}Y{y0:.3f}")
        for x, y in path[1:]:
            lines.append(f"G1X{x:.3f}Y{y:.3f}S{power * 10:.0f}F{speed:.0f}")   # S is % * 10
    return "\n".join(lines) + "\n"


def make_xf(gcode):
    """Build a .xf job package (a tar) by swapping motion.gcode in the template."""
    src = tarfile.open(XF_TEMPLATE, "r")
    buf = io.BytesIO()
    out = tarfile.open(fileobj=buf, mode="w")
    for member in src.getmembers():
        data = src.extractfile(member).read()
        if member.name == "motion.gcode":
            data = (GCODE_HEADER + gcode + GCODE_FOOTER).replace("\n", "\r\n").encode()
        info = tarfile.TarInfo(member.name)
        info.size = len(data)
        out.addfile(info, io.BytesIO(data))
    out.close()
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Wire framing
# ---------------------------------------------------------------------------

_SSL = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)   # device cert is self-signed by xTool's own CA
_SSL.check_hostname = False
_SSL.verify_mode = ssl.CERT_NONE

PT_JSON = 4          # envelope protocol types
PT_FILE = 33
FILE_REQUEST = 1     # file_stream opcodes
FILE_DATA = 129
PING_TXN = 65510     # transaction id reserved for heartbeats


def _crc16(data):
    """CRC-16/ARC (poly 0xA001 reflected, init 0)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _frame(payload, ptype=PT_JSON, crc=True):
    """10-byte header: magic, 3-byte length, type (bit7 = CRC disabled), payload CRC, header CRC."""
    h = bytearray(10)
    h[0:2] = b"\xba\xbe"
    h[2:5] = len(payload).to_bytes(3, "big")
    h[5] = (ptype & 0x7F) | (0 if crc else 0x80)
    h[6:8] = (_crc16(payload) if crc else 0).to_bytes(2, "big")
    h[8:10] = _crc16(bytes(h[:8])).to_bytes(2, "big")
    return bytes(h) + payload


def _unframe(buf):
    """Split a byte buffer into complete frames. Returns ([(ptype, payload)], remainder)."""
    frames, pos = [], 0
    while pos + 10 <= len(buf):
        if buf[pos:pos + 2] != b"\xba\xbe":
            pos += 1
            continue
        n = int.from_bytes(buf[pos + 2:pos + 5], "big")
        if pos + 10 + n > len(buf):
            break
        if _crc16(buf[pos:pos + 8]) != int.from_bytes(buf[pos + 8:pos + 10], "big"):
            pos += 1
            continue
        payload = buf[pos + 10:pos + 10 + n]
        if not (buf[pos + 5] & 0x80) and _crc16(payload) != int.from_bytes(buf[pos + 6:pos + 8], "big"):
            pos += 1
            continue
        frames.append((buf[pos + 5] & 0x7F, payload))
        pos += 10 + n
    return frames, buf[pos:]


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------

class F1Ultra:
    """Connection to one machine. Use as a context manager::

        with F1Ultra() as laser:
            jpg = laser.snap()
            laser.run_job(make_xf(make_cut_gcode(paths, power=80, speed=3000)))

    ``events`` collects push events (mode changes, head position, work results).
    """

    def __init__(self, host=HOST, verbose=False):
        self.host = host
        self.verbose = verbose
        self.session_id = int(time.time() * 1000)
        self.txn = 0
        self.channel = 0
        self.ws = None
        self.rx = b""
        self.events = []
        self._alive = False

    # ---- connection -------------------------------------------------------

    def _url(self, function):
        return f"wss://{self.host}:{PORT}/websocket?id={self.session_id}&function={function}"

    def _open(self, function, timeout=15):
        return ws_connect(self._url(function), ssl=_SSL, max_size=None, open_timeout=timeout,
                          additional_headers={"Origin": "atomm://renderer"})

    def connect(self):
        self.ws = self._open("instruction")
        # "parity" handshake: guest credentials, must be the first request
        self.request("/v1/user/parity", "GET", data={
            "userID": "mk-guest", "userKey": "bWFrZWJsb2NrLXh0b29s", "timezone": "UTC"})
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
        """Send one API request and return its ``data``. Push events seen meanwhile go to ``events``."""
        self.txn = self.txn % 65000 + 1
        txn = self.txn
        self._send_json({"type": "request", "method": method, "url": url, "params": params or {},
                         "data": data or {}, "timestamp": int(time.time() * 1000), "transactionId": txn})
        deadline = time.time() + timeout
        while True:
            ev = self._read_json(deadline - time.time())
            ev_txn = ev.get("transactionId")
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

    def _finish(self, ch):
        # Firmware answers code -1 after camera downloads even though the transfer completed.
        try:
            self.request("/v1/filetransfer/finish", "PUT",
                         data={"code": 0, "message": "file transfer finish", "channel": ch})
        except RuntimeError:
            pass

    def download(self, filename, filetype=5, timeout=30):
        """Fetch a file from the device (filetype 5 = camera/log blobs). MD5 is verified."""
        ch = self._next_channel()
        hs = self.request("/v1/filetransfer/download", "PUT", data={
            "filetype": filetype, "filename": filename, "digesttype": 1,
            "channel": ch, "packetsize": 1024 * 1024}, timeout=timeout)
        size = int(hs["filesize"])
        window = min(5 * 1024 * 1024, int(hs.get("packetsize") or 5 * 1024 * 1024))
        buf = bytearray(size)
        got = win_got = 0
        with self._open("file_stream") as fs:
            def req(offset):
                n = min(window, size - offset)
                pkt = bytes([FILE_REQUEST, ch]) + offset.to_bytes(5, "big") + n.to_bytes(3, "big")
                fs.send(_frame(pkt, PT_FILE, crc=False))
                return n
            want = req(0)
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
                    buf[off:off + len(content)] = content
                    got += len(content)
                    win_got += len(content)
                if win_got >= want and got < size:
                    win_got = 0
                    want = req(got)
        if hashlib.md5(buf).hexdigest().lower() != str(hs.get("digestdata", "")).lower():
            raise RuntimeError("download md5 mismatch")
        self._finish(ch)
        return bytes(buf)

    def upload(self, blob, filename, filetype=1, timeout=120):
        """Push a file to the device. The device asks for windows with FILE_REQUEST; we answer with FILE_DATA."""
        ch = self._next_channel()
        hs = self.request("/v1/filetransfer/upload", "PUT", data={
            "filetype": filetype, "filename": filename, "filesize": len(blob), "digesttype": 1,
            "digestdata": hashlib.md5(blob).hexdigest(), "channel": ch, "packetsize": 1024 * 1024},
            timeout=timeout)
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
                        continue
                    off = int.from_bytes(p[2:7], "big")
                    end = min(off + int.from_bytes(p[7:10], "big"), len(blob))
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
        """Runtime info; ``["curMode"]["mode"]`` is P_IDLE / P_SLEEP / Work / P_WORKING / P_WORK_DONE ..."""
        return self.request("/v1/device/runtime-infos")

    def set_mode(self, mode):
        return self.request("/v1/device/mode", "PUT", data={"mode": mode})

    def snap(self):
        """JPEG bytes from the bed camera (2592x1944 or 4656x3496, same field of view)."""
        r = self.request("/v1/camera/snap", "GET", params={"name": "main"}, timeout=30)
        return self.download(r["filename"], filetype=5)

    def set_fill_light(self, value):
        """Bed illumination 0-255. Turns off after every job."""
        return self.request("/v1/peripheral/param", "PUT", params={"type": "fill_light"},
                            data={"action": "set_bri", "idx": 1, "value": value})

    def go_to_z(self, z):
        """Move the head to an absolute Z (mm). Asynchronous: watch for MOTION_MOVE_FINISHED."""
        return self.request("/v1/laser-head/focus/control", "POST",
                            data={"action": "goTo", "Z": z, "stopFirst": 1, "F": 5000}, timeout=60)

    def autofocus(self, timeout=120):
        """Run the built-in height measurement and move to focus (~30 s). Returns the measured Z (mm).
        The start command is occasionally ignored; it is re-sent if no FOCUS_STARTED event follows."""
        seen = len(self.events)
        started = False
        t0 = time.time()
        for attempt in range(3):
            # the mode switch is sometimes slow or ignored; confirm it before starting
            self.set_mode("P_IDLE")
            time.sleep(1)
            for _ in range(10):
                self.set_mode("P_AUTOFOCUS")
                time.sleep(1)
                if self.status()["curMode"]["mode"] == "P_AUTOFOCUS":
                    break
            try:
                self.request("/v1/laser-head/focus/control", "POST",
                             data={"action": "auto_start", "stopFirst": 1}, timeout=timeout)
            except RuntimeError as e:   # code 1 = refused (not in autofocus mode yet)
                print("autofocus start refused, retrying:", e)
                time.sleep(2)
                continue
            t1 = time.time()
            while time.time() - t1 < 6 and not started:
                self.status()
                started = any(e["data"].get("type") == "FOCUS_STARTED" for e in self.events[seen:])
                time.sleep(0.5)
            if started:
                break
        while time.time() - t0 < timeout:
            self.status()   # drains push events
            if any(e["data"].get("type") == "FOCUS_FINISHED" for e in self.events[seen:]):
                zs = [e["data"]["info"]["z"] for e in self.events[seen:] if e.get("url") == "/laser_head/value"]
                return zs[-1] if zs else None
            time.sleep(0.5)
        raise TimeoutError("autofocus did not finish")

    def run_job(self, xf_bytes, auto_start=True, timeout=600):
        """Upload a .xf package and (optionally) start it. Blocks until the job finishes."""
        self.upload(xf_bytes, "tmp.xf", filetype=1, timeout=60)
        task = f"PC_F1Ultra_{int(time.time() * 1000)}"
        self.request("/v1/processing/upload/config", "PUT",
                     data={"fileType": "xf", "autoStart": int(auto_start), "taskId": task}, timeout=30)
        if not auto_start:
            return task
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.status()["curMode"]
            if m["mode"] == "P_WORK_DONE" and m["taskId"] == task:
                self.set_mode("P_IDLE")
                return task
            time.sleep(0.5)
        raise TimeoutError("job did not finish")

    def burn(self, paths, power=80.0, speed=3000.0):
        """Engrave polylines (laser mm) at the current Z."""
        return self.run_job(make_xf(make_cut_gcode(paths, power, speed)))


if __name__ == "__main__":
    with F1Ultra(verbose=True) as laser:
        print(json.dumps(laser.info(), indent=1)[:600])
        open("snap.jpg", "wb").write(laser.snap())
        print("saved snap.jpg")
