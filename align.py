"""Camera-based alignment of patterns to the workpiece on the F1 Ultra.

Usage:
  python align.py calibrate   # burn small marks, fit camera<->laser homography -> camera_calib.json
  python align.py wood        # photo + report workpiece corners in mm, save debug plot
  python align.py demo        # burn an inset border + crosshair aligned to the workpiece
"""
import sys, os, json, time
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import F1_ultra_driver as f1

CALIB_FILE = "camera_calib.json"
IMG_W, IMG_H = 2592, 1944   # camera returns 2592x1944 or 4656x3496 (same FOV); normalise to this
FILL_LIGHT = 30   # brighter saturates the wood in the camera's auto-exposure
DEBUG_DIR = "debug"


# ---- camera ---------------------------------------------------------------

def take_photo(laser):
    # The fill light goes off after a job; re-enable and retry until the frame is actually lit.
    for _ in range(5):
        laser.request("/v1/peripheral/param", "PUT", params={"type": "fill_light"},
                      data={"action": "set_bri", "idx": 1, "value": FILL_LIGHT})
        time.sleep(1.5)
        img = cv2.imdecode(np.frombuffer(laser.snap(), np.uint8), cv2.IMREAD_COLOR)
        if np.median(img) > 40:
            break
    if img.shape[1] != IMG_W:
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
    return img


def find_wood(img):
    """Workpiece = largest bright blob that does not touch the image border.
    Returns (4x2 float corners ordered TL,TR,BR,BL in image space, mask)."""
    v = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:, :, 2]
    _, th = cv2.threshold(cv2.GaussianBlur(v, (15, 15), 0), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((15, 15), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(th)
    h, w = th.shape
    best = None
    for i in range(1, n):
        x, y, bw, bh, a = stats[i]
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue
        if best is None or a > stats[best][4]:
            best = i
    mask = (lab == best).astype(np.uint8) * 255
    c = max(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    mask[:] = 0
    cv2.fillPoly(mask, [c], 255)   # burned marks and knots must not punch holes in the mask
    quad = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True).reshape(-1, 2).astype(float)
    if len(quad) != 4:
        quad = cv2.boxPoints(cv2.minAreaRect(c))
    return order_corners(quad), mask


def order_corners(pts):
    pts = np.asarray(pts, float)
    s, d = pts.sum(1), np.diff(pts, axis=1).ravel()
    return np.array([pts[s.argmin()], pts[d.argmin()], pts[s.argmax()], pts[d.argmax()]])


def detect_dark_blobs(img, mask, min_area=80, max_area=4000):
    """Centroids (px) of solid dark blobs on the workpiece (existing hatched marks)."""
    inner = cv2.erode(mask, np.ones((25, 25), np.uint8))
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    th = ((g < np.median(g[inner > 0]) - 60) & (inner > 0)).astype(np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(th)
    return [cent[i] for i in range(1, n) if min_area <= stats[i, 4] <= max_area]


# ---- laser ----------------------------------------------------------------

def burn(laser, paths, power=80.0, speed=3000.0):
    """Upload + run paths (mm) at the current Z, block until the job finishes."""
    xf = f1.make_xf(f1.make_cut_gcode(paths, 0.0, power, speed))
    laser.upload(xf, "tmp.xf", filetype=1, timeout=60)
    task = f"PC_F1Ultra_{int(time.time() * 1000)}"
    laser.request("/v1/processing/upload/config", "PUT",
                  data={"fileType": "xf", "autoStart": 1, "taskId": task}, timeout=30)
    t0 = time.time()
    while time.time() - t0 < 600:
        m = laser.status()["curMode"]
        if m["mode"] == "P_WORK_DONE" and m["taskId"] == task:
            laser.request("/v1/device/mode", "PUT", data={"mode": "P_IDLE"})
            return
        time.sleep(0.5)
    raise TimeoutError("job did not finish")


def hatched_square(cx, cy, size=3.0, pitch=0.3):
    h = size / 2
    lines = []
    for i, y in enumerate(np.arange(cy - h, cy + h + 1e-6, pitch)):
        xs = (cx - h, cx + h) if i % 2 == 0 else (cx + h, cx - h)
        lines.append([(xs[0], y), (xs[1], y)])
    return lines


# ---- calibration ----------------------------------------------------------

def load_calib():
    d = json.load(open(CALIB_FILE))
    return np.array(d["H_mm_to_px"]), d


def mm_to_px(H, pts):
    return cv2.perspectiveTransform(np.asarray(pts, float).reshape(-1, 1, 2), H).reshape(-1, 2)


def px_to_mm(H, pts):
    return cv2.perspectiveTransform(np.asarray(pts, float).reshape(-1, 1, 2), np.linalg.inv(H)).reshape(-1, 2)


def _fit(pairs):
    src = np.array([p[0] for p in pairs], np.float32); dst = np.array([p[1] for p in pairs], np.float32)
    H, _ = cv2.findHomography(src, dst, 0)
    return H, np.linalg.norm(mm_to_px(H, src) - dst, axis=1)


def burn_and_locate(laser, points_mm, pairs, tag):
    """Burn one mark per point; each is identified as the blob that was not in the previous photo."""
    img = take_photo(laser)
    quad, mask = find_wood(img)
    seen = detect_dark_blobs(img, mask)
    for x, y in points_mm:
        burn(laser, hatched_square(x, y))
        img = take_photo(laser)
        cv2.imwrite(f"{DEBUG_DIR}/{tag}_{x:.0f}_{y:.0f}.jpg", img)
        blobs = detect_dark_blobs(img, find_wood(img)[1])
        new = [b for b in blobs if all(np.hypot(*(b - s)) > 12 for s in seen)]
        if len(new) != 1:
            raise RuntimeError(f"mark ({x},{y}): expected 1 new blob, found {len(new)} (off the wood?)")
        print(f"mark ({x:.1f},{y:.1f}) mm -> {new[0].round(1)} px")
        pairs.append([[float(x), float(y)], new[0].tolist()])
        seen = blobs
    return img


def calibrate(laser, cluster=(120, 115), spread=20, inset=15):
    """Two passes: a small cluster of marks (no assumptions about camera orientation),
    then marks near the workpiece corners predicted from pass 1. Fit mm->px homography."""
    cx, cy = cluster
    pairs = []
    img = burn_and_locate(laser, [(cx - spread, cy - spread), (cx + spread, cy - spread),
                                  (cx + spread, cy + spread), (cx - spread, cy + spread)], pairs, "calib1")
    H, _ = _fit(pairs)
    corners = px_to_mm(H, find_wood(img)[0])
    c, ex, ey, (w, h) = rect_frame(corners)
    pts = [wood_local(corners, sx * (w / 2 - inset), sy * (h / 2 - inset)) for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
    print("pass 1 predicts wood corners mm:\n", corners.round(1), "\npass 2 marks at", np.round(pts, 1).tolist())
    img = burn_and_locate(laser, pts, pairs, "calib2")
    H, err = _fit(pairs)
    print(f"homography fit: {len(pairs)} points, residual px max {err.max():.2f} mean {err.mean():.2f}")
    json.dump({"H_mm_to_px": H.tolist(), "pairs": pairs, "img_size": [IMG_W, IMG_H],
               "fill_light": FILL_LIGHT}, open(CALIB_FILE, "w"), indent=1)
    return H, pairs, img


# ---- alignment ------------------------------------------------------------

def wood_mm(laser_or_img, H):
    img = laser_or_img if isinstance(laser_or_img, np.ndarray) else take_photo(laser_or_img)
    quad_px, mask = find_wood(img)
    return px_to_mm(H, quad_px), quad_px, img


def rect_frame(corners_mm):
    """Fit a rectangle to 4 corners: returns center, unit x axis, unit y axis, (w, h)."""
    c = corners_mm.mean(0)
    ex = (corners_mm[1] - corners_mm[0] + corners_mm[2] - corners_mm[3]) / 2
    w = np.linalg.norm(ex); ex /= w
    if ex[0] < 0:   # laser +x is image-left; keep the wood frame close to the laser frame
        ex = -ex
    ey = np.array([-ex[1], ex[0]])
    h = abs(np.dot(corners_mm[3] - corners_mm[0] + corners_mm[2] - corners_mm[1], ey)) / 2
    return c, ex, ey, (w, h)


def align_paths(paths, corners_mm, margin=5.0):
    """Scale/rotate/translate paths (in their own mm frame) to fit the workpiece rectangle."""
    pts = np.vstack([np.asarray(p, float) for p in paths])
    lo, hi = pts.min(0), pts.max(0)
    c, ex, ey, (w, h) = rect_frame(corners_mm)
    s = min((w - 2 * margin) / (hi[0] - lo[0]), (h - 2 * margin) / (hi[1] - lo[1]))
    mid = (lo + hi) / 2
    out = []
    for p in paths:
        q = (np.asarray(p, float) - mid) * s
        out.append([tuple(c + ex * x + ey * y) for x, y in q])
    return out


def wood_local(corners_mm, x, y):
    """Point given in the workpiece's own frame (origin at its center, axes along its edges)."""
    c, ex, ey, _ = rect_frame(corners_mm)
    return tuple(c + ex * x + ey * y)


def plot_alignment(img, H, corners_mm, paths, fname):
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    q = mm_to_px(H, corners_mm); ax.plot(*np.vstack([q, q[:1]]).T, "c-", lw=2, label="wood")
    for p in paths:
        ax.plot(*mm_to_px(H, p).T, "m-", lw=1.5)
    ax.legend(); plt.savefig(fname, dpi=70)


if __name__ == "__main__":
    os.makedirs(DEBUG_DIR, exist_ok=True)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "wood"
    with f1.F1Ultra(verbose=False) as laser:
        if cmd == "calibrate":
            calibrate(laser)
        elif cmd == "wood":
            H, _ = load_calib()
            corners, quad_px, img = wood_mm(laser, H)
            c, ex, ey, (w, h) = rect_frame(corners)
            print("wood corners mm:\n", corners.round(1))
            print(f"center {c.round(1)} size {w:.1f} x {h:.1f} mm, rotation {np.degrees(np.arctan2(ex[1], ex[0])):.2f} deg")
            plot_alignment(img, H, corners, [], f"{DEBUG_DIR}/wood_debug.png")
        elif cmd == "demo":
            H, _ = load_calib()
            corners, quad_px, img = wood_mm(laser, H)
            c, ex, ey, (w, h) = rect_frame(corners)
            # a border inset 5 mm from every edge + a centred crosshair, in the wood's own frame
            hw, hh = w / 2 - 5, h / 2 - 5
            border = [wood_local(corners, x, y) for x, y in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]]
            cross = [[wood_local(corners, -10, 0), wood_local(corners, 10, 0)],
                     [wood_local(corners, 0, -10), wood_local(corners, 0, 10)]]
            paths = [border] + cross
            plot_alignment(img, H, corners, paths, f"{DEBUG_DIR}/demo_plan.png")
            burn(laser, paths, power=80.0, speed=3000.0)
            after = take_photo(laser)
            plot_alignment(after, H, corners, paths, f"{DEBUG_DIR}/demo_result.png")
