"""Camera-based alignment of engravings to the workpiece on an xTool F1 Ultra.

Commands (all fire the laser except ``wood``):

  python align.py calibrate   burn 8 small marks, fit the camera->laser homography -> camera_calib.json
  python align.py wood        photo + workpiece corners / size / rotation in laser mm (+ debug plot)
  python align.py test        autofocus, then engrave a test pattern aligned to the workpiece:
                              corner brackets, a nested-squares whirl, and a text block with workpiece stats

Library use::

    with F1Ultra() as laser:
        H, calib = load_calib()
        corners, quad_px, img = wood_mm(laser, H)     # workpiece corners in laser mm
        focus(laser, calib)                           # autofocus, checked against the calibration Z
        laser.burn(align_paths(my_paths, corners))    # pattern scaled/rotated/centred on the workpiece
"""
import os
import sys
import shutil
import subprocess
import json
import time
import datetime
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
from F1_ultra_driver import F1Ultra, make_cut_gcode, make_xf

CALIB_FILE = "camera_calib.json"
IMG_W, IMG_H = 2592, 1944   # camera returns 2592x1944 or 4656x3496 (same FOV); normalise to this
FILL_LIGHT = 30             # brighter saturates light wood in the camera's auto-exposure
RUNS_DIR = "runs"
OUT = "runs/untracked"       # set per invocation by new_run()


def new_run(cmd, **meta):
    """Create runs/<time>_<cmd>_<git hash>/ and make it the output dir for this invocation.
    Every laser run is recorded with the exact code version that produced it."""
    global OUT
    git = lambda *a: subprocess.run(["git", *a], capture_output=True, text=True, cwd=os.path.dirname(os.path.abspath(__file__))).stdout.strip()
    rev = git("rev-parse", "--short", "HEAD") or "nogit"
    dirty = bool(git("status", "--porcelain", "--untracked-files=no"))
    OUT = os.path.join(RUNS_DIR, f"{datetime.datetime.now():%Y%m%d_%H%M%S}_{cmd}_{rev}{'-dirty' if dirty else ''}")
    os.makedirs(OUT, exist_ok=True)
    json.dump({"command": cmd, "time": datetime.datetime.now().isoformat(timespec="seconds"),
               "git_commit": git("rev-parse", "HEAD"), "git_dirty": dirty, "git_diff": git("diff") if dirty else "",
               **meta}, open(os.path.join(OUT, "meta.json"), "w"), indent=1)
    if os.path.exists(CALIB_FILE):
        shutil.copy(CALIB_FILE, OUT)
    return OUT


def save(name, obj):
    """Write an artefact into the current run directory (image, JSON-able object, or text)."""
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    if isinstance(obj, np.ndarray):
        cv2.imwrite(path, obj)
    elif isinstance(obj, str):
        open(path, "w").write(obj)
    else:
        json.dump(obj, open(path, "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    return path


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def take_photo(laser, tries=6):
    """Lit, normalised BGR photo of the bed.

    After an autofocus (and sometimes a lid cycle) the camera returns black frames and the fill
    light cannot be re-armed by the brightness call alone; running an empty zero-power job
    resets it. Frames are retried until neither dark nor saturated.
    """
    for i in range(tries):
        laser.set_fill_light(FILL_LIGHT)
        time.sleep(2)
        img = cv2.imdecode(np.frombuffer(laser.snap(), np.uint8), cv2.IMREAD_COLOR)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if np.median(g) > 40 and (g >= 250).mean() < 0.05:
            break
        if np.median(g) <= 40:
            laser.run_job(make_xf(""))   # header + footer only, laser power 0: wakes the camera/light
    else:
        raise RuntimeError("could not get a usable photo (dark or saturated); is the lid closed?")
    if img.shape[1] != IMG_W:
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
    return img


def find_wood(img):
    """Workpiece = largest bright blob that does not touch the image border.

    Returns (corners 4x2 float ordered TL,TR,BR,BL in image space, filled uint8 mask).
    """
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
    if best is None:
        raise RuntimeError("no workpiece found (nothing bright fully inside the frame)")
    mask = (lab == best).astype(np.uint8) * 255
    c = max(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    mask[:] = 0
    cv2.fillPoly(mask, [c], 255)   # engraved marks and knots must not punch holes in the mask
    quad = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True).reshape(-1, 2).astype(float)
    if len(quad) != 4:
        quad = cv2.boxPoints(cv2.minAreaRect(c))
    return order_corners(quad), mask


def order_corners(pts):
    pts = np.asarray(pts, float)
    s, d = pts.sum(1), np.diff(pts, axis=1).ravel()
    return np.array([pts[s.argmin()], pts[d.argmin()], pts[s.argmax()], pts[d.argmax()]])


def detect_dark_blobs(img, mask, min_area=80, max_area=4000):
    """Centroids (px) of small solid dark blobs on the workpiece, e.g. the hatched calibration marks."""
    inner = cv2.erode(mask, np.ones((25, 25), np.uint8))
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    th = ((g < np.median(g[inner > 0]) - 60) & (inner > 0)).astype(np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(th)
    return [cent[i] for i in range(1, n) if min_area <= stats[i, 4] <= max_area]


# ---------------------------------------------------------------------------
# Calibration: laser mm  <->  image px
# ---------------------------------------------------------------------------

def load_calib():
    d = json.load(open(CALIB_FILE))
    return np.array(d["H_mm_to_px"]), d


def mm_to_px(H, pts):
    return cv2.perspectiveTransform(np.asarray(pts, float).reshape(-1, 1, 2), H).reshape(-1, 2)


def px_to_mm(H, pts):
    return cv2.perspectiveTransform(np.asarray(pts, float).reshape(-1, 1, 2), np.linalg.inv(H)).reshape(-1, 2)


def hatched_square(cx, cy, size=3.0, pitch=0.3):
    """Serpentine fill of a small square: a solid dark blob the camera can find."""
    h = size / 2
    lines = []
    for i, y in enumerate(np.arange(cy - h, cy + h + 1e-6, pitch)):
        xs = (cx - h, cx + h) if i % 2 == 0 else (cx + h, cx - h)
        lines.append([(xs[0], y), (xs[1], y)])
    return lines


def _fit(pairs):
    src = np.array([p[0] for p in pairs], np.float32)
    dst = np.array([p[1] for p in pairs], np.float32)
    H, _ = cv2.findHomography(src, dst, 0)
    return H, np.linalg.norm(mm_to_px(H, src) - dst, axis=1)


def burn_and_locate(laser, points_mm, pairs, tag):
    """Burn one mark per point; each is identified as the blob that was not in the previous photo."""
    img = take_photo(laser)
    seen = detect_dark_blobs(img, find_wood(img)[1])
    for x, y in points_mm:
        laser.burn(hatched_square(x, y))
        img = take_photo(laser)
        save(f"{tag}_{x:.0f}_{y:.0f}.jpg", img)
        blobs = detect_dark_blobs(img, find_wood(img)[1])
        new = [b for b in blobs if all(np.hypot(*(b - s)) > 12 for s in seen)]
        if len(new) != 1:
            raise RuntimeError(f"mark ({x},{y}): expected 1 new blob, found {len(new)} (off the workpiece?)")
        print(f"mark ({x:.1f},{y:.1f}) mm -> {new[0].round(1)} px")
        pairs.append([[float(x), float(y)], new[0].tolist()])
        seen = blobs
    return img


Z_TOLERANCE = 1.5   # mm; the camera rides on the head, so the homography only holds near the calibration Z


def focus(laser, calib=None):
    """Autofocus and check the result against the Z the calibration was made at.
    A far-off value means a different material thickness (recalibrate) or a bad measurement."""
    z_ref = (calib or {}).get("z_mm")
    for attempt in range(2):
        z = laser.autofocus()
        print(f"autofocus Z = {z} mm" + (f" (calibration at {z_ref} mm)" if z_ref else ""))
        if z_ref is None or abs(z - z_ref) <= Z_TOLERANCE:
            return z
    raise RuntimeError(f"autofocus Z {z} mm is {z - z_ref:+.1f} mm from the calibration Z {z_ref} mm; "
                       "different material thickness? run calibrate again")


def calibrate(laser, cluster=(120, 115), spread=20, inset=15):
    """Two passes: a small cluster of marks (no assumptions about camera orientation), then marks
    near the workpiece corners predicted from pass 1. Fits and saves the mm->px homography.
    Autofocuses first and records the Z, since the mapping is only valid at that head height."""
    z = focus(laser)
    cx, cy = cluster
    pairs = []
    img = burn_and_locate(laser, [(cx - spread, cy - spread), (cx + spread, cy - spread),
                                  (cx + spread, cy + spread), (cx - spread, cy + spread)], pairs, "calib1")
    H, _ = _fit(pairs)
    corners = px_to_mm(H, find_wood(img)[0])
    c, ex, ey, (w, h) = rect_frame(corners)
    pts = [wood_local(corners, sx * (w / 2 - inset), sy * (h / 2 - inset))
           for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
    print("pass 1 predicts workpiece corners mm:\n", corners.round(1), "\npass 2 marks at", np.round(pts, 1).tolist())
    img = burn_and_locate(laser, pts, pairs, "calib2")
    H, err = _fit(pairs)
    print(f"homography fit: {len(pairs)} points, residual px max {err.max():.2f} mean {err.mean():.2f}")
    json.dump({"H_mm_to_px": H.tolist(), "pairs": pairs, "residual_px": err.tolist(), "z_mm": z,
               "img_size": [IMG_W, IMG_H], "fill_light": FILL_LIGHT,
               "date": datetime.date.today().isoformat()}, open(CALIB_FILE, "w"), indent=1)
    shutil.copy(CALIB_FILE, OUT)
    save("final.jpg", img)
    return H, pairs, img


# ---------------------------------------------------------------------------
# Workpiece geometry and pattern alignment (all in laser mm)
# ---------------------------------------------------------------------------

def wood_mm(laser_or_img, H):
    """Photograph (or use a given image) and return (corners mm 4x2, corners px 4x2, image)."""
    img = laser_or_img if isinstance(laser_or_img, np.ndarray) else take_photo(laser_or_img)
    quad_px, _ = find_wood(img)
    return px_to_mm(H, quad_px), quad_px, img


def rect_frame(corners_mm):
    """Fit a rectangle to 4 corners. Returns (center, unit x axis, unit y axis, (w, h)) in laser mm.

    The frame follows the physical / camera view of the bed: x to the right, y up as you look
    at the machine. Laser +x runs the *other* way, so a pattern expressed in raw laser
    coordinates is mirrored on the part; anything laid out in this frame reads correctly.
    """
    c = corners_mm.mean(0)
    ex = (corners_mm[1] - corners_mm[0] + corners_mm[2] - corners_mm[3]) / 2
    w = np.linalg.norm(ex)
    ex /= w
    if ex[0] > 0:            # image-right is laser -x
        ex = -ex
    ey = np.array([ex[1], -ex[0]])   # image-up is laser +y
    h = abs(np.dot(corners_mm[3] - corners_mm[0] + corners_mm[2] - corners_mm[1], ey)) / 2
    return c, ex, ey, (w, h)


def frame_rotation_deg(corners_mm):
    """Rotation of the workpiece's x edge relative to the bed, as seen in the camera view."""
    _, ex, _, _ = rect_frame(corners_mm)
    return np.degrees(np.arctan2(-ex[1], -ex[0]))


def wood_local(corners_mm, x, y):
    """Laser mm of a point given in the workpiece frame (origin at its centre, axes along its edges)."""
    c, ex, ey, _ = rect_frame(corners_mm)
    return tuple(c + ex * x + ey * y)


def to_wood(corners_mm, paths):
    """Map paths expressed in the workpiece frame into laser mm."""
    c, ex, ey, _ = rect_frame(corners_mm)
    return [[tuple(c + ex * x + ey * y) for x, y in p] for p in paths]


def align_paths(paths, corners_mm, margin=5.0):
    """Uniformly scale, rotate and centre paths (any units) to fit inside the workpiece with a margin."""
    pts = np.vstack([np.asarray(p, float) for p in paths])
    lo, hi = pts.min(0), pts.max(0)
    _, _, _, (w, h) = rect_frame(corners_mm)
    s = min((w - 2 * margin) / (hi[0] - lo[0]), (h - 2 * margin) / (hi[1] - lo[1]))
    mid = (lo + hi) / 2
    return to_wood(corners_mm, [[(x, y) for x, y in (np.asarray(p, float) - mid) * s] for p in paths])


# ---------------------------------------------------------------------------
# Pattern generators (workpiece frame, mm)
# ---------------------------------------------------------------------------

def corner_brackets(w, h, inset=3.0, arm=12.0):
    """L-shaped brackets hugging each corner: the clearest visual check of edge alignment."""
    hx, hy = w / 2 - inset, h / 2 - inset
    out = []
    for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        out.append([(sx * (hx - arm), sy * hy), (sx * hx, sy * hy), (sx * hx, sy * (hy - arm))])
    return out


def nested_squares(side, steps=14, twist_deg=5.0):
    """Whirl of squares, each inscribed in the previous one and turned by ``twist_deg``.
    The outer square is axis-aligned, so any misalignment with the workpiece edges is obvious."""
    out = []
    half, ang = side / 2, 0.0
    for _ in range(steps):
        cs = [np.array([np.cos(np.radians(ang + q)), np.sin(np.radians(ang + q))]) * half * np.sqrt(2)
              for q in (45, 135, 225, 315)]
        out.append([tuple(p) for p in cs + cs[:1]])
        # next square has its corners on this square's edges, turned by the twist
        half /= np.cos(np.radians(twist_deg)) + np.sin(np.radians(twist_deg))
        ang += twist_deg
    return out


def text_paths(text, height=4.0, x=0.0, y=0.0, align="center"):
    """Outline paths for a line of text using matplotlib's font machinery (no extra dependency).
    ``height`` is the cap height in mm; (x, y) is the baseline anchor."""
    tp = TextPath((0, 0), text, size=height / 0.72, prop=FontProperties(family="DejaVu Sans"))
    polys = [np.asarray(p) for p in tp.to_polygons() if len(p) > 2]
    xs = np.vstack(polys)[:, 0]
    dx = {"center": -(xs.min() + xs.max()) / 2, "left": -xs.min(), "right": -xs.max()}[align]
    return [[(px + dx + x, py + y) for px, py in np.vstack([p, p[:1]])] for p in polys]


def test_pattern(corners_mm, z, calib_residual_mm):
    """Corner brackets + nested-squares whirl + a stats block, all in the workpiece frame."""
    c, ex, ey, (w, h) = rect_frame(corners_mm)
    rot = frame_rotation_deg(corners_mm)
    stats = [
        f"{w:.1f} x {h:.1f} mm   ctr ({c[0]:.1f}, {c[1]:.1f})   rot {rot:+.2f} deg",
        f"Z {z:.2f} mm   calib {calib_residual_mm:.2f} mm   {datetime.date.today().isoformat()}",
    ]
    margin, text_h = 8.0, 16.0
    paths = corner_brackets(w, h)
    side = min(w - 2 * margin, h - 2 * margin - text_h)
    cy = (h / 2 - margin) - side / 2                       # whirl sits above the text block
    paths += [[(x, y + cy) for x, y in sq] for sq in nested_squares(side)]
    y0 = -h / 2 + margin
    for i, line in enumerate(reversed(stats)):
        paths += text_paths(line, height=3.5, y=y0 + i * 6.0)
    return to_wood(corners_mm, paths)


# ---------------------------------------------------------------------------
# Debug plotting
# ---------------------------------------------------------------------------

def plot_alignment(img, H, corners_mm, paths, fname):
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    q = mm_to_px(H, corners_mm)
    ax.plot(*np.vstack([q, q[:1]]).T, "c-", lw=2, label="workpiece")
    for p in paths:
        ax.plot(*mm_to_px(H, p).T, "m-", lw=0.8)
    ax.legend()
    plt.savefig(fname, dpi=70)
    plt.close(fig)


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "wood"
    if cmd not in ("calibrate", "wood", "test"):
        sys.exit(__doc__)
    new_run(cmd)
    print("run directory:", OUT)
    with F1Ultra() as laser:
        if cmd == "calibrate":
            calibrate(laser)
        elif cmd == "wood":
            H, _ = load_calib()
            corners, quad_px, img = wood_mm(laser, H)
            save("photo.jpg", img)
            c, ex, ey, (w, h) = rect_frame(corners)
            print("workpiece corners mm:\n", corners.round(1))
            print(f"center {c.round(1)} size {w:.1f} x {h:.1f} mm, rotation {frame_rotation_deg(corners):.2f} deg")
            save("workpiece.json", {"corners_mm": corners, "corners_px": quad_px, "center": c, "size": [w, h],
                                    "rotation_deg": frame_rotation_deg(corners)})
            plot_alignment(img, H, corners, [], f"{OUT}/workpiece.png")
        elif cmd == "test":
            H, calib = load_calib()
            # photograph before autofocus: the camera is unreliable for a while after the head moves
            corners, quad_px, img = wood_mm(laser, H)
            save("photo_before.jpg", img)
            z = focus(laser, calib)
            residual_px = calib.get("residual_px") or _fit(calib["pairs"])[1].tolist()
            residual_mm = max(residual_px) / np.linalg.norm(np.diff(mm_to_px(H, [(0, 0), (1, 0)]), axis=0))
            paths = test_pattern(corners, z, residual_mm)
            c, ex, ey, (w, h) = rect_frame(corners)
            save("workpiece.json", {"corners_mm": corners, "corners_px": quad_px, "center": c, "size": [w, h],
                                    "rotation_deg": frame_rotation_deg(corners), "z_mm": z})
            save("paths_mm.json", paths)
            save("job.gcode", make_cut_gcode(paths, 80.0, 4500.0))
            plot_alignment(img, H, corners, paths, f"{OUT}/plan.png")
            print(f"engraving {len(paths)} paths, {sum(len(p) for p in paths)} points")
            laser.burn(paths, power=80.0, speed=4500.0)
            after = take_photo(laser)
            save("photo_after.jpg", after)
            plot_alignment(after, H, corners, [], f"{OUT}/result.png")
