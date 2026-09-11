# cv_alignment_f1_ultra

Camera-based workpiece alignment for the xTool F1 Ultra over its LAN API,
plus a Python driver for the "V2" protocol that newer firmware (>= 40.52) uses.

Tested on firmware `40.52.016.2020.01.ht5` (device code GS002).

## What's here

| File | Purpose |
|---|---|
| `F1_ultra_driver.py` | LAN driver: TLS WebSocket on port 28900, CRC16 framed JSON, file transfer, camera snapshot, job upload/start, Z move, autofocus. Also builds `.xf` job packages from G-code. |
| `align.py` | CV tool: photograph the bed, segment the workpiece, calibrate camera to laser mm with burned marks, and fit patterns to the workpiece. |
| `F1-Ultra-template.xf` | Template job package; `make_xf` swaps in generated G-code. |
| `camera_calib.json` | Example 8-point homography (mm to px) from one setup. Recalibrate for your own machine and workpiece height. |

## Setup

```
pip install -r requirements.txt
```

Set the laser's IP in `F1_ultra_driver.py` (`HOST`).

## Usage

```
python align.py calibrate   # burns 8 tiny marks on the workpiece, writes camera_calib.json
python align.py wood        # photo + workpiece corners / size / rotation in laser mm
python align.py test        # autofocus, then engrave corner brackets, a nested-squares whirl and a stats block
```

`calibrate` and `test` fire the laser. Focus first (`F1Ultra().autofocus()` or the touchscreen),
keep the lid closed, and be next to the machine.

From Python:

```python
import F1_ultra_driver as f1, align

with f1.F1Ultra() as laser:
    laser.autofocus()                       # returns measured Z in mm
    H, _ = align.load_calib()
    corners, quad_px, img = align.wood_mm(laser, H)
    paths = align.align_paths(my_paths_mm, corners, margin=5)   # scale/rotate/translate into the workpiece
    laser.burn(paths, power=80, speed=4500)
```

## Run records

Every invocation writes `runs/<time>_<command>_<git hash>/` with the photos, plan and result
plots, the paths and G-code sent, a copy of the calibration, and `meta.json` holding the exact
commit (plus the diff if the tree was dirty). `runs/` is git-ignored; commit selectively.

## How calibration works

1. Four 3 mm hatched squares are burned in a small cluster near the bed centre. Each is found
   as the one new dark blob compared with the previous photo, so nothing is assumed about
   camera orientation (on this machine laser +x is image-left and +y is image-up).
2. A first homography predicts the workpiece corners; four more marks go 15 mm inside them.
3. All eight correspondences are refit. Residuals were 0.13 mm max on a 150 mm plywood square.

The homography is only valid for one workpiece surface height. Recalibrate when the
material thickness changes.

## Machine quirks worth knowing

- The fill light switches off after every job; `align.take_photo` re-enables it. Brightness 30
  is the only level that doesn't saturate light wood in the camera's auto-exposure.
- Autofocus only works after switching the device mode to `P_AUTOFOCUS`, with a moment for the mode
  to settle; it takes about 30 s and resets the fill light.
- With the lid open the fill light is off and the camera returns black frames; the tool refuses to
  photograph until the lid sensor reads closed.
- The camera returns 2592x1944 or 4656x3496 depending on mood; the tool normalises to 2592 wide.
- Protocol reference: https://github.com/thecodingdad/ha-xtool (docs/PROTOCOL.md).
