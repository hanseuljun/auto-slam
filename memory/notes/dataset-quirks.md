# Dataset quirks — EuRoC `machine_hall`

Living notes on the dataset in `data/machine_hall/`. Add to this as more is
learned; don't rewrite wholesale.

## Layout (confirmed 2026-07-20)

Each sequence (`MH_01_easy` .. `MH_05_difficult`) has `mav0/` with:

- `cam0/`, `cam1/` — stereo pair, 752x480, 20 Hz, PNGs named by nanosecond
  timestamp, indexed in `data.csv`. `sensor.yaml` per camera: `T_BS`
  (extrinsic to body frame), pinhole `intrinsics: [fu, fv, cu, cv]`,
  `distortion_model: radial-tangential` with 4 coefficients (no k3).
- `imu0/` — ADIS16448, 200 Hz, `data.csv` columns
  `[t, wx, wy, wz, ax, ay, az]`. `sensor.yaml` gives gyro/accel noise
  density and random-walk (bias diffusion), and `T_BS` = identity (IMU
  defines the body frame — cam extrinsics are relative to it).
- `leica0/` — sparse Leica MS50 total-station position fixes, prism offset
  in `sensor.yaml`. Not the primary ground truth source.
- `state_groundtruth_estimate0/` — the ground truth to evaluate against:
  `[t, p_RS_R(xyz), q_RS(wxyz), v_RS_R, b_w, b_a]`, high rate. Lives in a
  Vicon/Leica world frame that has no fixed relationship to whatever world
  frame the SLAM system will produce — evaluation must align trajectories
  (similarity/SE3, e.g. Umeyama) before computing ATE, never compare raw
  coordinates.
- `body.yaml` — cosmetic only (MAV name).

## Gotchas to design around

- **Timestamps aren't trivially 1:1 across streams.** cam0/cam1/imu0/
  groundtruth all have independent timestamp columns; need real
  nearest-neighbor or interpolated lookup by timestamp, not index alignment.
- **Sequence starts differ.** MH_01/02/03 start with the MAV stationary —
  usable for a simple static IMU-bias/gravity initializer. MH_04/05 start
  already in motion — need the dynamic vision-IMU alignment initializer as
  a fallback (see `plan/STAGE1.md` M4). Don't assume the static path works
  on all five sequences.
- **`data/` is gitignored.** The dataset itself is not (and should not be)
  committed — `.gitignore` at repo root contains just `data`. Any tooling
  that needs to know sequence paths should take them as arguments/config,
  not hardcode assumptions that only hold locally.
- **cam0/cam1 file counts**: MH_01_easy has 3682 frames in `cam0/data/` as
  of the 2026-07-20 check — a reasonable sanity-check number for a "did the
  loader read everything" test, but re-verify per-sequence rather than
  assuming all five match.
