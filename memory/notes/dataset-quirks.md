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
- **Sequence starts differ — but "MH_01/02/03 start stationary" is *not*
  the same as "sample 0 is stationary."** Corrected 2026-07-21: MH_01's
  groundtruth speed at its very first covered timestamp is already ~0.8
  m/s, and raw `imu0` gyro norms exceed 0.1 rad/s within the first couple
  hundred samples — index 0 is mid-handling, not at rest. The actual
  genuinely-still window (gyro norm consistently < ~0.09 rad/s, accel norm
  ≈ 9.78) starts around **sample ~4500-5300 of `imu0`, i.e. ~22-26.5s into
  the recording**, not at t=0. `slam_imu::find_stationary_window` scans
  for this rather than assuming any fixed offset — don't hardcode "first N
  samples" for a static initializer on any sequence without checking.
  MH_04/05 are still the ones that never settle at all (need the dynamic
  vision-IMU alignment initializer, `plan/STAGE1.md` M4's fallback path).
- **The ADIS16448's raw (factory-uncalibrated) gyro bias is large** —
  empirically ~0.08 rad/s (≈4.6°/s) on MH_01's z-axis, consistent across
  multiple genuinely-stationary windows at different times in the
  recording (so it's a real sustained bias, not noise/settling). Don't
  assume a "near-zero" gyro bias as a sanity bound in tests or
  initialization code; bound checks should allow for this.
- **`data/` is gitignored.** The dataset itself is not (and should not be)
  committed — `.gitignore` at repo root contains just `data`. Any tooling
  that needs to know sequence paths should take them as arguments/config,
  not hardcode assumptions that only hold locally.
- **cam0/cam1 file counts**: MH_01_easy has 3682 frames in `cam0/data/` as
  of the 2026-07-20 check — a reasonable sanity-check number for a "did the
  loader read everything" test, but re-verify per-sequence rather than
  assuming all five match.
- **cam0/cam1 are NOT always paired 1:1.** Confirmed 2026-07-20 via
  `slam-inspect`: MH_01/02/03/05 have matching cam0/cam1 counts and
  identical per-frame timestamps, but **MH_04_difficult has 2033 cam0
  frames vs. 2032 cam1 frames** — one camera dropped a frame the other
  didn't. Don't design the frontend around an assumed stereo-pair index
  alignment; `slam_dataset::EuRocSequence` treats cam0/cam1/imu0 as three
  independent time-sorted streams (see `EventStream` in
  `crates/slam-dataset/src/events.rs`) precisely because of this. Stereo
  matching downstream must pair frames by nearest timestamp, not by index.
- **IMU timestamps jitter around the nominal 200 Hz rate**, not exactly
  5,000,000 ns apart (observed deltas like 5,000,192 ns on MH_01). Any test
  or code asserting IMU cadence needs a tolerance, not exact equality.
