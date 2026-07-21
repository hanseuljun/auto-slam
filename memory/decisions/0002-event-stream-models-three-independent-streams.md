---
name: event-stream-models-three-independent-streams
description: slam-dataset's EventStream merges cam0/cam1/imu0 as three separate time-sorted streams instead of assuming cam0/cam1 are index-paired stereo frames.
metadata:
  type: decision
---

# Decision: model cam0/cam1/imu0 as three independent streams, not a paired stereo stream

## Decision

`slam_dataset::EuRocSequence::events()` returns an `EventStream` that
three-way-merges `imu0`, `cam0`, and `cam1` as independent time-sorted
sequences (`Event::Imu(idx)` / `Event::Cam0(idx)` / `Event::Cam1(idx)`),
rather than assuming `cam0[i]` and `cam1[i]` are always the same trigger and
emitting a single combined `Event::Stereo(idx)`.

## Alternatives considered

- **Assume index-paired stereo frames** (`cam0[i]` always corresponds to
  `cam1[i]`): simpler downstream code in M3 (stereo matching), and true for
  most EuRoC `machine_hall` sequences (same count, identical timestamps).

## Why not the simpler alternative

Verified by hand (via `slam-inspect`) across all five MH sequences before
committing to a design: MH_04_difficult has 2033 `cam0` frames vs. 2032
`cam1` frames — one camera dropped a frame the other didn't. An
index-paired design would either panic or silently misalign the stereo pair
partway through that sequence. See `notes/dataset-quirks.md` for the raw
numbers.

## How to apply

M3 (stereo visual frontend) must pair `cam0`/`cam1` frames by nearest
timestamp when doing stereo matching, not by shared index. If a future
session is tempted to zip `cam0_frames`/`cam1_frames` by index for
convenience, that's only safe on sequences confirmed to have matching
counts — check first, don't assume.
