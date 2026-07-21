use crate::sequence::{CameraFrame, ImuSample};

/// One event in the merged, time-ordered stream over a sequence's raw data.
/// The payload is an index into the corresponding `Vec` on `EuRocSequence`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    Imu(usize),
    Cam0(usize),
    Cam1(usize),
}

/// Lazily merges the three time-sorted streams (`imu0`, `cam0`, `cam1`) into
/// a single time-ordered iterator, via a three-way merge (each stream is
/// already sorted, so this is O(n) with no allocation beyond the cursors).
pub struct EventStream<'a> {
    imu: &'a [ImuSample],
    cam0: &'a [CameraFrame],
    cam1: &'a [CameraFrame],
    imu_cursor: usize,
    cam0_cursor: usize,
    cam1_cursor: usize,
}

impl<'a> EventStream<'a> {
    pub(crate) fn new(imu: &'a [ImuSample], cam0: &'a [CameraFrame], cam1: &'a [CameraFrame]) -> Self {
        EventStream {
            imu,
            cam0,
            cam1,
            imu_cursor: 0,
            cam0_cursor: 0,
            cam1_cursor: 0,
        }
    }
}

impl<'a> Iterator for EventStream<'a> {
    /// `(timestamp_ns, event)`.
    type Item = (u64, Event);

    fn next(&mut self) -> Option<Self::Item> {
        // Tie-break order (Imu < Cam0 < Cam1) is arbitrary but fixed, for
        // deterministic replay when timestamps coincide.
        let candidates = [
            self.imu
                .get(self.imu_cursor)
                .map(|s| (s.timestamp_ns, 0u8)),
            self.cam0
                .get(self.cam0_cursor)
                .map(|f| (f.timestamp_ns, 1u8)),
            self.cam1
                .get(self.cam1_cursor)
                .map(|f| (f.timestamp_ns, 2u8)),
        ];
        let (timestamp_ns, stream) = candidates.into_iter().flatten().min()?;
        let event = match stream {
            0 => {
                let idx = self.imu_cursor;
                self.imu_cursor += 1;
                Event::Imu(idx)
            }
            1 => {
                let idx = self.cam0_cursor;
                self.cam0_cursor += 1;
                Event::Cam0(idx)
            }
            _ => {
                let idx = self.cam1_cursor;
                self.cam1_cursor += 1;
                Event::Cam1(idx)
            }
        };
        Some((timestamp_ns, event))
    }
}
