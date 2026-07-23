use std::time::Instant;

use slam_dataset::EuRocSequence;

/// `plan/STAGE3.md` M4's video panel: `cam0` frames synced to a playback
/// time/frame index, with a scrub bar and play/pause. Lazily loads the
/// selected run's `EuRocSequence` from `data_dir/<sequence_name>/mav0` —
/// a run's `trajectory.csv` only carries timestamps/positions
/// (`slam_eval::TrajectoryPoints`), not the raw frames themselves, so
/// this is a second, independent load from the dataset a run was
/// originally computed against.
pub struct VideoPlayer {
    data_dir: std::path::PathBuf,
    sequence: Option<EuRocSequence>,
    /// The loaded run's per-keyframe timestamps (`plan/STAGE3.md` M4's
    /// "synced to a playback time" — the scrub bar moves through *this*
    /// index space, one entry per keyframe in the trajectory, not raw
    /// `cam0` frame indices, since a run's keyframes are already the
    /// natural playback granularity a user scrubbing a trajectory cares
    /// about).
    timestamps: Vec<u64>,
    scrub_index: usize,
    playing: bool,
    last_advance: Instant,
    error: Option<String>,
}

impl VideoPlayer {
    pub fn new(data_dir: std::path::PathBuf) -> Self {
        VideoPlayer { data_dir, sequence: None, timestamps: Vec::new(), scrub_index: 0, playing: false, last_advance: Instant::now(), error: None }
    }

    /// Called when a new run is selected (`App::select_run`): loads (or
    /// reports failure to load) `sequence_name`'s raw dataset frames, and
    /// resets playback to the run's own timestamp range.
    pub fn load_for_run(&mut self, sequence_name: &str, timestamps: Vec<u64>) {
        self.timestamps = timestamps;
        self.scrub_index = 0;
        self.playing = false;
        let mav0 = self.data_dir.join(sequence_name).join("mav0");
        match EuRocSequence::load(&mav0) {
            Ok(seq) => {
                self.sequence = Some(seq);
                self.error = None;
            }
            Err(e) => {
                self.sequence = None;
                self.error = Some(format!("no dataset frames at {} ({e})", mav0.display()));
            }
        }
    }

    /// The `cam0` frame index nearest the current scrub position's
    /// timestamp, via `slam_dataset`'s own sync lookup (`plan/STAGE3.md`
    /// M4: "reuses `slam-dataset`'s existing timestamp/frame-index
    /// lookup" — added there this milestone, since it didn't exist yet).
    fn current_cam0_index(&self) -> Option<usize> {
        let seq = self.sequence.as_ref()?;
        let timestamp = *self.timestamps.get(self.scrub_index)?;
        Some(seq.nearest_cam0_frame_index(timestamp))
    }

    /// The current playback position, in the same per-keyframe index
    /// space as `timestamps`/`load_for_run` — the shared cursor `plan/
    /// STAGE3.md` M6's synced playback reads to highlight the matching
    /// position in the 3D and graphs panels too.
    pub fn scrub_index(&self) -> usize {
        self.scrub_index
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("Video");
        if let Some(err) = &self.error {
            ui.colored_label(egui::Color32::YELLOW, err);
            return;
        }
        if self.timestamps.is_empty() || self.sequence.is_none() {
            ui.label("No run selected.");
            return;
        }

        let max_index = self.timestamps.len() - 1;
        ui.horizontal(|ui| {
            let play_label = if self.playing { "Pause" } else { "Play" };
            if ui.button(play_label).clicked() {
                self.playing = !self.playing;
                self.last_advance = Instant::now();
            }
            ui.add(egui::Slider::new(&mut self.scrub_index, 0..=max_index).text("keyframe"));
        });

        if self.playing {
            // ~10 keyframes/sec playback - fast enough to feel like
            // "playing," slow enough to actually see individual frames,
            // not tied to the original sequence's real capture rate
            // (this is scrubbing through *keyframes*, already a subsampled
            // view of the raw ~20Hz cam0 stream).
            if self.last_advance.elapsed().as_secs_f64() > 0.1 {
                self.last_advance = Instant::now();
                if self.scrub_index >= max_index {
                    self.playing = false;
                } else {
                    self.scrub_index += 1;
                }
            }
            ui.ctx().request_repaint();
        }

        let Some(cam0_index) = self.current_cam0_index() else {
            ui.label("No frame at this position.");
            return;
        };
        let seq = self.sequence.as_ref().unwrap();
        match seq.load_cam0_image(cam0_index) {
            Ok(gray) => {
                let (w, h) = (gray.width() as usize, gray.height() as usize);
                let rgba: Vec<u8> = gray.into_raw().into_iter().flat_map(|v| [v, v, v, 255]).collect();
                let image = egui::ColorImage::from_rgba_unmultiplied([w, h], &rgba);
                let texture = ui.ctx().load_texture("slam-viz-video-frame", image, egui::TextureOptions::LINEAR);
                ui.add(egui::Image::new(&texture).max_width(ui.available_width()));
                ui.label(format!("cam0 frame {cam0_index}/{}", seq.cam0_frames.len().saturating_sub(1)));
            }
            Err(e) => {
                ui.colored_label(egui::Color32::RED, format!("failed to decode frame {cam0_index}: {e}"));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // `cargo test`'s working directory is this crate's own manifest
    // directory (`bin/slam-viz/`), not the workspace root - resolve
    // relative to `CARGO_MANIFEST_DIR` instead, same convention
    // `slam-dataset`'s own tests already use.
    fn data_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/machine_hall")
    }

    #[test]
    fn loads_a_real_sequence_and_syncs_to_the_nearest_cam0_frame() {
        let mut player = VideoPlayer::new(data_dir());
        // Real MH_01_easy data, same fixture `slam-dataset`'s own tests
        // use — a genuine end-to-end check, not a synthetic stand-in.
        let seq = EuRocSequence::load(data_dir().join("MH_01_easy/mav0")).expect("real dataset must load for this test to be meaningful");
        let some_real_timestamp = seq.cam0_frames[10].timestamp_ns;

        player.load_for_run("MH_01_easy", vec![some_real_timestamp]);
        assert!(player.error.is_none(), "expected the real sequence to load without error: {:?}", player.error);
        assert!(player.sequence.is_some());
        assert_eq!(player.current_cam0_index(), Some(10));
    }

    #[test]
    fn missing_sequence_directory_sets_an_error_not_a_panic() {
        let mut player = VideoPlayer::new(data_dir());
        player.load_for_run("NOT_A_REAL_SEQUENCE", vec![0]);
        assert!(player.error.is_some());
        assert!(player.sequence.is_none());
        assert_eq!(player.current_cam0_index(), None);
    }
}
