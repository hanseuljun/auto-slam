use image::GrayImage;

/// A detected corner: pixel position plus a strength score (higher = more
/// corner-like), used both for grid-based non-max suppression and for
/// ranking within a cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Keypoint {
    pub x: f32,
    pub y: f32,
    pub score: f32,
}

/// The 16-pixel Bresenham circle of radius 3 used by FAST, as (dx, dy)
/// offsets in circular order.
const CIRCLE_OFFSETS: [(i32, i32); 16] = [
    (0, -3), (1, -3), (2, -2), (3, -1),
    (3, 0), (3, 1), (2, 2), (1, 3),
    (0, 3), (-1, 3), (-2, 2), (-3, 1),
    (-3, 0), (-3, -1), (-2, -2), (-1, -3),
];

/// Minimum contiguous arc length (FAST-9) of the 16-pixel circle that must
/// be uniformly brighter or darker than the center (by more than
/// `threshold`) for a pixel to be a corner.
const FAST_N: usize = 9;

fn is_corner(intensities: &[i16; 16], center: i16, threshold: i16) -> Option<f32> {
    let brighter: [bool; 16] = std::array::from_fn(|i| intensities[i] > center + threshold);
    let darker: [bool; 16] = std::array::from_fn(|i| intensities[i] < center - threshold);

    let has_run_of_at_least = |arr: &[bool; 16], n: usize| -> bool {
        let mut max_run = 0;
        let mut current = 0;
        for &b in arr.iter().chain(arr.iter()) {
            if b {
                current += 1;
                max_run = max_run.max(current);
            } else {
                current = 0;
            }
            if max_run >= n {
                return true;
            }
        }
        false
    };

    if has_run_of_at_least(&brighter, FAST_N) || has_run_of_at_least(&darker, FAST_N) {
        let score = intensities.iter().map(|&v| (v - center).unsigned_abs() as f32).sum();
        Some(score)
    } else {
        None
    }
}

/// Runs FAST-9 over the whole image (excluding the 3px border needed by the
/// circle), returning every candidate corner with its score. Unfiltered —
/// callers typically want [`detect_grid`] for an evenly-distributed subset.
pub fn detect_fast(image: &GrayImage, threshold: u8) -> Vec<Keypoint> {
    let (w, h) = image.dimensions();
    let threshold = threshold as i16;
    let mut keypoints = Vec::new();
    if w < 7 || h < 7 {
        return keypoints;
    }
    for y in 3..(h - 3) {
        for x in 3..(w - 3) {
            let center = image.get_pixel(x, y).0[0] as i16;
            let intensities: [i16; 16] = std::array::from_fn(|i| {
                let (dx, dy) = CIRCLE_OFFSETS[i];
                image.get_pixel((x as i32 + dx) as u32, (y as i32 + dy) as u32).0[0] as i16
            });
            if let Some(score) = is_corner(&intensities, center, threshold) {
                keypoints.push(Keypoint {
                    x: x as f32,
                    y: y as f32,
                    score,
                });
            }
        }
    }
    keypoints
}

/// Detects FAST corners and buckets them into a `cell_size x cell_size`
/// pixel grid, keeping only the `max_per_cell` highest-scoring corners in
/// each cell. This is what gives an even spatial distribution instead of
/// clustering in high-texture regions (the standard grid-based approach
/// used by ORB-SLAM-class frontends).
pub fn detect_grid(image: &GrayImage, threshold: u8, cell_size: u32, max_per_cell: usize) -> Vec<Keypoint> {
    let candidates = detect_fast(image, threshold);
    if candidates.is_empty() || cell_size == 0 || max_per_cell == 0 {
        return Vec::new();
    }

    let cols = image.width().div_ceil(cell_size).max(1) as usize;
    let mut cells: Vec<Vec<Keypoint>> = vec![Vec::new(); cols * (image.height().div_ceil(cell_size).max(1) as usize)];

    for kp in candidates {
        let col = (kp.x as u32 / cell_size) as usize;
        let row = (kp.y as u32 / cell_size) as usize;
        cells[row * cols + col].push(kp);
    }

    let mut selected = Vec::new();
    for mut cell in cells {
        cell.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        cell.truncate(max_per_cell);
        selected.extend(cell);
    }
    selected
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A synthetic checkerboard-corner pattern: FAST should fire on the
    /// corner of the bright/dark square boundary and nowhere in the flat
    /// interior of either square.
    fn checkerboard_corner_image() -> GrayImage {
        let mut img = GrayImage::new(40, 40);
        for y in 0..40 {
            for x in 0..40 {
                let bright = x >= 20 && y >= 20;
                img.put_pixel(x, y, image::Luma([if bright { 220 } else { 30 }]));
            }
        }
        img
    }

    #[test]
    fn detects_the_checkerboard_corner() {
        let img = checkerboard_corner_image();
        let corners = detect_fast(&img, 30);
        assert!(!corners.is_empty(), "expected at least one corner");
        // Every detection should be near the (20, 20) corner, not scattered
        // into the flat interiors.
        for kp in &corners {
            let dist = ((kp.x - 20.0).powi(2) + (kp.y - 20.0).powi(2)).sqrt();
            assert!(dist < 5.0, "corner at ({}, {}) too far from the real corner", kp.x, kp.y);
        }
    }

    #[test]
    fn flat_image_has_no_corners() {
        let mut img = GrayImage::new(30, 30);
        for p in img.pixels_mut() {
            *p = image::Luma([128]);
        }
        assert!(detect_fast(&img, 20).is_empty());
    }

    #[test]
    fn grid_detection_caps_keypoints_per_cell() {
        // A busy random-noise image has many raw FAST candidates; grid
        // detection must cap however many survive per cell.
        let mut img = GrayImage::new(100, 100);
        let mut state: u32 = 12345;
        for p in img.pixels_mut() {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            *p = image::Luma([(state >> 24) as u8]);
        }
        let cell_size = 20;
        let max_per_cell = 3;
        let kps = detect_grid(&img, 15, cell_size, max_per_cell);

        let cols = img.width().div_ceil(cell_size) as usize;
        let rows = img.height().div_ceil(cell_size) as usize;
        let mut counts = vec![0usize; cols * rows];
        for kp in &kps {
            let col = (kp.x as u32 / cell_size) as usize;
            let row = (kp.y as u32 / cell_size) as usize;
            counts[row * cols + col] += 1;
        }
        assert!(counts.iter().all(|&c| c <= max_per_cell));
    }
}
