use image::GrayImage;

/// A 256-bit BRIEF-style binary descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Descriptor(pub [u64; 4]);

impl Descriptor {
    pub fn hamming_distance(&self, other: &Descriptor) -> u32 {
        self.0.iter().zip(other.0.iter()).map(|(a, b)| (a ^ b).count_ones()).sum()
    }
}

const NUM_BITS: usize = 256;
const PATCH_RADIUS: i32 = 15;

type OffsetPair = ((i32, i32), (i32, i32));

/// A fixed, deterministic sampling pattern of 256 pixel-pair offsets within
/// a `[-PATCH_RADIUS, PATCH_RADIUS]` patch — the same pattern every call,
/// generated once from a fixed seed (not true BRIEF's Gaussian-sampled
/// pattern, but the same idea: fixed, reproducible, spatially spread
/// comparisons — what matters for a binary descriptor is that the pattern
/// is *fixed*, so the same keypoint always produces the same bits).
fn sampling_pattern() -> &'static [OffsetPair; NUM_BITS] {
    use std::sync::OnceLock;
    static PATTERN: OnceLock<[OffsetPair; NUM_BITS]> = OnceLock::new();
    PATTERN.get_or_init(|| {
        let mut state: u32 = 0xB812_9C07;
        let mut next = || {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            state
        };
        let mut next_offset = || {
            let r = next();
            let dx = ((r & 0xFFFF) % (2 * PATCH_RADIUS as u32 + 1)) as i32 - PATCH_RADIUS;
            let dy = (((r >> 16) & 0xFFFF) % (2 * PATCH_RADIUS as u32 + 1)) as i32 - PATCH_RADIUS;
            (dx, dy)
        };
        std::array::from_fn(|_| (next_offset(), next_offset()))
    })
}

/// Computes a BRIEF-style descriptor at `(x, y)` in `image`, or `None` if
/// the sampling patch would fall outside the image.
pub fn compute_descriptor(image: &GrayImage, x: f32, y: f32) -> Option<Descriptor> {
    let (w, h) = image.dimensions();
    let margin = PATCH_RADIUS as f32 + 1.0;
    if x < margin || y < margin || x > w as f32 - margin || y > h as f32 - margin {
        return None;
    }

    let sample = |dx: i32, dy: i32| -> u8 {
        let px = (x + dx as f32).round() as u32;
        let py = (y + dy as f32).round() as u32;
        image.get_pixel(px, py).0[0]
    };

    let mut bits = [0u64; 4];
    for (i, &(a, b)) in sampling_pattern().iter().enumerate() {
        if sample(a.0, a.1) < sample(b.0, b.1) {
            bits[i / 64] |= 1 << (i % 64);
        }
    }
    Some(Descriptor(bits))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checkerboard() -> GrayImage {
        let mut img = GrayImage::new(80, 80);
        for y in 0..80 {
            for x in 0..80 {
                let bright = ((x / 10) + (y / 10)) % 2 == 0;
                img.put_pixel(x, y, image::Luma([if bright { 220 } else { 30 }]));
            }
        }
        img
    }

    #[test]
    fn identical_patches_have_zero_hamming_distance() {
        let img = checkerboard();
        let a = compute_descriptor(&img, 40.0, 40.0).unwrap();
        let b = compute_descriptor(&img, 40.0, 40.0).unwrap();
        assert_eq!(a.hamming_distance(&b), 0);
    }

    #[test]
    fn distinct_patches_differ() {
        // The checkerboard is periodic with a 20px period, so points a
        // multiple of 20px apart (e.g. (20,20) and (60,60)) have
        // *identical* local neighborhoods — not a useful "distinct
        // patches" test. Use an offset that isn't a period multiple.
        let img = checkerboard();
        let a = compute_descriptor(&img, 20.0, 20.0).unwrap();
        let b = compute_descriptor(&img, 55.0, 45.0).unwrap();
        // Not a strict correctness bound (no guaranteed minimum distance
        // for arbitrary content), just confirms the descriptor is
        // actually content-sensitive, not a constant.
        assert!(a.hamming_distance(&b) > 10, "descriptors suspiciously similar: {}", a.hamming_distance(&b));
    }

    #[test]
    fn near_border_returns_none() {
        let img = checkerboard();
        assert!(compute_descriptor(&img, 1.0, 1.0).is_none());
        assert!(compute_descriptor(&img, 79.0, 79.0).is_none());
    }

    #[test]
    fn hamming_distance_matches_manual_popcount() {
        let a = Descriptor([0b1010, 0, 0, 0]);
        let b = Descriptor([0b0110, 0, 0, 0]);
        // XOR = 0b1100, popcount = 2.
        assert_eq!(a.hamming_distance(&b), 2);
    }
}
