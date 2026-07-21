use image::GrayImage;

use crate::pyramid::ImagePyramid;

#[derive(Debug, Clone, Copy)]
pub struct LkParams {
    /// Half-width of the tracking window (a window is `2*radius+1` square).
    pub window_radius: i32,
    pub max_iterations: usize,
    /// Convergence threshold on the per-iteration displacement update.
    pub epsilon: f32,
    /// Reject a track if the structure matrix's determinant falls below
    /// this (a cheap proxy for "both eigenvalues are large enough to
    /// constrain the flow" — the classic KLT trackability criterion).
    pub min_determinant: f32,
}

impl Default for LkParams {
    fn default() -> Self {
        LkParams {
            window_radius: 7,
            max_iterations: 20,
            epsilon: 0.01,
            min_determinant: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TrackResult {
    pub x: f32,
    pub y: f32,
    pub found: bool,
}

/// Tracks each point in `prev_points` (level-0 pixel coordinates in
/// `prev_pyramid`) into `curr_pyramid`, via pyramidal (coarse-to-fine)
/// Lucas-Kanade-Tomasi optical flow.
pub fn track_pyramid(
    prev_pyramid: &ImagePyramid,
    curr_pyramid: &ImagePyramid,
    prev_points: &[(f32, f32)],
    params: &LkParams,
) -> Vec<TrackResult> {
    prev_points
        .iter()
        .map(|&p| track_single_point(prev_pyramid, curr_pyramid, p, params))
        .collect()
}

fn track_single_point(
    prev_pyramid: &ImagePyramid,
    curr_pyramid: &ImagePyramid,
    point_level0: (f32, f32),
    params: &LkParams,
) -> TrackResult {
    let num_levels = prev_pyramid.levels.len().min(curr_pyramid.levels.len());
    let mut disp = (0.0f32, 0.0f32);
    let mut found = true;

    for level in (0..num_levels).rev() {
        let scale = 0.5f32.powi(level as i32);
        let point_at_level = (point_level0.0 * scale, point_level0.1 * scale);
        disp = (disp.0 * 2.0, disp.1 * 2.0);

        match lk_iterate_level(
            &prev_pyramid.levels[level],
            &curr_pyramid.levels[level],
            point_at_level,
            disp,
            params,
        ) {
            Some(refined) => disp = refined,
            // A level's window falling outside its (possibly tiny) coarse
            // image, or being locally textureless there, doesn't mean the
            // point is untrackable overall — keep propagating the current
            // displacement guess to finer levels. Only the finest level
            // (level 0) failing marks the track lost.
            None if level == 0 => found = false,
            None => {}
        }
    }

    TrackResult {
        x: point_level0.0 + disp.0,
        y: point_level0.1 + disp.1,
        found,
    }
}

/// One pyramid level's worth of forward-additive Lucas-Kanade-Tomasi
/// refinement: the template gradients/structure matrix are computed once
/// from `prev_img` (they don't change across iterations), and each
/// iteration only resamples `curr_img` at the current displacement guess.
fn lk_iterate_level(
    prev_img: &GrayImage,
    curr_img: &GrayImage,
    prev_point: (f32, f32),
    mut disp: (f32, f32),
    params: &LkParams,
) -> Option<(f32, f32)> {
    let r = params.window_radius;
    let mut gxx = 0.0f32;
    let mut gxy = 0.0f32;
    let mut gyy = 0.0f32;
    let mut window = Vec::with_capacity(((2 * r + 1) * (2 * r + 1)) as usize);

    for wy in -r..=r {
        for wx in -r..=r {
            let x = prev_point.0 + wx as f32;
            let y = prev_point.1 + wy as f32;
            let ix = 0.5 * (sample_bilinear(prev_img, x + 1.0, y)? - sample_bilinear(prev_img, x - 1.0, y)?);
            let iy = 0.5 * (sample_bilinear(prev_img, x, y + 1.0)? - sample_bilinear(prev_img, x, y - 1.0)?);
            let template_val = sample_bilinear(prev_img, x, y)?;
            gxx += ix * ix;
            gxy += ix * iy;
            gyy += iy * iy;
            window.push((wx as f32, wy as f32, ix, iy, template_val));
        }
    }

    let det = gxx * gyy - gxy * gxy;
    if det.abs() < params.min_determinant {
        return None;
    }

    for _ in 0..params.max_iterations {
        let mut bx = 0.0f32;
        let mut by = 0.0f32;
        for &(wx, wy, ix, iy, template_val) in &window {
            let cx = prev_point.0 + wx + disp.0;
            let cy = prev_point.1 + wy + disp.1;
            let curr_val = sample_bilinear(curr_img, cx, cy)?;
            let diff = template_val - curr_val;
            bx += ix * diff;
            by += iy * diff;
        }

        let delta_x = (gyy * bx - gxy * by) / det;
        let delta_y = (gxx * by - gxy * bx) / det;
        disp.0 += delta_x;
        disp.1 += delta_y;

        if delta_x * delta_x + delta_y * delta_y < params.epsilon * params.epsilon {
            break;
        }
    }

    Some(disp)
}

/// Bilinear sample at `(x, y)`; `None` if the 1px-padded footprint needed
/// for gradients would fall outside the image.
fn sample_bilinear(img: &GrayImage, x: f32, y: f32) -> Option<f32> {
    if x < 1.0 || y < 1.0 || x > (img.width() as f32 - 2.0) || y > (img.height() as f32 - 2.0) {
        return None;
    }
    let x0 = x.floor();
    let y0 = y.floor();
    let fx = x - x0;
    let fy = y - y0;
    let (x0, y0) = (x0 as u32, y0 as u32);

    let p00 = img.get_pixel(x0, y0).0[0] as f32;
    let p10 = img.get_pixel(x0 + 1, y0).0[0] as f32;
    let p01 = img.get_pixel(x0, y0 + 1).0[0] as f32;
    let p11 = img.get_pixel(x0 + 1, y0 + 1).0[0] as f32;

    Some(
        p00 * (1.0 - fx) * (1.0 - fy)
            + p10 * fx * (1.0 - fy)
            + p01 * (1.0 - fx) * fy
            + p11 * fx * fy,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A synthetic image with a sharp bright square on a dark background —
    /// textured enough to be trackable, and we know the exact translation
    /// we apply, so the test can assert on true sub-pixel accuracy.
    fn textured_image(offset_x: i32, offset_y: i32) -> GrayImage {
        let mut img = GrayImage::new(100, 100);
        for p in img.pixels_mut() {
            *p = image::Luma([40]);
        }
        for y in 0..100i32 {
            for x in 0..100i32 {
                let sx = x - offset_x;
                let sy = y - offset_y;
                if (20..60).contains(&sx) && (20..60).contains(&sy) {
                    img.put_pixel(x as u32, y as u32, image::Luma([220]));
                }
            }
        }
        img
    }

    #[test]
    fn tracks_a_known_integer_translation() {
        let prev = textured_image(0, 0);
        let curr = textured_image(5, -3);
        let prev_pyr = ImagePyramid::build(&prev, 3);
        let curr_pyr = ImagePyramid::build(&curr, 3);

        // Points near the square's *corners*, where both x and y gradients
        // are informative for LK — a point on a straight edge (e.g. the
        // midpoint of a side) has the classic aperture problem: the
        // structure matrix is singular along the edge direction.
        let prev_points = [(23.0f32, 23.0f32), (57.0, 23.0), (23.0, 57.0)];
        let results = track_pyramid(&prev_pyr, &curr_pyr, &prev_points, &LkParams::default());

        for (result, &(px, py)) in results.iter().zip(prev_points.iter()) {
            assert!(result.found, "expected track to succeed");
            assert!((result.x - (px + 5.0)).abs() < 0.5, "x drifted: {}", result.x);
            assert!((result.y - (py - 3.0)).abs() < 0.5, "y drifted: {}", result.y);
        }
    }

    #[test]
    fn rejects_tracking_in_a_flat_region() {
        let mut img = GrayImage::new(60, 60);
        for p in img.pixels_mut() {
            *p = image::Luma([100]);
        }
        let pyr = ImagePyramid::build(&img, 2);
        let results = track_pyramid(&pyr, &pyr, &[(30.0, 30.0)], &LkParams::default());
        assert!(!results[0].found, "a flat, untextured region should not be trackable");
    }

    #[test]
    fn sample_bilinear_matches_exact_pixel_values() {
        let mut img = GrayImage::new(10, 10);
        img.put_pixel(5, 5, image::Luma([200]));
        let sampled = sample_bilinear(&img, 5.0, 5.0).unwrap();
        assert!((sampled - 200.0).abs() < 1e-6);
    }
}
