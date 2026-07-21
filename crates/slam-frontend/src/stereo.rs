use image::GrayImage;
use nalgebra::{Vector2, Vector3};
use slam_geometry::{StereoRectification, StereoRig};
use slam_vision::Keypoint;

#[derive(Debug, Clone, Copy)]
pub struct StereoMatchParams {
    /// Half-width of the patch used for SSD correlation.
    pub patch_radius: i32,
    pub min_disparity: f64,
    pub max_disparity: f64,
    pub disparity_step: f64,
    /// Reject a match if the best SSD-per-pixel exceeds this.
    pub max_ssd_per_pixel: f64,
    pub max_depth: f64,
}

impl Default for StereoMatchParams {
    fn default() -> Self {
        StereoMatchParams {
            patch_radius: 5,
            min_disparity: 1.0,
            max_disparity: 128.0,
            disparity_step: 1.0,
            max_ssd_per_pixel: 700.0,
            max_depth: 40.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StereoMatch {
    pub left_pixel: (f32, f32),
    /// Triangulated 3D point in cam0's raw (unrectified) frame.
    pub point_cam0: Vector3<f64>,
}

/// Matches left keypoints against the right image along the rectified
/// epipolar line (same row, varying disparity), via patch SSD correlation
/// sampled directly from the *raw* images at each disparity candidate's
/// mapped-back raw pixel (no full-image rectification remap — EuRoC's
/// cam0/cam1 are close to parallel, so raw-space patches are already a
/// good local approximation of rectified ones; see
/// `memory/decisions` if this needs revisiting for higher accuracy).
pub fn match_stereo_keypoints(
    left_img: &GrayImage,
    right_img: &GrayImage,
    keypoints: &[Keypoint],
    rig: &StereoRig,
    rect: &StereoRectification,
    params: &StereoMatchParams,
) -> Vec<StereoMatch> {
    let [fu, fv, cu, cv] = rect.rectified_intrinsics;
    let num_steps = ((params.max_disparity - params.min_disparity) / params.disparity_step).round() as i32;

    let mut matches = Vec::new();
    for kp in keypoints {
        let normalized_left = rig.cam0.unproject_to_normalized(Vector2::new(kp.x as f64, kp.y as f64));
        let rect_left = rect.project_left(Vector3::new(normalized_left.x, normalized_left.y, 1.0));

        let mut ssd_by_step = Vec::with_capacity((num_steps + 1) as usize);
        let mut best_ssd = f64::INFINITY;
        let mut best_step = 0i32;

        for step in 0..=num_steps {
            let d = params.min_disparity + step as f64 * params.disparity_step;
            let rect_right = Vector2::new(rect_left.x - d, rect_left.y);

            let xn = (rect_right.x - cu) / fu;
            let yn = (rect_right.y - cv) / fv;
            let ray_cam1 = rect.r_rect1.transpose() * Vector3::new(xn, yn, 1.0);
            let normalized_cam1 = Vector2::new(ray_cam1.x / ray_cam1.z, ray_cam1.y / ray_cam1.z);
            let distorted = rig.cam1.distort(normalized_cam1);
            let raw_right = Vector2::new(
                rig.cam1.intrinsics[0] * distorted.x + rig.cam1.intrinsics[2],
                rig.cam1.intrinsics[1] * distorted.y + rig.cam1.intrinsics[3],
            );

            let ssd = patch_ssd(
                left_img,
                right_img,
                kp.x as f64,
                kp.y as f64,
                raw_right.x,
                raw_right.y,
                params.patch_radius,
            )
            .unwrap_or(f64::INFINITY);
            ssd_by_step.push(ssd);
            if ssd < best_ssd {
                best_ssd = ssd;
                best_step = step;
            }
        }

        if !best_ssd.is_finite() || best_ssd > params.max_ssd_per_pixel {
            continue;
        }

        let mut refined_disp = params.min_disparity + best_step as f64 * params.disparity_step;
        let idx = best_step as usize;
        if idx > 0 && idx + 1 < ssd_by_step.len() {
            let (s_m1, s_0, s_p1) = (ssd_by_step[idx - 1], ssd_by_step[idx], ssd_by_step[idx + 1]);
            let denom = s_m1 - 2.0 * s_0 + s_p1;
            if denom.abs() > 1e-9 {
                let offset = (0.5 * (s_m1 - s_p1) / denom).clamp(-1.0, 1.0);
                refined_disp += offset * params.disparity_step;
            }
        }
        if refined_disp <= 0.0 {
            continue;
        }

        let depth = fu * rect.baseline / refined_disp;
        if !depth.is_finite() || depth <= 0.0 || depth > params.max_depth {
            continue;
        }

        let x_rect = (rect_left.x - cu) / fu * depth;
        let y_rect = (rect_left.y - cv) / fv * depth;
        let p_cam0 = rect.r_rect0.transpose() * Vector3::new(x_rect, y_rect, depth);

        matches.push(StereoMatch {
            left_pixel: (kp.x, kp.y),
            point_cam0: p_cam0,
        });
    }
    matches
}

fn patch_ssd(left: &GrayImage, right: &GrayImage, lx: f64, ly: f64, rx: f64, ry: f64, radius: i32) -> Option<f64> {
    let mut sum = 0.0;
    let mut count = 0.0;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let lv = sample_bilinear(left, lx + dx as f64, ly + dy as f64)?;
            let rv = sample_bilinear(right, rx + dx as f64, ry + dy as f64)?;
            let diff = lv - rv;
            sum += diff * diff;
            count += 1.0;
        }
    }
    Some(sum / count)
}

fn sample_bilinear(img: &GrayImage, x: f64, y: f64) -> Option<f64> {
    if x < 0.0 || y < 0.0 || x > (img.width() as f64 - 1.0) || y > (img.height() as f64 - 1.0) {
        return None;
    }
    let x0 = x.floor();
    let y0 = y.floor();
    let (fx, fy) = (x - x0, y - y0);
    let (x0, y0) = (x0 as u32, y0 as u32);
    let (x1, y1) = ((x0 + 1).min(img.width() - 1), (y0 + 1).min(img.height() - 1));

    let p00 = img.get_pixel(x0, y0).0[0] as f64;
    let p10 = img.get_pixel(x1, y0).0[0] as f64;
    let p01 = img.get_pixel(x0, y1).0[0] as f64;
    let p11 = img.get_pixel(x1, y1).0[0] as f64;

    Some(p00 * (1.0 - fx) * (1.0 - fy) + p10 * fx * (1.0 - fy) + p01 * (1.0 - fx) * fy + p11 * fx * fy)
}

#[cfg(test)]
mod tests {
    use super::*;
    use slam_core::SE3;
    use slam_geometry::PinholeCamera;

    fn mh01_rig() -> StereoRig {
        let t_bs_cam0 = SE3::from_matrix(&nalgebra::Matrix4::from_row_slice(&[
            0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0,
        ]));
        let t_bs_cam1 = SE3::from_matrix(&nalgebra::Matrix4::from_row_slice(&[
            0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0,
        ]));
        StereoRig {
            t_bs_cam0,
            t_bs_cam1,
            cam0: PinholeCamera::new(
                [458.654, 457.296, 367.215, 248.375],
                [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05],
            ),
            cam1: PinholeCamera::new(
                [457.587, 456.134, 379.999, 255.238],
                [-0.28368365, 0.07451284, -0.00010473, -3.556e-05],
            ),
        }
    }

    /// Stamps a small, spatially-unique "fingerprint" pattern (not
    /// periodic, so SSD has one unambiguous global minimum) at `(cx, cy)`.
    fn stamp_fingerprint(img: &mut GrayImage, cx: i32, cy: i32) {
        for dy in -6..=6 {
            for dx in -6..=6 {
                let x = cx + dx;
                let y = cy + dy;
                if x < 0 || y < 0 || x >= img.width() as i32 || y >= img.height() as i32 {
                    continue;
                }
                let v = ((dx * 7 + dy * 13 + dx * dy * 3).rem_euclid(180) + 40) as u8;
                img.put_pixel(x as u32, y as u32, image::Luma([v]));
            }
        }
    }

    #[test]
    fn recovers_synthetic_point_depth_from_real_calibration() {
        let rig = mh01_rig();
        let rect = rig.rectify();
        let t10 = rig.relative_pose_cam1_from_cam0();

        let true_point_cam0 = Vector3::new(0.1, -0.05, 3.0);
        let true_point_cam1 = t10.transform(&true_point_cam0);
        let pixel0 = rig.cam0.project(true_point_cam0);
        let pixel1 = rig.cam1.project(true_point_cam1);

        let mut left_img = GrayImage::new(752, 480);
        let mut right_img = GrayImage::new(752, 480);
        for p in left_img.pixels_mut() {
            *p = image::Luma([90]);
        }
        for p in right_img.pixels_mut() {
            *p = image::Luma([90]);
        }
        stamp_fingerprint(&mut left_img, pixel0.x.round() as i32, pixel0.y.round() as i32);
        stamp_fingerprint(&mut right_img, pixel1.x.round() as i32, pixel1.y.round() as i32);

        let keypoints = vec![Keypoint {
            x: pixel0.x as f32,
            y: pixel0.y as f32,
            score: 0.0,
        }];
        let matches = match_stereo_keypoints(&left_img, &right_img, &keypoints, &rig, &rect, &StereoMatchParams::default());

        assert_eq!(matches.len(), 1, "expected the fingerprinted point to match");
        // 1px disparity step + parabola sub-pixel refinement, at ~17px
        // disparity for a 3m-depth point: ~0.18m depth error per pixel of
        // disparity error (d(depth)/d(disparity) = -depth^2/(fu*baseline)),
        // so decimeter-level (not millimeter-level) recovery is expected
        // here — `triangulate_refine` in slam-geometry is what gets this
        // down to sub-mm once real correspondences feed it (see M1's test).
        let error = (matches[0].point_cam0 - true_point_cam0).norm();
        assert!(error < 0.2, "point recovered too far off: {:?} vs {:?} (error {error:.3}m)", matches[0].point_cam0, true_point_cam0);
    }

    #[test]
    fn rejects_a_keypoint_with_no_correspondence() {
        let rig = mh01_rig();
        let rect = rig.rectify();
        let mut left_img = GrayImage::new(752, 480);
        let mut right_img = GrayImage::new(752, 480);
        for p in left_img.pixels_mut() {
            *p = image::Luma([90]);
        }
        for p in right_img.pixels_mut() {
            *p = image::Luma([90]);
        }
        stamp_fingerprint(&mut left_img, 400, 240);
        // No matching fingerprint anywhere in the right image.

        let keypoints = vec![Keypoint { x: 400.0, y: 240.0, score: 0.0 }];
        let matches = match_stereo_keypoints(&left_img, &right_img, &keypoints, &rig, &rect, &StereoMatchParams::default());
        assert!(matches.is_empty());
    }
}
