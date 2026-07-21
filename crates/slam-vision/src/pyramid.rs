use image::GrayImage;

/// A coarse-to-fine image pyramid. `levels[0]` is full resolution;
/// `levels[i]` is half the width/height of `levels[i-1]`, built with a 2x2
/// box-filter downsample (simple and our own, rather than pulling in a
/// general-purpose resize algorithm for something this small).
pub struct ImagePyramid {
    pub levels: Vec<GrayImage>,
}

impl ImagePyramid {
    pub fn build(image: &GrayImage, num_levels: usize) -> Self {
        let mut levels = Vec::with_capacity(num_levels.max(1));
        levels.push(image.clone());
        while levels.len() < num_levels {
            let prev = levels.last().unwrap();
            if prev.width() < 4 || prev.height() < 4 {
                break;
            }
            levels.push(downsample_half(prev));
        }
        ImagePyramid { levels }
    }
}

fn downsample_half(img: &GrayImage) -> GrayImage {
    let (w, h) = img.dimensions();
    let (out_w, out_h) = (w / 2, h / 2);
    let mut out = GrayImage::new(out_w, out_h);
    for y in 0..out_h {
        for x in 0..out_w {
            let (sx, sy) = (2 * x, 2 * y);
            let sum = img.get_pixel(sx, sy).0[0] as u32
                + img.get_pixel(sx + 1, sy).0[0] as u32
                + img.get_pixel(sx, sy + 1).0[0] as u32
                + img.get_pixel(sx + 1, sy + 1).0[0] as u32;
            out.put_pixel(x, y, image::Luma([(sum / 4) as u8]));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_level_is_half_the_previous() {
        let img = GrayImage::new(64, 48);
        let pyramid = ImagePyramid::build(&img, 4);
        assert_eq!(pyramid.levels.len(), 4);
        assert_eq!(pyramid.levels[0].dimensions(), (64, 48));
        assert_eq!(pyramid.levels[1].dimensions(), (32, 24));
        assert_eq!(pyramid.levels[2].dimensions(), (16, 12));
        assert_eq!(pyramid.levels[3].dimensions(), (8, 6));
    }

    #[test]
    fn stops_before_degenerate_sizes() {
        let img = GrayImage::new(6, 6);
        let pyramid = ImagePyramid::build(&img, 10);
        // 6 -> 3 is below the width>=4 cutoff, so building stops there.
        assert_eq!(pyramid.levels.len(), 2);
    }

    #[test]
    fn downsample_averages_a_flat_region() {
        let mut img = GrayImage::new(4, 4);
        for p in img.pixels_mut() {
            *p = image::Luma([100]);
        }
        let pyramid = ImagePyramid::build(&img, 2);
        assert_eq!(pyramid.levels[1].get_pixel(0, 0).0[0], 100);
    }
}
