/// The Huber robust kernel's residual-scaling weight: `1.0` inside the
/// `delta` threshold (behaves like ordinary least squares), tapering as
/// `sqrt(delta / |r|)` beyond it (behaves like L1, downweighting
/// outliers). Applying `sqrt(weight)` to both the residual and its
/// Jacobian before accumulating into the normal equations reproduces the
/// Huber loss's gradient — the standard "iteratively reweighted least
/// squares" implementation of a robust kernel.
pub fn huber_weight(residual_norm: f64, delta: f64) -> f64 {
    if residual_norm <= delta {
        1.0
    } else {
        (delta / residual_norm).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_downweighting_inside_threshold() {
        assert_eq!(huber_weight(0.5, 1.0), 1.0);
        assert_eq!(huber_weight(1.0, 1.0), 1.0);
    }

    #[test]
    fn downweights_beyond_threshold() {
        let w = huber_weight(4.0, 1.0);
        assert!(w < 1.0);
        // Weighted residual norm should be sqrt(delta * |r|), not |r|.
        assert!((w * 4.0 - (1.0f64 * 4.0).sqrt()).abs() < 1e-12);
    }
}
