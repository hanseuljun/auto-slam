use slam_vision::Descriptor;

/// A bag-of-visual-words vocabulary: `k` cluster centroids in Hamming
/// space (binary-descriptor k-means — nearest-centroid assignment by
/// Hamming distance, centroid update by per-bit majority vote, the
/// standard adaptation of k-means to binary descriptors used by
/// DBoW2-style loop closure systems).
pub struct Vocabulary {
    words: Vec<Descriptor>,
}

impl Vocabulary {
    pub fn len(&self) -> usize {
        self.words.len()
    }

    pub fn is_empty(&self) -> bool {
        self.words.is_empty()
    }

    /// Trains a flat vocabulary of `k` words from a pool of descriptors
    /// (typically gathered from many frames across the training
    /// sequences). Deterministic given the same `seed`.
    pub fn train(descriptors: &[Descriptor], k: usize, iterations: usize, seed: u64) -> Self {
        assert!(!descriptors.is_empty() && k > 0);
        let k = k.min(descriptors.len());

        let mut state = seed;
        let mut next_index = |n: usize| -> usize {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as usize) % n
        };

        // Seed centroids from distinct random descriptors.
        let mut chosen = std::collections::HashSet::new();
        let mut words = Vec::with_capacity(k);
        while words.len() < k {
            let idx = next_index(descriptors.len());
            if chosen.insert(idx) {
                words.push(descriptors[idx]);
            }
        }

        for _ in 0..iterations {
            let mut sums = vec![[0u32; 256]; k];
            let mut counts = vec![0u32; k];

            for d in descriptors {
                let nearest = nearest_word(&words, d);
                counts[nearest] += 1;
                // `bit` indexes both `d`'s bits and `sums[nearest]`'s
                // per-bit counters, so this isn't a plain single-slice
                // iteration clippy's suggested rewrite would fit.
                #[allow(clippy::needless_range_loop)]
                for bit in 0..256 {
                    if (d.0[bit / 64] >> (bit % 64)) & 1 == 1 {
                        sums[nearest][bit] += 1;
                    }
                }
            }

            for (word_idx, word) in words.iter_mut().enumerate() {
                if counts[word_idx] == 0 {
                    continue; // keep the previous centroid for an empty cluster
                }
                let half = counts[word_idx];
                let mut bits = [0u64; 4];
                for bit in 0..256 {
                    if sums[word_idx][bit] * 2 >= half {
                        bits[bit / 64] |= 1 << (bit % 64);
                    }
                }
                *word = Descriptor(bits);
            }
        }

        Vocabulary { words }
    }

    pub fn word_for(&self, d: &Descriptor) -> usize {
        nearest_word(&self.words, d)
    }

    /// A normalized (L1) histogram over vocabulary words — the standard
    /// bag-of-words representation of a keyframe's descriptor set.
    pub fn compute_bow(&self, descriptors: &[Descriptor]) -> BowVector {
        let mut weights = vec![0f32; self.words.len()];
        for d in descriptors {
            weights[self.word_for(d)] += 1.0;
        }
        let total: f32 = weights.iter().sum();
        if total > 0.0 {
            for w in &mut weights {
                *w /= total;
            }
        }
        BowVector { weights }
    }
}

fn nearest_word(words: &[Descriptor], d: &Descriptor) -> usize {
    words
        .iter()
        .enumerate()
        .min_by_key(|(_, w)| w.hamming_distance(d))
        .map(|(i, _)| i)
        .expect("vocabulary must be non-empty")
}

#[derive(Debug, Clone)]
pub struct BowVector {
    weights: Vec<f32>,
}

impl BowVector {
    /// The standard DBoW-style L1 similarity score, in `[0, 1]`: `1 -
    /// 0.5 * sum(|a_i - b_i|)`, `1.0` for identical histograms.
    pub fn similarity(&self, other: &BowVector) -> f32 {
        let l1: f32 = self.weights.iter().zip(other.weights.iter()).map(|(a, b)| (a - b).abs()).sum();
        1.0 - 0.5 * l1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn random_descriptor(state: &mut u64) -> Descriptor {
        let mut bits = [0u64; 4];
        for b in &mut bits {
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *b = *state;
        }
        Descriptor(bits)
    }

    #[test]
    fn identical_descriptor_sets_have_similarity_one() {
        let mut state = 42u64;
        let descriptors: Vec<Descriptor> = (0..200).map(|_| random_descriptor(&mut state)).collect();
        let vocab = Vocabulary::train(&descriptors, 20, 5, 7);

        let bow_a = vocab.compute_bow(&descriptors);
        let bow_b = vocab.compute_bow(&descriptors);
        assert!((bow_a.similarity(&bow_b) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn disjoint_word_usage_has_low_similarity() {
        // Two descriptor sets clustered tightly around two very different
        // "prototype" patterns should land in mostly-disjoint words.
        let mut state = 1u64;
        let prototype_a = random_descriptor(&mut state);
        let prototype_b = Descriptor(prototype_a.0.map(|w| !w)); // bitwise complement: maximally different

        let jitter = |proto: &Descriptor, state: &mut u64| -> Descriptor {
            let mut bits = proto.0;
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let flip_bit = (*state % 256) as usize;
            bits[flip_bit / 64] ^= 1 << (flip_bit % 64);
            Descriptor(bits)
        };

        let set_a: Vec<Descriptor> = (0..100).map(|_| jitter(&prototype_a, &mut state)).collect();
        let set_b: Vec<Descriptor> = (0..100).map(|_| jitter(&prototype_b, &mut state)).collect();

        let mut training = set_a.clone();
        training.extend(set_b.clone());
        let vocab = Vocabulary::train(&training, 10, 8, 3);

        let bow_a = vocab.compute_bow(&set_a);
        let bow_b = vocab.compute_bow(&set_b);
        assert!(bow_a.similarity(&bow_b) < 0.3, "expected low similarity, got {}", bow_a.similarity(&bow_b));

        let bow_a2 = vocab.compute_bow(&set_a);
        assert!(bow_a.similarity(&bow_a2) > 0.8, "expected high self-similarity, got {}", bow_a.similarity(&bow_a2));
    }
}
