use crate::vocabulary::BowVector;

struct Entry {
    keyframe_id: usize,
    bow: BowVector,
}

/// A database of keyframe BoW vectors, queryable for place-recognition
/// candidates — keyframes that look similar to a query but are far enough
/// away in the trajectory to plausibly be a genuine loop (not just the
/// previous keyframe looking like itself).
#[derive(Default)]
pub struct KeyframeDatabase {
    entries: Vec<Entry>,
}

impl KeyframeDatabase {
    pub fn new() -> Self {
        KeyframeDatabase { entries: Vec::new() }
    }

    pub fn insert(&mut self, keyframe_id: usize, bow: BowVector) {
        self.entries.push(Entry { keyframe_id, bow });
    }

    /// The best-matching keyframe at least `min_id_gap` away from
    /// `query_id`, if its similarity clears `min_similarity`.
    pub fn query(&self, query_id: usize, query_bow: &BowVector, min_id_gap: usize, min_similarity: f32) -> Option<(usize, f32)> {
        self.entries
            .iter()
            .filter(|e| query_id.abs_diff(e.keyframe_id) >= min_id_gap)
            .map(|e| (e.keyframe_id, query_bow.similarity(&e.bow)))
            .filter(|&(_, sim)| sim >= min_similarity)
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vocabulary::Vocabulary;
    use slam_vision::Descriptor;

    fn random_descriptor(state: &mut u64) -> Descriptor {
        let mut bits = [0u64; 4];
        for b in &mut bits {
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *b = *state;
        }
        Descriptor(bits)
    }

    #[test]
    fn finds_a_similar_distant_keyframe_but_ignores_nearby_ones() {
        let mut state = 11u64;
        let prototype: Vec<Descriptor> = (0..40).map(|_| random_descriptor(&mut state)).collect();
        let unrelated: Vec<Descriptor> = (0..40).map(|_| random_descriptor(&mut state)).collect();

        let mut training = prototype.clone();
        training.extend(unrelated.iter().cloned());
        let vocab = Vocabulary::train(&training, 15, 6, 5);

        let mut db = KeyframeDatabase::new();
        // Keyframe 0: the "place" we'll later revisit.
        db.insert(0, vocab.compute_bow(&prototype));
        // Keyframes 1..5: unrelated places in between.
        for id in 1..5 {
            db.insert(id, vocab.compute_bow(&unrelated));
        }

        // Keyframe 50 revisits keyframe 0's place.
        let query_bow = vocab.compute_bow(&prototype);
        let result = db.query(50, &query_bow, 10, 0.5);
        assert_eq!(result.map(|(id, _)| id), Some(0));

        // With too tight a temporal gap, keyframe 4 (unrelated, but
        // within the gap) shouldn't disqualify a real match, and a query
        // right next to keyframe 0 should be excluded by the gap.
        let result_too_close = db.query(1, &query_bow, 10, 0.5);
        assert!(result_too_close.is_none(), "keyframe 0 is within min_id_gap of query 1, should be excluded");
    }
}
