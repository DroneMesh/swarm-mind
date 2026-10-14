//! Measured link quality for the renderer: how much of what each end sent
//! the other actually heard, over the last few seconds.
//!
//! Per ordered pair `(rx, tx)` the meter counts brain frames `tx` put on
//! air that `rx` should have heard (`rx` alive, the frame broadcast or
//! addressed to `rx`) and how many it did, in [`BUCKETS`] buckets of
//! [`BUCKET_MS`] (the current, partial one included: a 3-4 s window). The
//! quality of a link is the worse direction's delivery ratio.
//!
//! A transmitter that has died sends nothing, so each dead node is charged
//! one unheard frame per bucket towards every live node: a link to it decays
//! to 0 within the window while the neighbour tables still list it. A live
//! node that moved out of range or behind terrain keeps transmitting
//! unheard frames, so its links decay the same way. With no loss at all the
//! ratio is exactly 1. A direction with nothing expected in the window
//! reads 0 (no evidence the link works).
//!
//! The meter only counts; it draws nothing and the brains never see it.

/// Buckets in the window (the current one included).
pub const BUCKETS: usize = 4;
/// Bucket length.
pub const BUCKET_MS: u32 = 1_000;

#[derive(Clone, Debug)]
pub struct LinkMeter {
    n: usize,
    /// Per bucket, `n × n` `[expected, heard]` at `[rx * n + tx]`.
    buckets: Vec<Vec<[u16; 2]>>,
    /// Absolute index of the current bucket.
    cur: u32,
}

impl LinkMeter {
    pub fn new(n: usize) -> Self {
        Self { n, buckets: vec![vec![[0, 0]; n * n]; BUCKETS], cur: 0 }
    }

    fn slot(&self) -> usize {
        self.cur as usize % BUCKETS
    }

    /// Moves to the bucket of `now_ms`; every bucket it enters charges each
    /// dead node one unheard frame towards each live one.
    pub fn roll(&mut self, now_ms: u32, alive: &dyn Fn(usize) -> bool) {
        let b = now_ms / BUCKET_MS;
        if b <= self.cur {
            return;
        }
        let gap = (b - self.cur).min(BUCKETS as u32);
        let n = self.n;
        for k in 1..=gap {
            let s = (self.cur + k) as usize % BUCKETS;
            self.buckets[s].fill([0, 0]);
            for d in (0..n).filter(|&d| !alive(d)) {
                for rx in (0..n).filter(|&rx| rx != d && alive(rx)) {
                    self.buckets[s][rx * n + d][0] = 1;
                }
            }
        }
        self.cur = b;
    }

    /// `tx` transmitted a frame `rx` should have heard; `heard` says if it did.
    pub fn record(&mut self, rx: usize, tx: usize, heard: bool) {
        if rx >= self.n || tx >= self.n {
            return;
        }
        let s = self.slot();
        let c = &mut self.buckets[s][rx * self.n + tx];
        c[0] = c[0].saturating_add(1);
        if heard {
            c[1] = c[1].saturating_add(1);
        }
    }

    /// Delivery ratio `tx → rx` over the window (0 when nothing was expected).
    pub fn direction(&self, rx: usize, tx: usize) -> f32 {
        if rx >= self.n || tx >= self.n {
            return 0.0;
        }
        let i = rx * self.n + tx;
        let (mut e, mut h) = (0u32, 0u32);
        for b in &self.buckets {
            e += u32::from(b[i][0]);
            h += u32::from(b[i][1]);
        }
        if e == 0 { 0.0 } else { h.min(e) as f32 / e as f32 }
    }

    /// Quality of the link `a ↔ b`: the worse direction, in `0..=1`.
    pub fn quality(&self, a: usize, b: usize) -> f32 {
        self.direction(a, b).min(self.direction(b, a))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ratio_window_and_dead_decay() {
        let mut m = LinkMeter::new(3);
        let all = |_: usize| true;
        for t in (0..4_000).step_by(250) {
            m.roll(t, &all);
            m.record(0, 1, true);
            m.record(1, 0, true);
            m.record(2, 1, t % 500 == 0);
            m.record(1, 2, true);
        }
        assert_eq!(m.quality(0, 1), 1.0, "no loss is exactly 1");
        assert!((m.quality(1, 2) - 0.5).abs() < 1e-6);
        assert_eq!(m.quality(0, 2), 0.0, "nothing expected");
        // Node 1 dies: its links fall to 0 within the window.
        let dead = |i: usize| i != 1;
        let mut last = 1.0;
        for t in (4_000..9_000).step_by(250) {
            m.roll(t, &dead);
            m.record(1, 0, false); // dead receivers are not counted by the world; harmless here
            let q = m.quality(0, 1);
            assert!(q <= last + 1e-6, "{t}: {q} > {last}");
            last = q;
        }
        assert_eq!(last, 0.0);
        assert_eq!(m.quality(0, 9), 0.0);
    }
}
