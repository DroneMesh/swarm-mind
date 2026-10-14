//! Bucketed sliding windows shared by the telemetry stats and the air ledger.

/// Bucketed sliding window (250 ms buckets).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Window<const N: usize> {
    buckets: [f32; N],
    /// Index of the current bucket (absolute, `now / 250`).
    cur: u32,
}

pub(crate) const BUCKET_MS: u32 = 250;

impl<const N: usize> Window<N> {
    pub(crate) const SPAN_S: f32 = (N as u32 * BUCKET_MS) as f32 / 1000.0;

    pub(crate) fn new() -> Self {
        Self { buckets: [0.0; N], cur: 0 }
    }

    pub(crate) fn roll(&mut self, now_ms: u32) {
        let b = now_ms / BUCKET_MS;
        if b <= self.cur {
            return;
        }
        let gap = (b - self.cur).min(N as u32);
        for k in 1..=gap {
            self.buckets[((self.cur + k) % N as u32) as usize] = 0.0;
        }
        self.cur = b;
    }

    pub(crate) fn add(&mut self, now_ms: u32, v: f32) {
        self.roll(now_ms);
        self.buckets[(self.cur % N as u32) as usize] += v;
    }

    pub(crate) fn sum(&mut self, now_ms: u32) -> f32 {
        self.roll(now_ms);
        self.buckets.iter().sum()
    }

    /// Adds `v` to the bucket of the earlier time `at_ms` (ignored when that
    /// bucket has already rolled out of the window).
    pub(crate) fn add_at(&mut self, now_ms: u32, at_ms: u32, v: f32) {
        self.roll(now_ms);
        let b = at_ms / BUCKET_MS;
        if b <= self.cur && self.cur - b < N as u32 {
            self.buckets[(b % N as u32) as usize] += v;
        }
    }

    /// Rate over the buckets at least `lag` buckets old (the newest `lag`,
    /// the current one included, are left out).
    pub(crate) fn per_s_lagged(&mut self, now_ms: u32, lag: u32) -> f32 {
        self.roll(now_ms);
        let mut sum = 0.0;
        for k in lag..N as u32 {
            if let Some(b) = self.cur.checked_sub(k) {
                sum += self.buckets[(b % N as u32) as usize];
            }
        }
        sum / ((N as u32 - lag) * BUCKET_MS) as f32 * 1000.0
    }

    /// Rate over the completed buckets (the current, partial one is left
    /// out so the rate doesn't sag between bucket boundaries).
    pub(crate) fn per_s(&mut self, now_ms: u32) -> f32 {
        self.roll(now_ms);
        let current = self.buckets[(self.cur % N as u32) as usize];
        (self.buckets.iter().sum::<f32>() - current) / (Self::SPAN_S - BUCKET_MS as f32 / 1000.0)
    }
}

impl<const N: usize> Window<N> {
    /// [`Window::per_s`] without rolling: the same value, from `&self`.
    pub(crate) fn per_s_at(&self, now_ms: u32) -> f32 {
        let b = now_ms / BUCKET_MS;
        let mut sum = 0.0;
        for k in 1..N as u32 {
            if let Some(idx) = b.checked_sub(k)
                && idx <= self.cur
                && self.cur - idx < N as u32
            {
                sum += self.buckets[(idx % N as u32) as usize];
            }
        }
        sum / (Self::SPAN_S - BUCKET_MS as f32 / 1000.0)
    }
}

/// 2 s of completed buckets plus the current one.
pub(crate) type Short = Window<9>;
/// Frames by their send time `"t"`: 2 s of buckets, read [`SENT_LAG`]
/// buckets late so frames still in flight are not missed.
pub(crate) type BySent = Window<11>;
/// Upward TCP runs Nagle: frames reach the ground station in bursts on the
/// parents' 250 ms delayed-ACK ticks, 0.1-0.5 s after `"t"` over a few
/// hops. Counted by arrival, a 2 s window would see one burst more or less
/// (±3 Hz at 25 Hz); counted by `"t"` and read 0.75 s late, a steady
/// stream reads its true rate.
pub(crate) const SENT_LAG: u32 = 3;
pub(crate) type Long = Window<40>;
