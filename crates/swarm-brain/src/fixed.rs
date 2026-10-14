use core::ops::{Deref, DerefMut};

/// A vector with a fixed capacity, stored inline. It never allocates.
#[derive(Clone, Copy, Debug)]
pub struct FixedVec<T: Copy + Default, const N: usize> {
    buf: [T; N],
    len: usize,
}

impl<T: Copy + Default, const N: usize> FixedVec<T, N> {
    pub fn new() -> Self {
        Self { buf: [T::default(); N], len: 0 }
    }

    pub fn is_full(&self) -> bool {
        self.len == N
    }

    /// Appends `value`, or hands it back if the vector is full.
    pub fn push(&mut self, value: T) -> Result<(), T> {
        if self.is_full() {
            return Err(value);
        }
        self.buf[self.len] = value;
        self.len += 1;
        Ok(())
    }

    /// Removes element `i` in O(1) by moving the last element into its slot.
    pub fn swap_remove(&mut self, i: usize) -> T {
        assert!(i < self.len, "index {i} out of bounds (len {})", self.len);
        let value = self.buf[i];
        self.len -= 1;
        self.buf[i] = self.buf[self.len];
        value
    }

    pub fn retain(&mut self, mut keep: impl FnMut(&T) -> bool) {
        let mut i = 0;
        while i < self.len {
            if keep(&self.buf[i]) {
                i += 1;
            } else {
                self.swap_remove(i);
            }
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<T: Copy + Default, const N: usize> Default for FixedVec<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Default, const N: usize> Deref for FixedVec<T, N> {
    type Target = [T];
    fn deref(&self) -> &[T] {
        &self.buf[..self.len]
    }
}

impl<T: Copy + Default, const N: usize> DerefMut for FixedVec<T, N> {
    fn deref_mut(&mut self) -> &mut [T] {
        &mut self.buf[..self.len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_rejects_when_full() {
        let mut v = FixedVec::<u8, 2>::new();
        assert_eq!(v.push(1), Ok(()));
        assert_eq!(v.push(2), Ok(()));
        assert_eq!(v.push(3), Err(3));
        assert_eq!(&*v, &[1, 2]);
    }

    #[test]
    fn retain_keeps_matching() {
        let mut v = FixedVec::<u8, 8>::new();
        for i in 0..8 {
            v.push(i).unwrap();
        }
        v.retain(|x| x % 2 == 0);
        let mut kept = [0u8; 4];
        kept.copy_from_slice(&v);
        kept.sort_unstable();
        assert_eq!(kept, [0, 2, 4, 6]);
    }
}
