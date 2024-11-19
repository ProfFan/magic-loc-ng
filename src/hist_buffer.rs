#[allow(unused)]
use core::fmt;
use core::mem::MaybeUninit;
use core::ops::Deref;
use core::ptr;
use core::slice;

use bytemuck::offset_of;
use bytemuck::AnyBitPattern;

/// A "history buffer", similar to a write-only ring buffer of fixed length.
///
/// This buffer keeps a fixed number of elements.  On write, the oldest element
/// is overwritten. Thus, the buffer is useful to keep a history of values with
/// some desired depth, and for example calculate a rolling average.
///
/// # Examples
/// ```
/// use heapless::HistoryBuffer;
///
/// // Initialize a new buffer with 8 elements.
/// let mut buf = HistoryBuffer::<_, 8>::new();
///
/// // Starts with no data
/// assert_eq!(buf.recent(), None);
///
/// buf.write(3);
/// buf.write(5);
/// buf.extend(&[4, 4]);
///
/// // The most recent written element is a four.
/// assert_eq!(buf.recent(), Some(&4));
///
/// // To access all elements in an unspecified order, use `as_slice()`.
/// for el in buf.as_slice() { println!("{:?}", el); }
///
/// // Now we can prepare an average of all values, which comes out to 4.
/// let avg = buf.as_slice().iter().sum::<usize>() / buf.len();
/// assert_eq!(avg, 4);
/// ```
#[repr(C)]
pub struct HistoryBuffer<T: AnyBitPattern, const N: usize> {
    data: [MaybeUninit<T>; N],
    write_at: usize,
    filled: bool,
}

impl<T: AnyBitPattern, const N: usize> HistoryBuffer<T, N> {
    const INIT: MaybeUninit<T> = MaybeUninit::uninit();

    /// Constructs a new history buffer.
    ///
    /// The construction of a `HistoryBuffer` works in `const` contexts.
    ///
    /// # Examples
    ///
    /// ```
    /// use heapless::HistoryBuffer;
    ///
    /// // Allocate a 16-element buffer on the stack
    /// let x: HistoryBuffer<u8, 16> = HistoryBuffer::new();
    /// assert_eq!(x.len(), 0);
    /// ```
    #[inline]
    pub const fn new() -> Self {
        // Const assert
        assert!(N > 0, "N must be greater than 0");

        Self {
            data: [Self::INIT; N],
            write_at: 0,
            filled: false,
        }
    }

    /// Clears the buffer, replacing every element with the default value of
    /// type `T`.
    pub fn clear(&mut self) {
        *self = Self::new();
    }
}

impl<T: AnyBitPattern, const N: usize> HistoryBuffer<T, N> {
    pub fn is_valid(data: &[u8]) -> bool {
        // Check if the data is of the correct length
        if data.len() != size_of::<Self>() {
            return false;
        }

        // Check if the write_at is within bounds
        let write_at_usize = unsafe {
            data.as_ptr()
                .add(offset_of!(Self, write_at))
                .cast::<usize>()
                .read_unaligned()
        };
        if write_at_usize >= N {
            return false;
        }

        true
    }
}

impl<T: AnyBitPattern, const N: usize> HistoryBuffer<T, N>
where
    T: Copy + Clone,
{
    /// Constructs a new history buffer, where every element is the given value.
    ///
    /// # Examples
    ///
    /// ```
    /// use heapless::HistoryBuffer;
    ///
    /// // Allocate a 16-element buffer on the stack
    /// let mut x: HistoryBuffer<u8, 16> = HistoryBuffer::new_with(4);
    /// // All elements are four
    /// assert_eq!(x.as_slice(), [4; 16]);
    /// ```
    #[inline]
    pub fn new_with(t: T) -> Self {
        Self {
            data: [MaybeUninit::new(t); N],
            write_at: 0,
            filled: true,
        }
    }

    /// Clears the buffer, replacing every element with the given value.
    pub fn clear_with(&mut self, t: T) {
        *self = Self::new_with(t);
    }
}

impl<T: AnyBitPattern, const N: usize> HistoryBuffer<T, N> {
    /// Returns the current fill level of the buffer.
    #[inline]
    pub fn len(&self) -> usize {
        if self.filled {
            N
        } else {
            self.write_at
        }
    }

    /// Returns the capacity of the buffer, which is the length of the
    /// underlying backing array.
    #[inline]
    pub fn capacity(&self) -> usize {
        N
    }

    /// Writes an element to the buffer, overwriting the oldest value.
    pub fn write(&mut self, t: T) {
        if self.filled {
            // Drop the old before we overwrite it.
            unsafe { ptr::drop_in_place(self.data[self.write_at].as_mut_ptr()) }
        }
        self.data[self.write_at] = MaybeUninit::new(t);

        self.write_at += 1;
        if self.write_at == self.capacity() {
            self.write_at = 0;
            self.filled = true;
        }
    }

    /// Clones and writes all elements in a slice to the buffer.
    ///
    /// If the slice is longer than the buffer, only the last `self.len()`
    /// elements will actually be stored.
    pub fn extend_from_slice(&mut self, other: &[T])
    where
        T: Clone,
    {
        for item in other {
            self.write(*item);
        }
    }

    /// Returns a reference to the most recently written value.
    ///
    /// # Examples
    ///
    /// ```
    /// use heapless::HistoryBuffer;
    ///
    /// let mut x: HistoryBuffer<u8, 16> = HistoryBuffer::new();
    /// x.write(4);
    /// x.write(10);
    /// assert_eq!(x.recent(), Some(&10));
    /// ```
    pub fn recent(&self) -> Option<&T> {
        if self.write_at == 0 {
            if self.filled {
                Some(unsafe { &*self.data[self.capacity() - 1].as_ptr() })
            } else {
                None
            }
        } else {
            Some(unsafe { &*self.data[self.write_at - 1].as_ptr() })
        }
    }

    /// Returns the array slice backing the buffer, without keeping track
    /// of the write position. Therefore, the element order is unspecified.
    pub fn as_slice(&self) -> &[T] {
        unsafe { slice::from_raw_parts(self.data.as_ptr() as *const _, self.len()) }
    }

    /// Returns a pair of slices which contain, in order, the contents of the buffer.
    ///
    /// # Examples
    ///
    /// ```
    /// use heapless::HistoryBuffer;
    ///
    /// let mut buffer: HistoryBuffer<u8, 6> = HistoryBuffer::new();
    /// buffer.extend([0, 0, 0]);
    /// buffer.extend([1, 2, 3, 4, 5, 6]);
    /// assert_eq!(buffer.as_slices(), (&[1, 2, 3][..], &[4, 5, 6][..]));
    /// ```
    pub fn as_slices(&self) -> (&[T], &[T]) {
        let buffer = self.as_slice();

        if !self.filled {
            (buffer, &[])
        } else {
            (&buffer[self.write_at..], &buffer[..self.write_at])
        }
    }

    /// Returns an iterator for iterating over the buffer from oldest to newest.
    ///
    /// # Examples
    ///
    /// ```
    /// use heapless::HistoryBuffer;
    ///
    /// let mut buffer: HistoryBuffer<u8, 6> = HistoryBuffer::new();
    /// buffer.extend([0, 0, 0, 1, 2, 3, 4, 5, 6]);
    /// let expected = [1, 2, 3, 4, 5, 6];
    /// for (x, y) in buffer.oldest_ordered().zip(expected.iter()) {
    ///     assert_eq!(x, y)
    /// }
    ///
    /// ```
    pub fn oldest_ordered(&self) -> OldestOrdered<'_, T, N> {
        if self.filled {
            OldestOrdered {
                buf: self,
                cur: self.write_at,
                wrapped: false,
            }
        } else {
            // special case: act like we wrapped already to handle empty buffer.
            OldestOrdered {
                buf: self,
                cur: 0,
                wrapped: true,
            }
        }
    }
}

impl<T: AnyBitPattern, const N: usize> Extend<T> for HistoryBuffer<T, N> {
    fn extend<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = T>,
    {
        for item in iter.into_iter() {
            self.write(item);
        }
    }
}

impl<'a, T: AnyBitPattern, const N: usize> Extend<&'a T> for HistoryBuffer<T, N>
where
    T: 'a + Clone,
{
    fn extend<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = &'a T>,
    {
        self.extend(iter.into_iter().cloned())
    }
}

impl<T: AnyBitPattern, const N: usize> Clone for HistoryBuffer<T, N>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        let mut ret = Self::new();
        for (new, old) in ret.data.iter_mut().zip(self.as_slice()) {
            new.write(*old);
        }
        ret.filled = self.filled;
        ret.write_at = self.write_at;
        ret
    }
}

impl<T: AnyBitPattern, const N: usize> Drop for HistoryBuffer<T, N> {
    fn drop(&mut self) {
        unsafe {
            ptr::drop_in_place(ptr::slice_from_raw_parts_mut(
                self.data.as_mut_ptr() as *mut T,
                self.len(),
            ))
        }
    }
}

impl<T: AnyBitPattern, const N: usize> Deref for HistoryBuffer<T, N> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        self.as_slice()
    }
}

impl<T: AnyBitPattern, const N: usize> AsRef<[T]> for HistoryBuffer<T, N> {
    #[inline]
    fn as_ref(&self) -> &[T] {
        self
    }
}

impl<T: AnyBitPattern, const N: usize> fmt::Debug for HistoryBuffer<T, N>
where
    T: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        <[T] as fmt::Debug>::fmt(self, f)
    }
}

impl<T: AnyBitPattern, const N: usize> Default for HistoryBuffer<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: AnyBitPattern, const N: usize> PartialEq for HistoryBuffer<T, N>
where
    T: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        self.oldest_ordered().eq(other.oldest_ordered())
    }
}

/// An iterator on the underlying buffer ordered from oldest data to newest
#[derive(Clone)]
pub struct OldestOrdered<'a, T: AnyBitPattern, const N: usize> {
    buf: &'a HistoryBuffer<T, N>,
    cur: usize,
    wrapped: bool,
}

impl<'a, T: AnyBitPattern, const N: usize> Iterator for OldestOrdered<'a, T, N> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        if self.cur == self.buf.len() && self.buf.filled {
            // roll-over
            self.cur = 0;
            self.wrapped = true;
        }

        if self.cur == self.buf.write_at && self.wrapped {
            return None;
        }

        let item = &self.buf[self.cur];
        self.cur += 1;
        Some(item)
    }
}
