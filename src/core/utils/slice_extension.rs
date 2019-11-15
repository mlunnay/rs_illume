/// Extension methods on slices to provide functionality from c++ not in rust.
pub trait SliceExtension<T> {
    /// Rearanges the slice based on the given binary predicate such that all elements
    /// for which the predicate returns true precede the elements for
    /// which it returns false.
    fn partition<P>(&mut self, predicate: P) -> usize
    where
    P: Fn(&T) -> bool;

    /// Partially sorts a slice in place such that the element at n appears where
    /// it would if the slice was sorted, and that all elements before n are
    /// less than or equal to all elements after n. Elements are compared using the
    /// binary predicate supplied.
    fn partial_sort<P>(&mut self, n: usize, predicate: P)
    where
    P: Fn(&T, &T) -> bool;
}

impl<T> SliceExtension<T> for [T] {
    fn partition<P>(&mut self, predicate: P) -> usize
    where
    P: Fn(&T) -> bool
    {
        let len = self.len();
        let mut first = len;
        for i in 0..len {
            if !predicate(&self[i]) {
            first = i;
            break;
            }
        }
        if first == len {
            return len;
        }

        for i in first..len {
            if predicate(&self[i]) {
            self.swap(i, first);
            first += 1;
            }
        }
        first
    }

    fn partial_sort<P>(&mut self, n: usize, predicate: P)
    where
    P: Fn(&T, &T) -> bool {
        fn partial_sort<T, P>(data: &mut [T], n: usize, predicate: &P, accu: &mut usize)
        where
        P: Fn(&T, &T) -> bool {
            if !data.is_empty() && *accu < n {
                let mut pivot = 1;
                let mut lower = 0;
                let mut upper = data.len();
                while pivot < upper {
                    if predicate(&data[pivot], &data[lower]) {
                        data.swap(pivot, lower);
                        lower += 1;
                        pivot += 1;
                    } else {
                        upper -= 1;
                        data.swap(pivot, upper);
                    }
                }
                partial_sort(&mut data[..lower], n, predicate, accu);
                partial_sort(&mut data[upper..], n, predicate, accu);
            } else {
                *accu += 1;
            }
        }
        partial_sort(self, n, &predicate, &mut 0);
    }
}