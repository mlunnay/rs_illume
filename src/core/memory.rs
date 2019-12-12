use std::mem::size_of;
use std::ops::{Index, IndexMut};

const LOG_BLOCK_SIZE: usize = 2;
const BLOCK_SIZE: usize = 1 << LOG_BLOCK_SIZE;

/// A generic 2D array of values, with the items ordered in memory using a blocked memory layout.
pub struct BlockedArray<T> {
    data: Vec<T>,
    u_res: usize,
    v_res: usize,
    u_blocks: usize
}

impl<T> BlockedArray<T>
where
T: Copy + Default
{
    pub fn new(u_res: usize, v_res: usize) -> BlockedArray<T> {
        let n_alloc = round_up(u_res) * round_up(v_res);
        let data: Vec<T> = vec![Default::default(); n_alloc];
        BlockedArray::<T> {
            data,
            u_res,
            v_res,
            u_blocks: round_up(u_res) >> LOG_BLOCK_SIZE
        }
    }

    pub fn new_from(u_res: usize, v_res: usize, d: &[T]) -> BlockedArray<T> {
        let mut ba = BlockedArray::<T>::new(u_res, v_res);
        for v in 0..v_res {
            for u in 0..u_res {
                ba[(u, v)] = d[v * u_res + u];
            }
        }
        ba
    }

    pub fn block(&self, a: usize) -> usize {
        a >> LOG_BLOCK_SIZE
    }

    pub fn offset(&self, a: usize) -> usize {
        a & (BLOCK_SIZE - 1)
    }

    pub fn get_linear_array(&self) -> Vec<&T> {
        let res = Vec::with_capacity(self.data.len());
        for v in 0..self.v_res {
            for u in 0..self.u_res {
                res.push(&self[(u,v)]);
            }
        }
        res
    }

    pub fn u_size(&self) -> usize {
        self.u_res
    }

    pub fn v_size(&self) -> usize {
        self.v_res
    }

    pub fn block_size(&self) -> usize {
        BLOCK_SIZE
    }
}

impl<T> Index<(usize, usize)> for BlockedArray<T>
where
T: Copy + Default
{
    type Output = T;

    fn index(&self, i: (usize, usize)) -> &T {
        let bu = self.block(i.0);
        let bv = self.block(i.1);
        let ou = self.offset(i.0);
        let ov = self.offset(i.1);
        let mut offset = BLOCK_SIZE * BLOCK_SIZE * (self.u_blocks * bv + bu);
        offset += BLOCK_SIZE * ov + ou;
        &self.data[offset]
    }
}

impl<T> IndexMut<(usize, usize)> for BlockedArray<T>
where
T: Copy + Default
{
    fn index_mut(&mut self, i: (usize, usize)) -> &mut T {
        let bu = self.block(i.0);
        let bv = self.block(i.1);
        let ou = self.offset(i.0);
        let ov = self.offset(i.1);
        let mut offset = BLOCK_SIZE * BLOCK_SIZE * (self.u_blocks * bv + bu);
        offset += BLOCK_SIZE * ov + ou;
        &mut self.data[offset]
    }
}

#[inline]
fn round_up(x: usize) -> usize {
    (x + BLOCK_SIZE - 1) & !(BLOCK_SIZE - 1)
}