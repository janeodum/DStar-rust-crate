//! Compute a shortest path (or all shorted paths) using the [D* search
//! algorithm](https://en.wikipedia.org/wiki/D*#Pseudocode).

use indexmap::map::Entry::{Occupied, Vacant};
use num_traits::Zero;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};
use std::hash::Hash;
use std::iter::FusedIterator;
use std::usize;

use super::reverse_path;
use crate::FxIndexMap;


#[allow(clippy::missing_panics_doc)]
pub fn dstar<N, C, FN, IN, FH, FS>(
    start: &N,
    mut successors: FN,
    mut heuristic: FH,
    mut success: FS,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut to_see = BinaryHeap::new();
    to_see.push(SmallestCostHolder {
        estimated_cost: Zero::zero(),
        cost: Zero::zero(),
        index: 0,
    });
    let mut parents: FxIndexMap<N, (usize, C)> = FxIndexMap::default();
    parents.insert(start.clone(), (usize::max_value(), Zero::zero()));
    while let Some(SmallestCostHolder { cost, index, .. }) = to_see.pop() {
        let successors = {
            let (node, &(_, c)) = parents.get_index(index).unwrap(); // Cannot fail
            if success(node) {
                let path = reverse_path(&parents, |&(p, _)| p, index);
                return Some((path, cost));
            }
            // We may have inserted a node several time into the binary heap if we found
            // a better way to access it. Ensure that we are currently dealing with the
            // best path and discard the others.
            if cost > c {
                continue;
            }
            successors(node)
        };
        for (successor, move_cost) in successors {
            let new_cost = cost + move_cost;
            let h; // heuristic(&successor)
            let n; // index for successor
            match parents.entry(successor) {
                Vacant(e) => {
                    h = heuristic(e.key());
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = heuristic(e.key());
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_see.push(SmallestCostHolder {
                estimated_cost: new_cost + h,
                cost: new_cost,
                index: n,
            });
        }
    }
    None
}


/// The shortest paths starting from `start` up to a node for which `success` returns `true` are
/// computed and returned in an iterator along with the cost (which, by definition, is the same for
/// each shortest path), wrapped in a `Some`. If no paths are found, `None` is returned.
///
/// - `start` is the starting node.
/// - `successors` returns a list of successors for a given node, along with the cost for moving
/// from the node to the successor.
/// - `heuristic` returns an approximation of the cost from a given node to the goal. The
/// approximation must not be greater than the real cost, or a wrong shortest path may be returned.
/// - `success` checks whether the goal has been reached. It is not a node as some problems require
/// a dynamic solution instead of a fixed node.
///
/// A node will never be included twice in the path as determined by the `Eq` relationship.
///
/// Each path comprises both the start and an end node. Note that while every path shares the same
/// start node, different paths may have different end nodes.
#[allow(clippy::missing_panics_doc)]
pub fn dstar_bag<N, C, FN, IN, FH, FS>(
    start: &N,
    mut successors: FN,
    mut heuristic: FH,
    mut success: FS,
) -> Option<(DstarSolution<N>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut to_see = BinaryHeap::new();
    let mut min_cost = None;
    let mut sinks = HashSet::new();
    to_see.push(SmallestCostHolder {
        estimated_cost: Zero::zero(),
        cost: Zero::zero(),
        index: 0,
    });
    let mut parents: FxIndexMap<N, (HashSet<usize>, C)> = FxIndexMap::default();
    parents.insert(start.clone(), (HashSet::new(), Zero::zero()));
    while let Some(SmallestCostHolder {
        cost,
        index,
        estimated_cost,
        ..
    }) = to_see.pop()
    {
        if matches!(min_cost, Some(min_cost) if estimated_cost > min_cost) {
            break;
        }
        let successors = {
            let (node, &(_, c)) = parents.get_index(index).unwrap(); // Cannot fail
            if success(node) {
                min_cost = Some(cost);
                sinks.insert(index);
            }
            // We may have inserted a node several time into the binary heap if we found
            // a better way to access it. Ensure that we are currently dealing with the
            // best path and discard the others.
            if cost > c {
                continue;
            }
            successors(node)
        };
        for (successor, move_cost) in successors {
            let new_cost = cost + move_cost;
            let h; // heuristic(&successor)
            let n; // index for successor
            match parents.entry(successor) {
                Vacant(e) => {
                    h = heuristic(e.key());
                    n = e.index();
                    let mut p = HashSet::new();
                    p.insert(index);
                    e.insert((p, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = heuristic(e.key());
                        n = e.index();
                        let s = e.get_mut();
                        s.0.clear();
                        s.0.insert(index);
                        s.1 = new_cost;
                    } else {
                        if e.get().1 == new_cost {
                            // New parent with an identical cost, this is not
                            // considered as an insertion.
                            e.get_mut().0.insert(index);
                        }
                        continue;
                    }
                }
            }

            to_see.push(SmallestCostHolder {
                estimated_cost: new_cost + h,
                cost: new_cost,
                index: n,
            });
        }
    }

    min_cost.map(|cost| {
        let parents = parents
            .into_iter()
            .map(|(k, (ps, _))| (k, ps.into_iter().collect()))
            .collect();
        (
            AstarSolution {
                sinks: sinks.into_iter().collect(),
                parents,
                current: vec![],
                terminated: false,
            },
            cost,
        )
    })
}


pub fn dstar_bag_collect<N, C, FN, IN, FH, FS>(
    start: &N,
    successors: FN,
    heuristic: FH,
    success: FS,
) -> Option<(Vec<Vec<N>>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    dstar_bag(start, successors, heuristic, success)
        .map(|(solutions, cost)| (solutions.collect(), cost))
}

struct SmallestCostHolder<K> {
    estimated_cost: K,
    cost: K,
    index: usize,
}

impl<K: PartialEq> PartialEq for SmallestCostHolder<K> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl<K: PartialEq> Eq for SmallestCostHolder<K> {}

impl<K: Ord> PartialOrd for SmallestCostHolder<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Ord> Ord for SmallestCostHolder<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}

/// Iterator structure created by the `dstar_bag` function.
#[derive(Clone)]
pub struct DstarSolution<N> {
    sinks: Vec<usize>,
    parents: Vec<(N, Vec<usize>)>,
    current: Vec<Vec<usize>>,
    terminated: bool,
}

impl<N: Clone + Eq + Hash> DstarSolution<N> {
    fn complete(&mut self) {
        loop {
            let ps = match self.current.last() {
                None => self.sinks.clone(),
                Some(last) => self.parents(*last.last().unwrap()).clone(),
            };
            if ps.is_empty() {
                break;
            }
            self.current.push(ps);
        }
    }

    fn next_vec(&mut self) {
        while self.current.last().map(Vec::len) == Some(1) {
            self.current.pop();
        }
        self.current.last_mut().map(Vec::pop);
    }

    fn node(&self, i: usize) -> &N {
        &self.parents[i].0
    }

    fn parents(&self, i: usize) -> &Vec<usize> {
        &self.parents[i].1
    }
}

impl<N: Clone + Eq + Hash> Iterator for DstarSolution<N> {
    type Item = Vec<N>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.terminated {
            return None;
        }
        self.complete();
        let path = self
            .current
            .iter()
            .rev()
            .map(|v| v.last().copied().unwrap())
            .map(|i| self.node(i).clone())
            .collect::<Vec<_>>();
        self.next_vec();
        self.terminated = self.current.is_empty();
        Some(path)
    }
}

impl<N: Clone + Eq + Hash> FusedIterator for DstarSolution<N> {}
