/// Compute the shortest path using the D*
/// This is a variate of the A* algorithm as it accounts for dynamic changing environment.
///
/// - `start` is the starting node.
/// - `successors` returns a list of successors for a given node, along with the cost for moving
/// from the node to the successor.
/// - `h` returns an approximation of the cost from a given node to the goal.
/// - `success` checks whether the goal has been reached. 
///
/// A node will never be included twice in the path as determined by the `Eq` relationship.
///

use std::fmt::Debug;

use num_traits::Zero;
use rand::Rng;


pub fn dstar<N, C, FN, IN, FH, FS>(
    start: &N,
    mut successors: FN,
    mut h: FH,
    mut success: FS,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Clone + Debug,
    C: Zero + Ord + Copy + Debug,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut bound = h(start);
    let mut path = vec![start.clone()]; //vector containing path to the goal starting from the start location
    loop {
        match compute_shortest_path(
            &mut path,
            Zero::zero(), //Cost to reach the node from the start node
            bound,
            &mut successors,
            &mut h, // Heuristic cost from the node to the GOALLOC node
            &mut success,
        ) { 
            //ComputeShortestPathes for a path to the goal location, if found returns the coordinate and cost
            Path::Found(path, cost) => return Some((path, cost)), 
            Path::MinimumPath(min) => {
                if bound == min {
                    return None;
                }
                bound = min;
            }
            Path::NoTraversal => return None,
        }
    }
}

enum Path<N, C> {
    Found(Vec<N>, C),
    MinimumPath(C),
    NoTraversal,
}
//computes the shortest path and dynamically change the cost of each edges to account for changing obstacle or changing environemnt
fn compute_shortest_path<N, C, FN, IN, FH, FS>(
    path: &mut Vec<N>,
    cost: C,
    bound: C,
    successors: &mut FN,
    h: &mut FH,
    success: &mut FS,
) -> Path<N, C>
where
    N: Eq + Clone + Debug,
    C: Zero + Ord + Copy + Debug,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let neighbours = {
        let start = &path[path.len() - 1];
        let f = cost + h(start);
        if f > bound {
            return Path::MinimumPath(f);
        }
        if success(start) {
            return Path::Found(path.clone(), f);
        }
        let mut neighbours = successors(start)
            .into_iter()
            .filter_map(|(n, c)| {
                (!path.contains(&n)).then(|| {
                    let h = h(&n);
                    (n, c, c + h)
                })
            })
            .collect::<Vec<_>>();
        neighbours.sort_unstable_by(|(_, _, c1), (_, _, c2)| c1.cmp(c2));
        neighbours
    };
    let mut min = None;
    for (node, extra, _) in neighbours {
        path.push(node);
        match compute_shortest_path(path, cost + extra, bound, successors, h, success) {
            found_path @ Path::Found(_, _) => return found_path,
            Path::MinimumPath(m) => match min {
                None => min = Some(m),
                Some(n) if m < n => min = Some(m),
                Some(_) => (),
            },
            Path::NoTraversal => (),
        }
        path.pop();
    }
    min.map_or(Path::NoTraversal, Path::MinimumPath)
}

fn main(){
    static GOALLOC: (i32, i32) = (8, 6);
    let result = dstar(&(1, 1),
                   |&(x, y)| vec![(x+4,y+1), (x+1,y-5), (x-1,y+2), (x-1,y-2),
                                  (x+2,y+1), (x+2,y-1), (x-2,y+1), (x-2,y-9), (x-3,y+5), (x-1,y+9), (x-7,y+4), (x-5,y+2), (x-4,y+3)]
                              .into_iter().map(|p| (p, 1)),
                   |&(x, y)| (GOALLOC.0.abs_diff(x) + GOALLOC.1.abs_diff(y)) / 5,
                   |&p| p == GOALLOC);
    println!("Path: {:?}", result);
}