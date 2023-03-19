use ndarray::{Array2, ArrayView2};
use std::cmp::Ordering;

#[derive(Copy, Clone, Debug, PartialEq)]
enum State {
    New,
    Open,
    Closed,
}

#[derive(Copy, Clone, Debug)]
struct Node {
    x: usize,
    y: usize,
    rhs: f32,
    g: f32,
    state: State,
}

impl Node {
    fn new(x: usize, y: usize, g: f32) -> Self {
        Self {
            x,
            y,
            g,
            rhs: f32::MAX,
            state: State::New,
        }
    }

    fn key(&self) -> (f32, f32) {
        (self.rhs + self.g, self.rhs)
    }

    fn is_new(&self) -> bool {
        self.state == State::New
    }

    fn is_open(&self) -> bool {
        self.state == State::Open
    }

    fn is_closed(&self) -> bool {
        self.state == State::Closed
    }

    fn set_new(&mut self) {
        self.state = State::New;
    }

    fn set_open(&mut self) {
        self.state = State::Open;
    }

    fn set_closed(&mut self) {
        self.state = State::Closed;
    }
}

// This implementation takes in a 2D boolean map representing the environment, where true values represent obstacles and false values represent free spaces. It also takes in the start and goal positions as tuples of row and column indices.

// The function creates a new 2D array of Node structs, initialized with the appropriate g and rhs values based on the map. It sets the g value of the goal node to infinity and the rhs value to 0, and sets the g value of the start node to 0 and the rhs value to its key.

// The function returns the 2D array of nodes and the goal node as a tuple.

fn initialize(map: &Array2<bool>, start: (usize, usize), goal: (usize, usize)) -> (Array2<Node>, Node) {
    let (nrows, ncols) = map.dim();

    let mut nodes = Array2::from_shape_fn((nrows, ncols), |(i, j)| {
        if map[(i, j)] {
            Node::new(i, j, f32::INFINITY)
        } else {
            Node::new(i, j, f32::MAX)
        }
    });

    let goal_node = nodes.get_mut(goal.0, goal.1).unwrap();
    goal_node.g = f32::INFINITY;
    goal_node.rhs = 0.0;

    let start_node = nodes.get_mut(start.0, start.1).unwrap();
    start_node.g = 0.0;
    start_node.rhs = start_node.key().1;

    (nodes, *goal_node)
}
// This implementation takes in a 2D array of Node structs nodes, a Node node representing the current node, and a successor position as a tuple (usize, usize).

// The function first checks if the successor position is the same as the current node's position, in which case it returns a cost of 0.

// Next, it checks if the successor position is within the bounds of the nodes array. If it's not, the function returns None.

// If the successor position is valid, the function retrieves the Node struct corresponding to that position. If that node is closed, the function also returns None.

// Finally, the function computes the cost to get from the current node to the successor node based on their positions and g-values, and returns it as an Option<f32>. If any of the checks fail, the function returns None.
fn cost_compute(
    nodes: &ArrayView2<Node>,
    node: &Node,
    successor: &(usize, usize),
) -> Option<f32> {
    let (x, y) = successor;
    let (nx, ny) = (node.x as isize, node.y as isize);
    let (sx, sy) = (nodes.raw_dim()[0] as isize, nodes.raw_dim()[1] as isize);

    if *successor == (node.x, node.y) {
        return Some(0.0);
    }

    if !((0..sx).contains(&nx + x as isize) && (0..sy).contains(&ny + y as isize)) {
        return None;
    }

    let successor_node = nodes.get(*x, *y)?;

    if successor_node.is_closed() {
        return None;
    }

    let dx = (x as isize - nx).abs();
    let dy = (y as isize - ny).abs();
    let cost = if dx + dy == 1 {
        1.0
    } else {
        1.4142135623730951
    };

    Some(cost + successor_node.g - node.g)
}

// update_vertex function works:

// First, the function retrieves a mutable reference to the node to update from the nodes array.

// Then, if the current node is not the goal node, the function calculates a new rhs (right-hand-side) value for the node by considering the costs of reaching each of its successors. If a successor node is "new" (i.e., has not been visited before), its rhs value is set to the cost of reaching it plus its current g value. If a successor node is "open" (i.e., has been visited before and is still open for expansion), its rhs value is set to the minimum of its current rhs value and the cost of reaching it plus the g value of the current node plus the cost of the edge between the two nodes. If a successor node is "closed" (i.e., has been visited before but is not open for expansion anymore), its rhs value is not updated.

// Once the rhs values of all the successors have been considered, the function updates the rhs value of the current node to be the minimum of the rhs values of all its successors.

// If the current node is open, the function updates its g value to be the minimum of its rhs value and its current g value. If the g value has changed, the node is marked as closed and its neighbors are recursively updated using the update_vertex function. If the g value has not changed, the node remains open.

// If the current node is not open but is new, it is marked as open and its neighbors are recursively updated using the update_vertex function.

// If the current node is not open and is not new, its g value and rhs value may need to be updated based on the g values and rhs values of its predecessors. If the rhs value of the current node is equal to its previous g value, the function checks if any predecessor nodes can reach the current node with a lower cost than before. If so, the function updates the rhs value of the current node accordingly. Then, the function checks if the g value of the current node needs to be updated based on its new rhs value. If the g value has changed, the node is marked as open and its neighbors are recursively updated using the update_vertex function. If the g value has not changed, the node remains closed.
fn update_vertex(
    nodes: &mut Array2<Node>,
    node: &(usize, usize),
    start: &(usize, usize),
    goal: &Node,
) {
    let mut u = nodes.get_mut(node.0, node.1).unwrap();

    if u != goal {
        let mut rhs = f32::INFINITY;

        for successor in successors() {
            if let Some(cost) = cost_compute(&nodes.view(), &u, &successor) {
                let (x, y) = successor;
                let successor_node = nodes.get(x, y).unwrap();

                if successor_node.is_new() {
                    rhs = rhs.min(cost + successor_node.g);
                } else if successor_node.is_open() {
                    rhs = rhs.min(cost + successor_node.g);
                    let pred_cost = cost_compute(&nodes.view(), &successor_node, node).unwrap();
                    if pred_cost + u.g < successor_node.g {
                        successor_node.rhs = pred_cost + u.g;
                        successor_node.g = pred_cost + u.g;
                        nodes.get_mut(successor.0, successor.1).unwrap().set_open();
                    }
                }
            }
        }

        u.rhs = rhs;
    }

    if u.is_open() {
        let k_old = u.key();
        let k_new = (u.rhs + u.g, u.rhs);
        if k_old < k_new {
            nodes.get_mut(u.x, u.y).unwrap().g = u.rhs + u.g;
            nodes.get_mut(u.x, u.y).unwrap().set_closed();

            for neighbor in neighbors(node) {
                if let Some(cost) = cost_compute(&nodes.view(), &u, &neighbor) {
                    update_vertex(nodes, &neighbor, start, goal);
                }
            }
        } else {
            let mut g_old = u.g;
            let mut rhs_old = u.rhs;
            u.g = f32::INFINITY;
            u.rhs = u.key().1;

            for predecessor in predecessors() {
                if let Some(cost) = cost_compute(&nodes.view(), &predecessor, node) {
                    let (x, y) = predecessor;
                    let predecessor_node = nodes.get(x, y).unwrap();

                    if predecessor_node == goal {
                        continue;
                    }

                    if predecessor_node.rhs + cost == g_old {
                        if let Some(new_cost) = cost_compute(&nodes.view(), &predecessor, &u) {
                            rhs_old = rhs_old.min(predecessor_node.g + new_cost);
                        }
                    }
                }
            }

            u.rhs = rhs_old;
            if u.rhs != g_old {
                u.set_open();
            }

            for neighbor in neighbors(node) {
                if let Some(cost) = cost_compute(&nodes.view(), &u, &neighbor) {
                    update_vertex(nodes, &neighbor, start, goal);
                }
            }
        }
    } else if u.is_new() {
        u.rhs = u.key().1;
        u.set_open();

        for neighbor in neighbors(node) {
            if let Some(cost) = cost_compute(&nodes.view(), &u, &neighbor) {
                update_vertex(nodes, &neighbor, start, goal);
            }
        }
    } else {
        u.set_open();

        for neighbor in neighbors(node) {
            if let Some(cost) = cost_compute(&nodes.view(), &u, &neighbor) {
                update_vertex(nodes, &neighbor, start, goal);
            }
        }
    }
}
// The calculate_key function takes a reference to a Node struct and returns a tuple of two f64 values. The first value, k1, is calculated as the sum of the smaller of the g and rhs values for the node, and the heuristic estimate of the distance from the node to the goal. The second value, k2, is simply the smaller of the g and rhs values.

// The returned tuple is used as the key for the priority queue, so that nodes with smaller keys are popped from the queue first. The choice of k1 and k2 for the key values ensures that nodes with smaller g and rhs values, and nodes closer to the goal, are prioritized by the algorithm.

fn calculate_key(node: &Node) -> (f64, f64) {
    let k1 = f64::min(node.g, node.rhs) + heuristic(node, &node.goal);
    let k2 = f64::min(node.g, node.rhs);
    (k1, k2)
}

// we initialize a priority queue with a single state consisting of the start node, and then repeatedly pop the state with the smallest key value from the priority queue until we reach the goal node or the priority queue is empty. For each node popped from the priority queue, we update its g and rhs values using the update_vertex function, and then iterate over its neighbors to see if we can improve their values. If a neighbor's g or rhs value is updated, we push it onto the priority queue with a new key value computed using the calculate_key function.

// Once we have found the optimal path from the start node to the goal node, we construct it by starting at the goal node and repeatedly moving to its neighbor with the smallest rhs value until we reach the start node. The path is then printed to the console.

fn main() {
    // create the initial search graph
    let mut nodes = vec![
        Node { x: 0, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 0, y: 1, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 0, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 1, y: 1, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 1, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 2, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 3, y: 2, g: 0.0, rhs: 0.0 },
        Node { x: 4, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 5, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 6, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 7, y: 2, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 7, y: 1, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 7, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 6, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 5, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 4, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 3, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 2, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 1, y: 0, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
        Node { x: 1, y: 3, g: std::f64::INFINITY, rhs: std::f64::INFINITY },
    ];

    // set the start and goal nodes
    let start = &mut nodes[1];
    let goal = &mut nodes[18];
    start.g = 0.0;
    start.rhs = 0.0;

    // initialize the priority queue
    let mut queue = BinaryHeap::new();
    queue.push(State { key: calculate_key(start), node: start });

    // run the algorithm
    while let Some(State { node, .. }) = queue.pop() {
        if node == goal {
            break;
        }

        node.visited = true;

        update_vertex(&mut nodes, &mut queue, node);

        for neighbor in get_neighbors(&nodes, node) {
            if !neighbor.visited {
                let new_g = node.g + cost(&nodes, node, neighbor);
                if new_g < neighbor.g {
                    neighbor.rhs = new_g + heuristic(&neighbor, goal);
                    neighbor.g = new_g;
                    queue.push(State { key: calculate_key(&neighbor), node: neighbor });
                } else if new_g < neighbor.rhs {
                    neighbor.rhs = new_g;
                    queue.push(State { key: calculate_key(&neighbor), node: neighbor });
                }
            }
        }
    }

    // get the path
    let mut path = vec![goal];
    let mut current = goal;

    while current != start {
        let neighbors = get_neighbors(&nodes, current);
        let mut min_rhs = std::f64::INFINITY;
        let mut next_node = current;

        for neighbor in neighbors {
            let rhs = neighbor.g + cost(&nodes, neighbor, current);
            if rhs < min_rhs {
                min_rhs = rhs;
                next_node = neighbor;
            }
        }

        current = next_node;
        path.push(current);
    }

    path.reverse();

    // print the path
    println!("Optimal path:");
    for node in &path {
        println!("({}, {})", node.x, node.y);
    }
}
