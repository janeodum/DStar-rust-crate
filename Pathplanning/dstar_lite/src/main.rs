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
