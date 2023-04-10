use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

// Define the type aliases for node ID, cost and heuristic function
type NodeId = u32;
type Cost = f32;
type HeuristicFn = fn(&NodeId, &NodeId) -> Cost;

// Define the struct for a node in the graph
#[derive(Copy, Clone)]
struct Node {
    g: Cost,
    rhs: Cost,
    predecessors: Vec<NodeId>,
}

// Define the struct for an item in the priority queue
struct QueueItem {
    id: NodeId,
    key: (Cost, Cost),
}

// Implement the comparison function for the priority queue
impl PartialEq for QueueItem {
    fn eq(&self, other: &QueueItem) -> bool {
        self.key.eq(&other.key)
    }
}

impl Eq for QueueItem {}

impl PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &QueueItem) -> Option<Ordering> {
        other.key.partial_cmp(&self.key)
    }
}

impl Ord for QueueItem {
    fn cmp(&self, other: &QueueItem) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

// Function to calculate the key value for a node
fn calculate_key(node: &Node, k_old: &(Cost, Cost)) -> (Cost, Cost) {
    let g = node.g;
    let rhs = node.rhs;
    let min_val = g.min(rhs);
    (min_val + k_old.0, min_val)
}

// Function to update the node and its neighbors
fn update_node(
    open_list: &mut BinaryHeap<QueueItem>,
    closed_list: &mut HashMap<NodeId, (Cost, Cost)>,
    nodes: &mut HashMap<NodeId, Node>,
    id: &NodeId,
    goal: &NodeId,
    k_old: &(Cost, Cost),
) {
    // Update the rhs value of the node
    let node = nodes.get_mut(id).unwrap();
    if *id != *goal {
        node.rhs = f32::INFINITY;
        for pred_id in node.predecessors.iter() {
            let pred = nodes.get(pred_id).unwrap();
            let cost = pred.g + 1.0; // Assume cost of 1 for each edge
            node.rhs = node.rhs.min(cost);
        }
    }

    // Check if the node is in the open list and remove it if necessary
    if open_list.iter().any(|i| i.id == *id) {
        open_list.retain(|i| i.id != *id);
    }

    // Check if the node's g and rhs values have changed and add it to the open list if necessary
    if node.g != node.rhs {
        let key = calculate_key(node, k_old);
        open_list.push(QueueItem {
            id: *id,
            key: key,
        });
    }

    // Update the nodes in the predecessor list of the node
    for pred_id in node.predecessors.iter() {
        update_node(open_list, closed_list, nodes, pred_id, goal, k_old);
    }
}

// Function to compare two cost values
fn compare(a: &Cost, b: &Cost) -> bool {
    let eps = 1e-6; // Define the tolerance for floating point comparisons
    (a - b).abs() < eps
}

// Function to run the D* Lite algorithm
fn d_star_lite(
    start: NodeId,
    goal: NodeId,
    nodes: &mut HashMap) -> Option<Vec<NodeId>> {
        let mut open_list: BinaryHeap<QueueItem> = BinaryHeap::new();
        let mut closed_list: HashMap<NodeId, (Cost, Cost)> = HashMap::new();

        // Initialize the start node
        let start_node = nodes.get_mut(&start).unwrap();
        start_node.g = f32::INFINITY;
        start_node.rhs = 0.0;
        update_node(&mut open_list, &mut closed_list, &mut nodes, &start, &goal, &k_old);
    
        // Run the D* Lite algorithm
        while u != start {
            // Get the neighbors of the current node
            let neighbors = get_neighbors(&u, &nodes, &map);
    
            // Update the g and rhs values of the neighbors
            for v in neighbors.iter() {
                update_node(&mut open_list, &mut closed_list, &mut nodes, &v, &goal, &k_old);
            }
    
            // If the start node has been updated, update all its neighbors
            if u == start {
                for v in neighbors.iter() {
                    update_node(&mut open_list, &mut closed_list, &mut nodes, &v, &goal, &k_old);
                }
            } else {
                // Otherwise, update the current node
                update_node(&mut open_list, &mut closed_list, &mut nodes, &u, &goal, &k_old);
            }
    
            // Set the new current node to be the node with the smallest key on the open list
            u = open_list.peek().unwrap().id;
        }
    
        // Reconstruct the path
        let mut path = Vec::new();
        let mut current = start;
        while current != goal {
            path.push(current);
            let neighbors = get_neighbors(&current, &nodes, &map);
            let mut min_cost = f32::INFINITY;
            let mut next = current;
            for neighbor in neighbors.iter() {
                let cost = nodes.get(&neighbor).unwrap().g + map.get(&(current, *neighbor)).unwrap();
                if cost < min_cost {
                    min_cost = cost;
                    next = *neighbor;
                }
            }
            current = next;
        }
        path.push(goal);
    
        Some(path)
    }

    fn main() {
        // Define the nodes of the graph
        let mut nodes = HashMap::new();
        let start = NodeId(0);
        let goal = NodeId(5);
        nodes.insert(start, Node::new(0.0, f32::INFINITY));
        nodes.insert(NodeId(1), Node::new(f32::INFINITY, f32::INFINITY));
        nodes.insert(NodeId(2), Node::new(f32::INFINITY, 4.0));
        nodes.insert(NodeId(3), Node::new(f32::INFINITY, 2.0));
        nodes.insert(NodeId(4), Node::new(f32::INFINITY, 3.0));
        nodes.insert(goal, Node::new(f32::INFINITY, 0.0));
    
        // Define the edges of the graph
        let mut edges = HashMap::new();
        edges.insert((NodeId(0), NodeId(1)), 1.0);
        edges.insert((NodeId(0), NodeId(3)), 3.0);
        edges.insert((NodeId(1), NodeId(2)), 1.0);
        edges.insert((NodeId(2), NodeId(4)), 3.0);
        edges.insert((NodeId(3), NodeId(2)), 1.0);
        edges.insert((NodeId(3), NodeId(4)), 1.0);
        edges.insert((NodeId(4), NodeId(5)), 1.0);
    
        // Run the D* Lite algorithm
        let path = d_star_lite(start, goal, &mut nodes);
    
        // Print the path if one is found
        if let Some(path) = path {
            println!("Path found: {:?}", path);
        } else {
            println!("No path found");
        }
    }
    
    

