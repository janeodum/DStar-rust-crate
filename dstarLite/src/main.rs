/// Compute the shortest path using the D*Lite
/// This is a variate of the D* algorithm with more improved functionality.
/// it also reuses information from  previous nodes
/// - `start` is the starting node.
/// - `successors` returns a list of successors for a given node, along with the cost for moving
/// from the node to the successor.
/// - `Predecessors` returns a list of predecessor for a given node, along with the cost for moving
/// from the node to the predecessors.
/// - `h` returns an approximation of the cost from a given node to the goal.
/// - `g` Cost to reach the node from the start node. 
///
/// A node will never be included twice in the path as determined by the `Eq` relationship.
///

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::num::Wrapping;
use rand::Rng;

type NodeId = i32;
type Cost = i32;

#[derive(Clone, Debug, PartialEq)]
struct  Node {
    x: i32,
    y: i32,
    g: Cost, // Cost to reach the node from the start node
    rhs: Cost, // Cost to reach the node from the start node through the current best path
    h: Cost, // Heuristic cost from the node to the goal node
    successors: Vec<NodeId>,
    predecessors: Vec<NodeId>,
}

#[derive(Debug, Eq, PartialEq)]
struct QueueItem {
    id: NodeId,
    key: (Cost, Cost),
}

impl Ord for QueueItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other.key.cmp(&self.key)
    }
}

impl PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn calculate_key(node: &Node, k_old: &(Cost, Cost)) -> (Cost, Cost) {
    let g = node.g;
    let rhs = node.rhs;
    let mut rng = rand::thread_rng();
    let h = rng.gen_range(1,10) as i32; // TODO: calculate the heuristic value
    let min_val = g.min(rhs);
    (min_val + h + k_old.0, min_val + k_old.1)
}

fn compare(a: &(Cost, Cost), b: &(Cost, Cost)) -> bool {
    if a.0 < b.0 {
        true
    } else if a.0 == b.0 && a.1 < b.1 {
        true
    } else {
        false
    }
}

fn update_node(
    open_list: &mut BinaryHeap<QueueItem>,
    closed_list: &mut HashMap<NodeId, (Cost, Cost)>,
    nodes: &mut HashMap<NodeId, Node>,
    u: &NodeId,
    goal: &NodeId,
    k_old: &(Cost, Cost),
) {
    let node = nodes.get(&u).unwrap();
    if u != goal {
        let mut rhs_val = 70;
        let mut rng = rand::thread_rng();
        for pred_id in node.predecessors.iter() {
            let pred_node = nodes.get(&pred_id).unwrap();
            //println!("{} days", pred_node.g);
            let val = pred_node.g + pred_node.successors.iter().map(|&suc_id| {
                let suc_node = nodes.get(&suc_id).unwrap();
                if suc_node.g != i32::MIN {
                    let cost = rng.gen_range(1,20) as i32; // TODO: calculate the cost between pred_node and suc_node
                    pred_node.g + cost
                } else {
                    i32::MIN
                }
            }).min_by(|&a, &b| a.partial_cmp(&b).unwrap_or(Ordering::Equal)).unwrap_or(i32::MIN);
            if val < rhs_val {
                rhs_val = val;
            }
        }
       //node.rhs = rhs_val;
    }
    if let Some(k) = closed_list.get(u) {
        if !compare(&calculate_key(node, k_old), k) {
            return;
        }
    }
    //node.g = node.rhs;
    closed_list.remove(u);
    open_list.iter().find(|&item| item.id == *u);
    open_list.push(QueueItem { id: *u, key: calculate_key(node, k_old) });
}

fn d_star_lite(start: NodeId, goal: NodeId, mut nodes: &mut HashMap<NodeId, Node>) -> Option<Vec<NodeId>> {
    let  u: NodeId = goal;
    let  k_old: (Cost, Cost) = (0, 0);
    let mut closed_list: HashMap<NodeId, (Cost, Cost)> = HashMap::new();

    // Initialize the priority queue with the start node
    let mut open_list: BinaryHeap<QueueItem> = BinaryHeap::new();
    let start_node = nodes.get_mut(&start).unwrap();
    let start_key = calculate_key(&start_node, &k_old);
    open_list.push(QueueItem { id: start, key: start_key });
    
    
    //println!("{:?} empty", open_list);
    // Update the rhs value of the start node
    nodes.get_mut(&start).unwrap().rhs = 0;

    while !open_list.is_empty() && (compare(&open_list.peek().unwrap().key, &calculate_key(&nodes.get(&u).unwrap(), &k_old)) || nodes.get(&u).unwrap().rhs != nodes.get(&u).unwrap().g) {
        // Pop the node with the smallest key from the open list
        let QueueItem { id: current, key: _ } = open_list.pop().unwrap();

        // Check if the current node has been expanded previously
        if let Some(k) = closed_list.get(&current) {
            if compare(&calculate_key(&nodes.get(&current).unwrap(), &k_old), k) {
                // If the node is consistent, update its g value
                nodes.get_mut(&current).unwrap().g = nodes.get(&current).unwrap().rhs;
            } else {
                // If the node is inconsistent, update its rhs value and add it to the open list
                nodes.get_mut(&current).unwrap().g = i32::MIN;
                update_node(&mut open_list, &mut closed_list, &mut nodes, &current, &goal, &k_old);
            }
        } else {
            // Add the current node to the closed list
            let k = calculate_key(&nodes.get(&current).unwrap(), &k_old);
            closed_list.insert(current, k);

            // Update the g and rhs values of the current node
            if current != goal {
                nodes.get_mut(&current).unwrap().g = i32::MIN;
                update_node(&mut open_list, &mut closed_list, &mut nodes, &current, &goal, &k_old);
            }
        }

        // Update the key values of the nodes in the open list if necessary
        if let Some(k) = closed_list.get(&current) {
            if compare(k, &calculate_key(&nodes.get(&current).unwrap(), &k_old)) {
                update_node(&mut open_list, &mut closed_list, &mut nodes, &current, &goal, &k_old);
            } else if nodes.get(&current).unwrap().g > nodes.get(&current).unwrap().rhs {
                let new_key = calculate_key(&nodes.get(&current).unwrap(), &k_old);
                open_list.push(QueueItem { id: current, key: new_key });
            } else {
                let new_key = calculate_key(&nodes.get(&current).unwrap(), &k_old);
                open_list.push(QueueItem { id: current, key: new_key });
                update_node(&mut open_list, &mut closed_list, &mut nodes, &current, &goal, &k_old);
            }
        } else {
            let new_key = calculate_key(&nodes.get(&current).unwrap(), &k_old);
            open_list.push(QueueItem { id: current, key: new_key });
            update_node(&mut open_list, &mut closed_list, &mut nodes, &current, &goal, &k_old);
        }
    }

    // Check if a path was found
    if nodes.get(&start).unwrap().rhs == i32::MIN {
        return None;
    }

    // Build the path
    let mut path: Vec<NodeId> = vec![start];
    let mut current = start;

    while current != goal {
        let successors = nodes.get(&current).unwrap().successors.iter().cloned().collect::<Vec<NodeId>>();
        let mut min_g = i32::MIN;
        let mut next: Option<NodeId> = None;

        for s in successors {
            
            //println!("{} empty", nodes.get(&current).unwrap().g);
            let cost = nodes.get(&s).unwrap().g + 12;
            if cost < min_g {
                min_g = cost;
                next = Some(s);
            }
        }

        match next {
            Some(n) => {
                path.push(n);
                current = n;
            },
            None => {
                // No path found
                return None;
            }
        }
    }

    Some(path)
}


fn main() {
    // Define the graph
    let mut nodes: HashMap<NodeId, Node> = HashMap::new();
    // nodes.insert(0, Node::new(0, vec![(1, 1.0), (2, 4.0)]));
    // nodes.insert(1, Node::new(1, vec![(3, 3.0)]));
    // nodes.insert(2, Node::new(2, vec![(1, 1.0), (3, 1.0)]));
    // nodes.insert(3, Node::new(3, vec![(4, 2.0)]));
    // nodes.insert(4, Node::new(4, vec![]));

    // Run the D* Lite algorithm
    let start = 0;
    let goal = 4;
    nodes.insert(0, Node { x: 0, y: 0, g: 8, rhs: 4 , h:16, successors: vec![9, 6], predecessors: vec![2]});
    nodes.insert(1, Node { x: 1, y: 0, g: 4, rhs: 3, h:8 , successors: vec![5], predecessors: vec![4]});
    nodes.insert(2, Node { x: 2, y: 0, g: 9, rhs: 8, h:15, successors: vec![7], predecessors: vec![5]});
    nodes.insert(3, Node { x: 3, y: 0, g: 6, rhs: 2, h:17, successors: vec![8], predecessors: vec![1]});
    nodes.insert(4, Node { x: 4, y: 0, g: 8, rhs: 1, h:13, successors: vec![1], predecessors: vec![2] });
    nodes.insert(5, Node { x: 5, y: 0, g: 1, rhs: 8, h:8, successors: vec![0], predecessors: vec![2] });
    nodes.insert(6, Node { x: 6, y: 0, g: 8, rhs: 3, h:16, successors: vec![5], predecessors: vec![3]});
    nodes.insert(7, Node { x: 7, y: 0, g: 7, rhs: 6, h:19, successors: vec![6], predecessors: vec![6]});
    nodes.insert(8, Node { x: 8, y: 0, g: 8, rhs: 5, h:11, successors: vec![3], predecessors: vec![5]});
    nodes.insert(9, Node { x: 9, y: 0, g: 2, rhs: 9, h:10, successors: vec![6], predecessors: vec![2]});
    let mut path = d_star_lite(start, goal,  &mut nodes);
    println!("Path: {:?}", path);
    // Print the path
    match path {
        Some(p) => println!("Path: {:?}", p),
        None => println!("No path found"),
    }

    // Modify the graph
    nodes.get_mut(&1).unwrap().predecessors.insert(1, 5);

    // Re-run the D* Lite algorithm
    path = d_star_lite(start, goal, &mut nodes);

    // Print the path
    match path {
        Some(p) => println!("Path: {:?}", p),
        None => println!("No path found"),
    }
}
