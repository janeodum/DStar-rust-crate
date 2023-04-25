# DStar-rust-crate

This is an implementation of DStar and Dstar lite algorithm usign Rust.
D* is a search algorithm that is used for path planning. It works by iteratively selecting a node from the OPEN list and evaluating it. It then propagates the node's changes to all the neighboring nodes and places them on the OPEN list. This propagation process is termed "expansion".

The D* Lite algorithm was developed to provide a quick and effective method for robots to navigate through unfamiliar environments. It accomplishes this by searching for the shortest path from the destination towards the current position of the robot. The algorithm employs heuristics to optimize the search, and it efficiently manages the priority queue to minimize the need for reordering.

Like Dijkstra's algorithm and A*, D* maintains a list of nodes to be evaluated, known as the "OPEN list". Nodes are marked as having one of several states:
1) NEW, meaning it has never been placed on the OPEN list
2) OPEN, meaning it is currently on the OPEN list
3) CLOSED, meaning it is no longer on the OPEN list
4) RAISE, indicating its cost is higher than the last time it was on the OPEN list
5) LOWER, indicating its cost is lower than the last time it was on the OPEN list

Special Instructions: 
1) The D* code can be found under the Dstar folder, In addition, documentation under target folder.
2) The D* lite can be found under the Dstar lite folder, with documentation under the doc folder in the target folder

