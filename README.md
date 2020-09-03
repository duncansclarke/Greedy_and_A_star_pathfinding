# Pathfinding Algorithms

### Authors

- Aubrey McLeod
- Duncan Clarke
- Lianne Orlowski
- Taylor Jones
- Jiaoqi Liu

## Summary

This program was written collaboratively as a part of CISC 352 - Artificial Intelligence at Queen's university.

This project uses two algorithms, Greedy and A\* to solve the pathfinding problem. For the first part, the agent can move up, down, left, and right, but not diagonally. For the second part, the agent can move in four directions as well as diagonally.

Two types of moves (with or without diagonal moves) use different kinds of heuristics. Because heuristic is admissible, its estimated cost for a node should never exceed the true cost from a node to goal. However, in order to have the most informed heuristic, the estimated cost should be as similar as the true cost. At the end, Manhattan distance is used for pathfinding without diagonal moves, and Chebyshev distance is used for pathfinding with diagonal moves. Manhattan distance is the distance between two points measured along axes at right angles. Chebyshew distance is a metric defined on a vector space where the distance between two vectors is the greatest of their differences along any coordinate dimension.

There are differences in the use of heuristics in diagonal move is because heuristics should always be admissible. For pathfinding with diagonal moves, the weight of one move is 1. Both Manhattan distance and Euclidean distance would overestimate the cost. Therefore, only Chebyshew distance works here.

As for side notes, when choosing nodes, for Greedy Algorithm, it only uses the heuristic value. But for A\* algorithm, it takes both heuristic value and actural cost into consideration.

## Methods

**class grid_node**
This class represents a single point on our grid. At first, we initialize the cost of travelling in this direction as calculated through the heuristic and the cost from the start point to 0. Then, we check the value of the given node to see if it is navigable or not. We also noted the parent node, for latter 2 use in pricing and reversing for the route during pathfinding. A flag is created to represent the state of the current node. “0” being new, “1” being open and “2” being closed. We use the current nodes’ XY coordinate to represent its position.

**class grid**
Wrapper class for a 2d array.

**class solver()**
In this class, we initialize the grid of tiles, the start coordinates for the START, the end coordinates for the GOAL. Then we create an open list for the queue of available tiles, and a closed list for the queue of tiles that are more or less settled. Due to a slight modification as suggested through the wikipedia article for A\*, closed tiles may be reopened if a shorter path to them is found.

**manhattan \_search(self, node)**
This is a search method that finds only the non-diagonal adjacent nodes to a given node.

**full_search(self,node)**
This a search method that finds all adjacent nodes to a given node.

**get_path(self, search_function, heuristic, memory)**
This is our path finding function. It takes a search function to determine how it looks for adjacent nodes, a heuristic to determine what the cost of a given node is, and a flag to determine whether or not the path finder will remember the cost of arriving at a given node. In order to not reopen a closed node, we double checked the current node's flag status to make sure the node is closed at this point. While visiting nodes, the method keeps checking if we have reached the goal yet. If not, it continues to look at the neighbours of the current node. If we have never seen this node before, then its parent is the currently examined node. By looking at the memory, we consider if the cost from the start to the current position is being stored already. Once the result is true, then we check if the known cost to arrive at the neighbor node is greater than the cost of arriving at it through the current node. Last, we change the node’s status to open. However, if this node was not already
closed or set to closed, keep updating its heuristic cost and mark it as open. At the end, we push the node’s neighbor onto the queue of open nodes.

**get_chebyshev_distance(self, target)**
This function is the heuristic function for diagonal path findings for both greedy and A*. It takes
two tuples, *self* and *target\*, which indicate the positions in the map for current node and
target node. It returns the chebyshev distance between them.

**get_manhattan_distance(self, target)**
This function is the heuristic function for non-diagonal path findings for both greedy and A*. It takes two tuples, *self* and *target\*, which indicate the positions in the map for current node and target node. It returns the manhattan distance between them.

**read_grids(src)**
This function takes the _src_ as its parameter and reads the grid map. 'X' is wall. '\_' is clear. 'S' is the start, and 'G' is the goal.

**trace_back(end_node)**
This function traces back the parent node of the current node.

**generate_string(end_node, grid)**
This function works for all pathfinding in this assignment. It takes two inputs, the _end_node_ and
the _grid_. The purpose of it is to mark the path with ‘P' on the map.

**main()**
The main function first opens and then reads the source file. Then it calls both search functions (Manhattan and Chebyshev) and applies them to the source file, then gets the calculated distance as a result. After that, it writes to/formats the output file for the results of our pathfinding program.
