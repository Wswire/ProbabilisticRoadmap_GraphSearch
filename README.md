## ProbabilisticRoadmap_GraphSearch

# Probabalistic Roadmap
Given a large state space, solve the motion planning problem of find a path from start to finish in the presence of obstacles.

## Method
Discretize
Random sample from state space to create a smaller state space  
Conduct a graph search to connect random sampled vertices, resulting in a graph with V vertices and E edges
Conduct graph search on new graph to get path from start to finish

# Chessboard
Given the dynamics of a knight on a chessboard, plan the path from any initial state to final state

## Method
bfs traversal of state space
