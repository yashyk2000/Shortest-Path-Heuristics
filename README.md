# Shortest-Path-Heuristics

Consider the graph presented below. Each node represents a single state of the US. If
two states are neighbors, there is an edge between them.
Assuming that edge weights represent driving distances between state capitals. Here
the problem is to find the shortest path between any two cities.

Problem Statement:
input/output specification:
driving.csv - with driving distances between state capitals.
straightline.csv - with straight line distances between state capitals.
Numerical data in both files is either:
● a non-negative integer corresponding to the distance between two state capitals
● negative integer -1 indicating that there is no direct “road” (no edge on the graph
below) between two state capitals.
This program accept two command line arguments corresponding to two states(initial
and goal states) so code could be executed with
python main.py INITIAL GOAL
where:
main.py is python code file name,
INITIAL is the name of the initial state,
GOAL is the name of the final state.
Example: python main.py WA TX




