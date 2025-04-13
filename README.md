Minimum Cost to Connect All Points
Problem Description
You are given an array points representing integer coordinates of some points on a 2D-plane, where points[i] = [xi, yi].
The cost of connecting two points [xi, yi] and [xj, yj] is the Manhattan distance between them: |xi - xj| + |yi - yj|.
Return the minimum cost to make all points connected. All points are connected if there is exactly one simple path between any two points.
Solution Approaches
This repository contains three different implementations of solutions to the problem:
1. Prim's Algorithm

Uses a min-heap to efficiently select the next edge with the smallest weight
Time Complexity: O(n² log n)
Space Complexity: O(n²)

2. Kruskal's Algorithm

Uses Union-Find (Disjoint Set) data structure with path compression and union by rank optimizations
Sorts all potential edges by weight and greedily selects the smallest edges
Time Complexity: O(n² log n)
Space Complexity: O(n²)

3. Optimized Prim's Algorithm

Uses an array-based approach to reduce constant factors
Time Complexity: O(n²)
Space Complexity: O(n)

Advanced Data Structures & Algorithms Used

Union-Find (Disjoint Set)

Path compression optimization
Union by rank for balanced tree heights


Priority Queue (Min-Heap)

Efficient selection of minimum weight edges


Graph Representation

Adjacency list using defaultdict
Complete graph with weighted edges


Minimum Spanning Tree Algorithms

Prim's Algorithm
Kruskal's Algorithm



Performance Analysis
The solution demonstrates multiple approaches with different performance characteristics:

The Kruskal's implementation sorts all edges which takes O(n² log n) time
Prim's implementation with a heap has better average-case performance on sparse graphs
The optimized implementation reduces the constants in the time complexity

Example Test Cases
Example 1:

Input: points = [[0,0],[2,2],[3,10],[5,2],[7,0]]
Output: 20

Example 2:

Input: points = [[3,12],[-2,5],[-4,1]]
Output: 18

Day X of 30 Days of LeetCode
This solution is part of my 30-day LeetCode challenge to improve my algorithm and data structure skills. This implementation showcases advanced graph algorithms, data structure optimization techniques, and algorithm analysis.
