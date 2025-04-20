import heapq
from collections import defaultdict

class Solution:
    def minCostConnectPoints(self, points):aaaaaaaa
        if not points or len(points) <= 1:
            return 0
        
        n = len(points)
        
        # Implementation using Prim's Algorithm
        # Calculate Manhattan distance between two points
        def manhattan_distance(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
        
        # Create adjacency list representation of the graph
        graph = defaultdict(list)
        for i in range(n):
            for j in range(i+1, n):
                dist = manhattan_distance(points[i], points[j])
                graph[i].append((dist, j))
                graph[j].append((dist, i))
        
        # Prim's algorithm
        total_cost = 0
        visited = set([0])  # Start with node 0
        min_heap = graph[0].copy()  # Initialize min heap with edges from node 0
        heapq.heapify(min_heap)
        
        while min_heap and len(visited) < n:
            cost, next_node = heapq.heappop(min_heap)
            if next_node in visited:
                continue
                
            visited.add(next_node)
            total_cost += cost
            
            # Add edges from newly visited node
            for new_cost, neighbor in graph[next_node]:
                if neighbor not in visited:
                    heapq.heappush(min_heap, (new_cost, neighbor))
        
        return total_cost
    
    def minCostConnectPoints_kruskal(self, points):
        if not points or len(points) <= 1:
            return 0
            
        n = len(points)
        
        # Union-Find (Disjoint Set) implementation
        parent = list(range(n))
        rank = [0] * n
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])  # Path compression
            return parent[x]
        
        def union(x, y):
            root_x = find(x)
            root_y = find(y)
            
            if root_x == root_y:
                return False
                
            # Union by rank for better performance
            if rank[root_x] < rank[root_y]:
                parent[root_x] = root_y
            elif rank[root_x] > rank[root_y]:
                parent[root_y] = root_x
            else:
                parent[root_y] = root_x
                rank[root_x] += 1
                
            return True
        
        # Create edges with weightsaaaaa
        edges = []
        for i in range(n):
            for j in range(i+1, n):
                weight = abs(points[i][0] - points[j][0]) + abs(points[i][1] - points[j][1])
                edges.append((weight, i, j))
        
        # Sort edges by weight
        edges.sort()
        
        # Kruskal's algorithm
        total_cost = 0
        edges_used = 0
        
        for weight, u, v in edges:
            if union(u, v):
                total_cost += weight
                edges_used += 1
                
                # Early termination
                if edges_used == n - 1:
                    break
        
        return total_cost
    
    def minCostConnectPoints_optimized(self, points):
        """Optimized implementation with improved time complexity"""
        if not points or len(points) <= 1:
            return 0
            
        n = len(points)
        
        # Use Prim's algorithm with optimized data structures
        visited = [False] * n
        heap_dict = {}  # To quickly update distances
        min_cost = [float('inf')] * n
        min_cost[0] = 0
        
        total_cost = 0
        remaining_nodes = n
        
        while remaining_nodes > 0:
            # Find minimum-cost vertex not yet visited
            curr_node = -1
            curr_min_cost = float('inf')
            
            for i in range(n):
                if not visited[i] and min_cost[i] < curr_min_cost:
                    curr_min_cost = min_cost[i]
                    curr_node = i
            
            # Mark current node as visited
            visited[curr_node] = True
            total_cost += curr_min_cost
            remaining_nodes -= 1
            
            # Update distances to all unvisited neighbors
            x1, y1 = points[curr_node]
            for next_node in range(n):
                if not visited[next_node]:
                    x2, y2 = points[next_node]
                    distance = abs(x1 - x2) + abs(y1 - y2)
                    min_cost[next_node] = min(min_cost[next_node], distance)
        
        return total_cost

# Example usage
if __name__ == "__main__":
    solution = Solution()
    
    # Example 1: [[0,0],[2,2],[3,10],[5,2],[7,0]]
    # Expected output: 20
    points1 = [[0,0],[2,2],[3,10],[5,2],[7,0]]
    print(f"Example 1 (Prim's): {solution.minCostConnectPoints(points1)}")
    print(f"Example 1 (Kruskal's): {solution.minCostConnectPoints_kruskal(points1)}")
    print(f"Example 1 (Optimized): {solution.minCostConnectPoints_optimized(points1)}")
    
    # Example 2: [[3,12],[-2,5],[-4,1]]
    # Expected output: 18
    points2 = [[3,12],[-2,5],[-4,1]]
    print(f"Example 2 (Prim's): {solution.minCostConnectPoints(points2)}")
    print(f"Example 2 (Kruskal's): {solution.minCostConnectPoints_kruskal(points2)}")
    print(f"Example 2 (Optimized): {solution.minCostConnectPoints_optimized(points2)}")
