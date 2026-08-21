import math
from collections import deque
import heapq

class SearchAgent:
    def __init__(self):
        self.plan = []
        # LAB 04: Changed to A* Search
        self.active_algo = 'AStar' 

    def get_neighbors(self, state, grid_size, walls):
        x, y = state
        w, h = grid_size
        neighbors = []
        directions = {"Up": (x, y + 1), "Down": (x, y - 1), "Left": (x - 1, y), "Right": (x + 1, y)}
        for action, (nx, ny) in directions.items():
            if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in walls:
                neighbors.append((action, (nx, ny), 1))
        return neighbors

    # --- LAB 04: Heuristic Functions ---
    def manhattan_distance(self, pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def euclidean_distance(self, pos, goal):
        return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

    # --- LAB 04: A* Search ---
    def astar_search(self, start, goal, grid_size, walls, heuristic_type='manhattan'):
        frontier = []
        # Calculate initial h(n)
        if heuristic_type == 'manhattan':
            h_cost = self.manhattan_distance(start, goal)
        else:
            h_cost = self.euclidean_distance(start, goal)
            
        # Priority Queue stores: (f_cost, g_cost, current_state, path)
        heapq.heappush(frontier, (h_cost, 0, start, []))
        reached_states = set()

        while frontier:
            f_cost, g_cost, current_state, path = heapq.heappop(frontier)

            if current_state == goal:
                return path

            if current_state not in reached_states:
                reached_states.add(current_state)
                
                for action, next_state, step_cost in self.get_neighbors(current_state, grid_size, walls):
                    if next_state not in reached_states:
                        new_g = g_cost + step_cost
                        if heuristic_type == 'manhattan':
                            new_h = self.manhattan_distance(next_state, goal)
                        else:
                            new_h = self.euclidean_distance(next_state, goal)
                        
                        # f(n) = g(n) + h(n)
                        new_f = new_g + new_h
                        heapq.heappush(frontier, (new_f, new_g, next_state, path + [action]))
        return []

    # Keeping BFS, DFS, UCS from Lab 03 for comparison (Omitted for brevity, paste your Lab 3 methods here)
    def bfs_search(self, start, goal, grid_size, walls):
        # ... (Your existing Lab 3 BFS code) ...
        pass
        
    def dfs_search(self, start, goal, grid_size, walls):
        # ... (Your existing Lab 3 DFS code) ...
        pass
        
    def ucs_search(self, start, goal, grid_size, walls):
        # ... (Your existing Lab 3 UCS code) ...
        pass

    def sense_and_act(self, percept):
        if not self.plan:
            all_food = percept['all_food']
            if not all_food:
                return "Stay" 

            start_state = percept['agent_pos']
            walls = set(percept['walls'])
            grid_size = percept['grid_size']

            # Find closest food using Manhattan distance as the heuristic
            closest_food = min(
                all_food, 
                key=lambda f: self.manhattan_distance(start_state, f)
            )

            # Execution Logic
            if self.active_algo == 'BFS':
                self.plan = self.bfs_search(start_state, closest_food, grid_size, walls)
            elif self.active_algo == 'DFS':
                self.plan = self.dfs_search(start_state, closest_food, grid_size, walls)
            elif self.active_algo == 'UCS':
                self.plan = self.ucs_search(start_state, closest_food, grid_size, walls)
            elif self.active_algo == 'AStar':
                self.plan = self.astar_search(start_state, closest_food, grid_size, walls, 'manhattan')

        if self.plan:
            return self.plan.pop(0)
        else:
            return "Stay"