import math
from collections import deque
import heapq
from logic_engine import KnowledgeBase

class SearchAgent:
    def __init__(self):
        self.plan = []
        self.active_algo = 'AStar' 
        
        self.kb = KnowledgeBase()
        self.kb.tell_rule(['TargetVisible', 'HasDust'], 'SafeToEngage')
        self.kb.tell_rule(['SafeToEngage', 'BloodseekerMissing'], 'Retreat') 

    def get_neighbors(self, state, grid_size, walls, toxic_traps):
        x, y = state
        w, h = grid_size
        neighbors = []
        directions = {
            "Up": (x, y + 1), "Down": (x, y - 1),
            "Left": (x - 1, y), "Right": (x + 1, y)
        }
        
        for action, (nx, ny) in directions.items():
            if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in walls:
                
                self.kb.clear_facts()
                self.kb.tell_fact('TargetVisible')
                self.kb.tell_fact('HasDust')
                
                if (nx, ny) in toxic_traps: 
                    self.kb.tell_fact('BloodseekerMissing') 

                self.kb.forward_chain()
                
                if 'Retreat' in self.kb.facts:
                    continue
                
                step_cost = 1
                neighbors.append((action, (nx, ny), step_cost))
                
        return neighbors

    def manhattan_distance(self, pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def euclidean_distance(self, pos, goal):
        return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

    def bfs_search(self, start, goal, grid_size, walls, toxic_traps):
        frontier = deque([(start, [])])
        reached = {start}

        while frontier:
            current_state, path = frontier.popleft()

            if current_state == goal:
                return path

            for action, next_state, _ in self.get_neighbors(current_state, grid_size, walls, toxic_traps):
                if next_state not in reached:
                    reached.add(next_state)
                    frontier.append((next_state, path + [action]))
        return []

    def dfs_search(self, start, goal, grid_size, walls, toxic_traps):
        frontier = [(start, [])]
        reached = set()

        while frontier:
            current_state, path = frontier.pop()

            if current_state == goal:
                return path

            if current_state not in reached:
                reached.add(current_state)
                for action, next_state, _ in self.get_neighbors(current_state, grid_size, walls, toxic_traps):
                    if next_state not in reached:
                        frontier.append((next_state, path + [action]))
        return []

    def ucs_search(self, start, goal, grid_size, walls, toxic_traps):
        frontier = []
        heapq.heappush(frontier, (0, start, []))
        reached = {} 

        while frontier:
            cost, current_state, path = heapq.heappop(frontier)

            if current_state == goal:
                return path

            if current_state not in reached or cost < reached[current_state]:
                reached[current_state] = cost
                
                for action, next_state, action_cost in self.get_neighbors(current_state, grid_size, walls, toxic_traps):
                    new_cost = cost + action_cost
                    if next_state not in reached or new_cost < reached.get(next_state, float('inf')):
                        heapq.heappush(frontier, (new_cost, next_state, path + [action]))
        return []

    def astar_search(self, start, goal, grid_size, walls, toxic_traps, heuristic_type='manhattan'):
        frontier = []
        if heuristic_type == 'manhattan':
            h_cost = self.manhattan_distance(start, goal)
        else:
            h_cost = self.euclidean_distance(start, goal)
            
        heapq.heappush(frontier, (h_cost, 0, start, []))
        cost_so_far = {start: 0}

        while frontier:
            f_cost, g_cost, current_state, path = heapq.heappop(frontier)

            if current_state == goal:
                return path

            if g_cost > cost_so_far.get(current_state, float('inf')):
                continue

            for action, next_state, step_cost in self.get_neighbors(current_state, grid_size, walls, toxic_traps):
                new_g = g_cost + step_cost
                if new_g < cost_so_far.get(next_state, float('inf')):
                    cost_so_far[next_state] = new_g
                    if heuristic_type == 'manhattan':
                        new_h = self.manhattan_distance(next_state, goal)
                    else:
                        new_h = self.euclidean_distance(next_state, goal)
                    new_f = new_g + new_h
                    heapq.heappush(frontier, (new_f, new_g, next_state, path + [action]))
        return []

    def sense_and_act(self, percept):
        if not self.plan:
            all_food = percept['all_food']
            if not all_food:
                return "Stay" 

            start_state = percept['agent_pos']
            walls = set(percept['walls'])
            grid_size = percept['grid_size']
            toxic_traps = set(percept.get('toxic_traps', []))

            closest_food = min(
                all_food, 
                key=lambda f: self.manhattan_distance(start_state, f)
            )

            if self.active_algo == 'BFS':
                self.plan = self.bfs_search(start_state, closest_food, grid_size, walls, toxic_traps)
            elif self.active_algo == 'DFS':
                self.plan = self.dfs_search(start_state, closest_food, grid_size, walls, toxic_traps)
            elif self.active_algo == 'UCS':
                self.plan = self.ucs_search(start_state, closest_food, grid_size, walls, toxic_traps)
            elif self.active_algo == 'AStar':
                self.plan = self.astar_search(start_state, closest_food, grid_size, walls, toxic_traps, 'manhattan')

        if self.plan:
            return self.plan.pop(0)
        else:
            return "Stay"