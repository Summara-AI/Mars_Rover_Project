import heapq

class GridMap:
    def __init__(self, width=10, height=10):
        self.width = width
        self.height = height
        self.obstacles = set()

    def add_obstacle(self, x, y):
        self.obstacles.add((x, y))

    def display(self):
        for y in range(self.height - 1, -1, -1):
            row = ""
            for x in range(self.width):
                if (x, y) in self.obstacles:
                    row += " X "
                else:
                    row += " . "
            print(row)

    def neighbors(self, node):
        x, y = node
        directions = [(1,0), (-1,0), (0,1), (0,-1)]
        result = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if (nx, ny) not in self.obstacles:
                    result.append((nx, ny))
        return result

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        # Reconstruct path
        path = []
        curr = goal
        while curr:
            path.append(curr)
            curr = came_from.get(curr)
        path.reverse()
        return path
