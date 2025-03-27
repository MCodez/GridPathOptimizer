import numpy as np
import matplotlib.pyplot as plt
import heapq
from shapely.geometry import Polygon, Point

# Grid settings
GRID_STEP = 100  # Grid resolution
POWER_DOMAIN_CROSS_COST = 1000  # Cost for crossing power domains
BASE_COST = 1  # Cost per grid step
DIVERGENCE_THRESHOLD = 0.5 # Threshold for divergence vs shortest path

# Function to snap points to the nearest grid
def snap_to_grid(point):
    x, y = point
    return (round(x / GRID_STEP) * GRID_STEP, round(y / GRID_STEP) * GRID_STEP)

# Manhattan heuristic function for A* search
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Function to assign power domains based on user-defined polygons
def assign_power_domains(grid_size, power_domain_shapes):
    power_grid = {}
    default_domain = "Pink"

    for x in range(0, grid_size[0] + GRID_STEP, GRID_STEP):
        for y in range(0, grid_size[1] + GRID_STEP, GRID_STEP):
            point = Point(x, y)
            power_grid[(x, y)] = default_domain  # Default to Pink

            for domain_name, polygon in power_domain_shapes.items():
                if polygon.contains(point):
                    power_grid[(x, y)] = domain_name
                    break  # Stop checking if it belongs to a domain

    return power_grid

# Function to get valid neighbors considering grid, obstacles, and power domains
def get_neighbors(node, grid_size, obstacles):
    x, y = node
    neighbors = [(x + GRID_STEP, y), (x - GRID_STEP, y), (x, y + GRID_STEP), (x, y - GRID_STEP)]
    
    valid_neighbors = []
    for nx, ny in neighbors:
        if 0 <= nx <= grid_size[0] and 0 <= ny <= grid_size[1]:
            if not any(Polygon(ob).contains(Point(nx, ny)) for ob in obstacles):  # Obstacle check
                valid_neighbors.append((nx, ny))
    return valid_neighbors

# A* pathfinding algorithm with power domain awareness
def astar(start, end, grid_size, obstacles, power_grid):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in get_neighbors(current, grid_size, obstacles):
            step_cost = BASE_COST

            # Check if power domain crossing happens
            if power_grid[current] != power_grid[neighbor]:
                step_cost += POWER_DOMAIN_CROSS_COST

            tentative_g_score = g_score[current] + step_cost

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Function to find the best divergence point
def find_divergence_point(existing_path, new_end):
    """Finds the best divergence point based on closest distance to new_end."""
    return min(existing_path, key=lambda point: heuristic(point, new_end))

# Function to compute total path cost
def compute_path_cost(path, power_grid):
    if not path:
        return float('inf')

    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += BASE_COST
        if power_grid[path[i]] != power_grid[path[i + 1]]:
            total_cost += POWER_DOMAIN_CROSS_COST
    return total_cost

# Function to find the best path considering divergence and cost
def find_best_path(start, end, grid_size, obstacles, power_grid, existing_paths):
    """Finds the best path, considering power domain constraints and divergence paths."""

    if not existing_paths:
        return astar(start, end, grid_size, obstacles, power_grid)  # First path is always shortest path

    # Find the closest existing path
    closest_existing_path = min(existing_paths, key=lambda path: heuristic(path[-1], end)) if existing_paths else None

    if closest_existing_path is None:
        return astar(start, end, grid_size, obstacles, power_grid)  # Fallback to shortest path

    # Find the divergence point
    divergence_point = find_divergence_point(closest_existing_path, end)

    # Construct path using divergence point
    path_to_divergence = closest_existing_path[:closest_existing_path.index(divergence_point) + 1]
    path_from_divergence = astar(divergence_point, end, grid_size, obstacles, power_grid)

    if path_from_divergence is None:
        return astar(start, end, grid_size, obstacles, power_grid)  # Fallback to shortest path

    divergence_path = path_to_divergence + path_from_divergence[1:]

    # Compute cost for both paths
    divergence_cost = sum(BASE_COST + (POWER_DOMAIN_CROSS_COST if power_grid[divergence_path[i]] != power_grid[divergence_path[i+1]] else 0) for i in range(len(divergence_path)-1))
    shortest_path = astar(start, end, grid_size, obstacles, power_grid)
    if shortest_path is None:
        return divergence_path
    
    shortest_cost = sum(BASE_COST + (POWER_DOMAIN_CROSS_COST if power_grid[shortest_path[i]] != power_grid[shortest_path[i+1]] else 0) for i in range(len(shortest_path)-1))


    return shortest_path if shortest_cost < (DIVERGENCE_THRESHOLD * divergence_cost) else divergence_path

# Function to plot paths
def plot_paths(grid_size, start_end_pairs, obstacles, power_grid, paths):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, grid_size[0])
    ax.set_ylim(0, grid_size[1])

    # Plot power domain background
    for (x, y), domain in power_grid.items():
        color = {'Pink': 'lightpink', 'Green': 'lightgreen', 'Blue': 'lightblue'}.get(domain, 'white')
        ax.add_patch(plt.Rectangle((x, y), GRID_STEP, GRID_STEP, color=color, alpha=0.3))

    # Plot obstacles
    for ob in obstacles:
        polygon = Polygon(ob)
        x, y = polygon.exterior.xy
        ax.fill(x, y, color='gray', alpha=0.5)

    # Plot paths
    for path in paths:
        if path:
            px, py = zip(*path)
            ax.plot(px, py, marker='o', color='blue', linewidth=2, markersize=3)

    # Plot start & end points
    for start, end in start_end_pairs:
        ax.plot(*start, marker='o', color='green', markersize=8)
        ax.plot(*end, marker='o', color='red', markersize=8)

    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.show()

# Example Input
grid_size = (8000, 8000)
obstacles = [ [(3000, 3000), (4000, 3000), (4000, 4000), (3000, 4000)] ]  # Example obstacle

# Define power domain regions
power_domain_shapes = {
    "Green": Polygon([(1000, 1000), (2000, 1000), (2000, 5000), (1000, 5000)]),
    "Blue": Polygon([(5000, 5000), (7000, 5000), (7000, 7000), (5000, 7000)])
}

power_grid = assign_power_domains(grid_size, power_domain_shapes)

start_end_pairs = [ ((500, 2000), (300, 5700)),((500, 2000), (2200, 6500)),((500, 2000), (1300, 3200)),((500, 2000), (900, 2000)),((500, 2000), (6200, 7500)),((500, 2000), (1800, 4000)),((500, 2000), (6000, 6000)),((500, 2000), (5000, 4000)), ((500, 2000), (7500, 2700)) ]
start_end_pairs = [(snap_to_grid(start), snap_to_grid(end)) for start, end in start_end_pairs]

existing_paths = []
paths = []
for start, end in start_end_pairs:
    path = find_best_path(start, end, grid_size, obstacles, power_grid, existing_paths)
    paths.append(path)
    if path:
        existing_paths.append(path)

plot_paths(grid_size, start_end_pairs, obstacles, power_grid, paths)
