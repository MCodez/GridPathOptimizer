import numpy as np
import matplotlib.pyplot as plt
import heapq
from shapely.geometry import Polygon, Point

# Grid settings
GRID_STEP = 100  # Grid resolution
DIVERGENCE_THRESHOLD = 0.5  # 50% threshold for shortest path vs. divergence path

def snap_to_grid(point):
    """Snap coordinates to the nearest grid point."""
    x, y = point
    return (round(x / GRID_STEP) * GRID_STEP, round(y / GRID_STEP) * GRID_STEP)

def heuristic(a, b):
    """Manhattan distance heuristic for A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(node, grid_size, obstacles):
    """Returns valid neighbors considering grid and obstacles."""
    x, y = node
    neighbors = [(x + GRID_STEP, y), (x - GRID_STEP, y), (x, y + GRID_STEP), (x, y - GRID_STEP)]
    
    valid_neighbors = []
    for nx, ny in neighbors:
        if 0 <= nx <= grid_size[0] and 0 <= ny <= grid_size[1]:
            if not any(Polygon(ob).contains(Point(nx, ny)) for ob in obstacles):  # Obstacle check
                valid_neighbors.append((nx, ny))
    return valid_neighbors

def astar(start, end, grid_size, obstacles):
    """A* shortest path algorithm."""
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
            return path[::-1]  # Reverse path
        
        for neighbor in get_neighbors(current, grid_size, obstacles):
            tentative_g_score = g_score[current] + GRID_STEP
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found

def find_divergence_point(existing_path, new_end):
    """Finds the best divergence point based on the closest distance to new_end."""
    return min(existing_path, key=lambda point: heuristic(point, new_end))

def find_best_path(start, end, grid_size, obstacles, existing_paths):
    """Finds the best path, either using an existing path or a new shortest path."""
    if not existing_paths:
        return astar(start, end, grid_size, obstacles)  # First path is a pure shortest path
    
    # Find the closest existing endpoint to new_end
    closest_existing_path = min(existing_paths, key=lambda path: heuristic(path[-1], end))
    
    # Find the divergence point
    divergence_point = find_divergence_point(closest_existing_path, end)
    
    # Construct the path from start → divergence_point → end
    path_to_divergence = closest_existing_path[:closest_existing_path.index(divergence_point) + 1]
    path_from_divergence = astar(divergence_point, end, grid_size, obstacles)
    
    if path_from_divergence is None:
        return None
    
    divergence_path = path_to_divergence + path_from_divergence[1:]  # Merge paths
    
    # Compare with fresh shortest path
    shortest_new_path = astar(start, end, grid_size, obstacles)
    
    if shortest_new_path and len(shortest_new_path) < DIVERGENCE_THRESHOLD * len(divergence_path):
        return shortest_new_path  # Use shortest if it's significantly better
    return divergence_path  # Otherwise, prefer divergence-based path

def plot_paths(grid_size, start_end_pairs, obstacles, paths):
    """Plots all paths, start & end points."""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, grid_size[0])
    ax.set_ylim(0, grid_size[1])

    # Plot obstacles
    for ob in obstacles:
        polygon = Polygon(ob)
        x, y = polygon.exterior.xy
        ax.fill(x, y, color='gray', alpha=0.5)

    # Plot paths
    for path in paths:
        if path:
            px, py = zip(*path)
            ax.plot(px, py, marker='o', color='blue', linewidth=2, markersize=3)  # Consistent path color
            
            # Add arrows for direction
            for i in range(len(path) - 1):
                ax.annotate('', xy=path[i+1], xytext=path[i],
                            arrowprops=dict(arrowstyle='->', color='blue', lw=1.5))

    # Plot start & end points
    for start, end in start_end_pairs:
        ax.plot(*start, marker='o', color='green', markersize=8)  # Start points
        ax.plot(*end, marker='o', color='red', markersize=8)  # End points

    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.show()

# Example Input
grid_size = (7852.8, 9423.36)
obstacles = []  # No obstacles for now

start_end_pairs = [
    ((500, 1000), (6500, 7500)),
    ((500, 1000), (7000, 900)),
    ((500, 1000), (4000, 3000)),
    ((500, 1000), (2500, 5010)),
    ((500, 1000), (3500, 3500))  # Newly added point
]

# Snap all points to the grid
start_end_pairs = [(snap_to_grid(start), snap_to_grid(end)) for start, end in start_end_pairs]

# Compute paths
existing_paths = []
paths = []
for start, end in start_end_pairs:
    path = find_best_path(start, end, grid_size, obstacles, existing_paths)
    paths.append(path)
    if path:
        existing_paths.append(path)

# Plot the paths
plot_paths(grid_size, start_end_pairs, obstacles, paths)
