# -*- coding: utf-8 -*-
"""
Created on Thu Mar 27 10:23:11 2025

@author: lalit
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
from shapely.geometry import Polygon, Point

# Grid settings
GRID_STEP = 100  # Grid resolution
BASE_COST = 1  # Cost per grid step
POWER_DOMAIN_CROSSING_COST = 1000  # High cost for power domain crossing
DIVERGENCE_THRESHOLD = 0.5  # Threshold for divergence vs. shortest path

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

def astar(start, end, grid_size, obstacles, power_grid):
    """A* shortest path algorithm with power domain cost handling."""
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
            base_cost = BASE_COST  # Base movement cost
            crossing_cost = POWER_DOMAIN_CROSSING_COST if power_grid[current] != power_grid[neighbor] else 0
            tentative_g_score = g_score[current] + base_cost + crossing_cost
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found

def find_divergence_point(existing_path, new_end):
    """Finds the best divergence point based on the closest distance to new_end."""
    return min(existing_path, key=lambda point: heuristic(point, new_end))

def find_best_path(start, end, grid_size, obstacles, existing_paths, power_grid):
    """Finds the best path by comparing shortest and divergence paths based on total cost."""
    if not existing_paths:
        print("Routing with shortest path due to missing existing paths.")
        return astar(start, end, grid_size, obstacles, power_grid)  # First path is purely shortest
    
    # Find the closest existing endpoint to new_end
    closest_existing_path = min(existing_paths, key=lambda path: heuristic(path[-1], end))
    
    # Find divergence point
    divergence_point = find_divergence_point(closest_existing_path, end)
    
    # Construct the divergence path
    path_to_divergence = closest_existing_path[:closest_existing_path.index(divergence_point) + 1]
    path_from_divergence = astar(divergence_point, end, grid_size, obstacles, power_grid)
    
    if path_from_divergence is None:
        print("Path from divergence is none..")
        return None
    
    divergence_path = path_to_divergence + path_from_divergence[1:]  # Merge paths

    # Compute cost for divergence path
    divergence_cost = sum(
        BASE_COST + (POWER_DOMAIN_CROSSING_COST if power_grid[divergence_path[i]] != power_grid[divergence_path[i + 1]] else 0)
        for i in range(len(divergence_path) - 1)
    )

    # Compute cost for shortest path
    shortest_path = astar(start, end, grid_size, obstacles, power_grid)
    if shortest_path is None:
        return divergence_path  # If no direct path, use divergence

    shortest_cost = sum(
        BASE_COST + (POWER_DOMAIN_CROSSING_COST if power_grid[shortest_path[i]] != power_grid[shortest_path[i + 1]] else 0)
        for i in range(len(shortest_path) - 1)
    )
    print("Divergence Path cost : "+str(divergence_cost))
    print("Shortest Path cost : "+str(shortest_cost))
    # Choose the path with the lower cost
    return shortest_path if shortest_cost < divergence_cost else divergence_path

def plot_paths(grid_size, start_end_pairs, obstacles, paths, power_grid):
    """Plots all paths, start & end points, and power domains."""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, grid_size[0])
    ax.set_ylim(0, grid_size[1])

    # Plot power domains
    for (x, y), domain in power_grid.items():
        color = "pink" if domain == 0 else "green" if domain == 1 else "blue"
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

# Power domains (User-defined polygons)
power_domains = {
    1: [(2000, 2000), (6000, 2000), (6000, 6000), (2000, 6000)],  # Green
    2: [(4000, 4000), (7000, 4000), (7000, 8000), (4000, 8000)],  # Blue
}

# Assign power domains to grid cells
power_grid = {}
for x in range(0, int(grid_size[0]), GRID_STEP):
    for y in range(0, int(grid_size[1]), GRID_STEP):
        point = Point(x, y)
        power_grid[(x, y)] = 0  # Default pink
        for domain, polygon in power_domains.items():
            if Polygon(polygon).contains(point):
                power_grid[(x, y)] = domain

# Define start and end points
start_end_pairs = [((500, 1000), (6500, 7500)), ((500, 1000), (6000, 6000))]
start_end_pairs = [
    ((500, 1000), (6500, 7500)),
    ((500, 1000), (2500, 5010)),
    ((500, 1000), (7000, 900)),
    ((500, 1000), (4000, 3000)),
    ((500, 1000), (3500, 3500)),  
    ((500,1000),  (6000, 6000))  # Newly added point
]

start_end_pairs = [(snap_to_grid(start), snap_to_grid(end)) for start, end in start_end_pairs]

# Compute paths
paths = []
for start, end in start_end_pairs:
    print(">> Routing : "+str(start) + " -> "+str(end))
    path = find_best_path(start, end, grid_size, obstacles, paths, power_grid)
    if path is not None:
        print("Routed successfully.")
    paths.append(path)
# Plot results
plot_paths(grid_size, start_end_pairs, obstacles, paths, power_grid)
