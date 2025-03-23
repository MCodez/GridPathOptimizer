# GridPathOptimizer: Efficient Pathfinding with A* and Common Path Selection  

## Overview  
**GridPathOptimizer** is an intelligent **pathfinding algorithm** that efficiently finds **shortest paths** in a 2D coordinate space while maximizing **common path utilization**. Unlike traditional **A* algorithms**, this approach ensures that new paths reuse existing paths as much as possible, reducing unnecessary deviations and optimizing routing efficiency.  

## Features  
✅ **Grid-Based Movement**: Horizontal & vertical traversal with a configurable grid step size.  
✅ **Optimized Path Selection**: Prioritizes common paths to minimize redundancy.  
✅ **A(*) Algorithm for Pathfinding**: Uses an optimized **A-star (A*) search** for shortest path computation.  
✅ **Divergence Point Calculation**: Identifies the best point of deviation for new paths.  
✅ **Shortest Path Fallback**: If the divergence-based path is significantly longer, it switches to a direct shortest path.  
✅ **Obstacle Handling**: Supports polygonal obstacles to avoid restricted areas.  
✅ **Configurable Parameters**: User-controlled grid size, divergence threshold, and step size.  
✅ **Visualization with Matplotlib**: Generates color-coded plots highlighting paths.  

## Algorithm Workflow  
1. **First Path Calculation**  
   - Computes the shortest path using **A* (A-star) search algorithm** while avoiding obstacles.  
2. **New Path Addition**  
   - Finds the closest previously computed path to the new endpoint.  
   - Identifies the **best divergence point** by minimizing distance to the new endpoint.  
   - Constructs the new path using the common segment and diverging only when necessary.  
   - If the newly computed shortest path is significantly shorter than the divergence-based path, it is selected instead.  
3. **Visualization**  
   - Plots all paths in a uniform color with **directional arrows**.  
   - Highlights start and end points distinctly.  

## How to Run  

### 1️⃣ Clone the Repository  
```sh
git clone https://github.com/yourusername/GridPathOptimizer.git
cd GridPathOptimizer

python pathfinder.py
```

## Output Visualization

Below is the generated path visualization from the algorithm:


