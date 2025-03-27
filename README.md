# GridPathOptimizer: Efficient Pathfinding with A* and Common Path Selection  

## Overview  
**GridPathOptimizer** is an intelligent **pathfinding algorithm** that efficiently finds **shortest paths** in a 2D coordinate space while maximizing **common path utilization**. A **power-aware** and **obstacle-aware** pathfinding algorithm designed for **VLSI routing**, specifically for **data and clock signal routing**. The algorithm minimizes **power domain crossings**, optimizes **shortest paths**, and intelligently utilizes **divergent paths** to ensure efficient signal propagation.

## Features  
✅ **Grid-Based Movement**: Horizontal & vertical traversal with a configurable grid step size.  
✅ **Optimized Path Selection**: Prioritizes common paths to minimize redundancy.  
✅ **A(*) Algorithm for Pathfinding**: Uses an optimized **A-star (A*) search** for shortest path computation.  
✅ **Divergence Point Calculation**: Identifies the best point of deviation for new paths.  
✅ **Shortest Path Fallback**: If the divergence-based path is significantly longer, it switches to a direct shortest path.  
✅ **Obstacle Handling**: Supports polygonal obstacles to avoid restricted areas.  
✅ **VLSI-Aware Routing** – Optimized for **clock tree synthesis (CTS)** and **data signal routing** in **chip design**.  
✅ **Power Domain Awareness** – Ensures paths remain within the same power domain as much as possible, minimizing crossings.  
✅ **Configurable Parameters**: User-controlled grid size, divergence threshold, and step size.  
✅ **Visualization with Matplotlib**: Generates color-coded plots highlighting paths.  

## 🚀 Applications in VLSI & Physical Design

### 1️⃣ **Clock & Data Signal Routing**
- In **VLSI physical design**, **clock distribution networks** need optimized routing to reduce skew and power consumption.
- This algorithm can **optimize clock tree synthesis (CTS)** by **reusing existing paths** instead of computing independent shortest paths for each signal.  
- It can also **improve bus routing** in **SoC interconnect designs**.

### 2️⃣ **Multi-Net Optimization in PCB & Chip Layouts**
- **PCB trace routing** and **IC interconnect optimization** can use this algorithm to **reduce metal layer usage**.
- Can be used for **multi-net signal routing** in **MCM (Multi-Chip Modules)** and **3D-IC** designs.

### 3️⃣ **Routing in Advanced VLSI Nodes**
- In **sub-10nm process nodes**, reducing **wire congestion** is crucial for **timing closure**.
- Our approach **minimizes wirelength** and **reuses common paths**, helping in **power, performance, and area (PPA) optimization**.

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

## 🎯 How Power Aware Routing Algorithm Works  
1️⃣ The grid is divided into **different power domains** (Pink, Green, Blue).  
2️⃣ User-defined **obstacles** are placed in the grid to simulate real-world constraints.  
3️⃣ The algorithm finds the **shortest path** while **minimizing power domain crossings**.  
4️⃣ If a direct path is **blocked**, it uses **divergent routing** from existing paths.  
5️⃣ Each grid step has a **base cost**, and **power domain crossings** have a **high penalty (1000 units)**.  
6️⃣ The final path is **selected based on total cost** (shortest and least domain crossings).  

## How to Run  

### 1️⃣ Clone the Repository  
```sh
git clone https://github.com/yourusername/GridPathOptimizer.git
cd GridPathOptimizer
```

### 2️⃣ Run the code
```
python pathfinder.py
python pathfinder_power_aware.py
```

## Output Visualization

Below is the generated path visualization from the algorithm:

![Path Visualization](output.png?)
![Power Aware Routing Path Visualization](power_aware_routing.png?)
