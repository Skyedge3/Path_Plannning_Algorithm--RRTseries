# RRT Series Path Planning Algorithms

## Introduction

This repository implements a series of path planning algorithms based on RRT (Rapidly-exploring Random Tree) for robot navigation and motion planning. The code supports path planning in both 2D and 3D environments, and integrates various optimization strategies such as RRT, RRT*, QRRT*, PSORRT*, etc., along with Bezier curve smoothing functionality.

## Algorithm Introduction

Implemented algorithms include:

- **RRT**: Random sampling-based rapidly exploring tree algorithm
- **RRT***: RRT with path optimization, generating asymptotically optimal paths
- **QRRT***: Quick-RRT* algorithm with ancestor node depth search capability
- **PSORRT***: RRT* algorithm combined with Particle Swarm Optimization (PSO) to improve sampling efficiency
- **C_QRRT***: QRRT* algorithm with angle constraints to limit path deflection
- **C_PQRRT***: PSOQRRT* algorithm with angle constraints, combining PSO to optimize the random sampling process

- **MORE**: More RRT-based algorithms being continuously updated...

## Dependencies

- MATLAB (Recommended version: R2019b or higher)
- MATLAB Robotics System Toolbox
- MATLAB Navigation Toolbox

## Code Structure

```
.
├── lib/                       # Algorithm implementation library
│   ├── bezierCurveSmoothing.m # Bezier curve smoothing
│   ├── checkIfGoalIsReached.m # Goal arrival detection
│   ├── planC_QRRTStar.m       # QRRT* algorithm with angle constraints
│   ├── planCP_QRRTStar.m      # CP_QRRT* algorithm
│   ├── planPSOQRRTStar.m      # PSOQRRT* algorithm
│   ├── planPSORRTStar.m       # RRT* algorithm with PSO
│   ├── planQRRTStar.m         # QRRT* algorithm implementation
│   ├── planRRT.m              # RRT algorithm
│   ├── planRRTStar.m          # RRT* algorithm
│   └── sphere3D.m             # 3D distance calculation function
├── workspace/                 # Map data
│   ├── map.mat                # Various test map data
│   ├── obtainMAP.m            # Map generation tool, converts PNG to .mat format
│   └── *.png                  # Map image files
├── main.m                     # 2D path planning main program
├── main3d.m                   # 3D path planning main program
```

## Usage Instructions

### 2D Path Planning

1. Open MATLAB and navigate to the project root directory
2. Run the `main.m` script for path planning in 2D environments:

```matlab
>> main
```

### 3D Path Planning

1. Open MATLAB and navigate to the project root directory
2. Run the `main3d.m` script for path planning in 3D environments:

```matlab
>> main3d
```

## Parameter Configuration

Main configuration parameters are located in the `main.m` and `main3d.m` files, including:

- `planner.MaxConnectionDistance`: Maximum connection distance between nodes
- `planner.MaxIterations`: Maximum number of iterations
- `planner.ContinueAfterGoalReached`: Whether to continue planning after reaching the goal
- `planner.MaxTime`: Maximum time limit for planning (seconds)
- `planner.GoalBias`: Probability of bias toward the goal point

For QRRT* algorithms:
- `planner.ParentDepthSearch`: Parent node depth search level
- `planner.ParentDepthRewire`: Parent node rewiring depth level
- `planner.MaxDeflectionAngle`: Maximum allowed deflection angle (radians)

For PSO-related algorithms:
- `planner.InertiaWeight`: PSO inertia weight
- `planner.PopulationSize`: Particle swarm size
- `planner.Iterations`: Number of PSO iterations

## Map Configuration

The `workspace` directory contains various test maps. To switch maps, modify the map loading section in `main.m` or `main3d.m`:

```matlab
% In main.m
load workspace\mymap.mat  % Can be replaced with other maps

% In main3d.m, modify the obstacle definition section
```

## Path Smoothing

This project provides Bezier curve smoothing functionality, which can be enabled by uncommenting the relevant code in `main.m`:

```matlab
bezierOutput = bezierCurveSmoothing(pthObj);

f2 = figure;
f1.Position = [400 200 600 500];
hold on
show(sv.Map)
plot(bezierOutput.bx, bezierOutput.by, 'r-', 'LineWidth', 2)
title("Path After Smoothing")
hold off
```

## Functionality Demonstration

After running the program, the following will be displayed:
1. Planning environment and obstacles
2. Exploration tree structure
3. Final planned path
4. Path planning time and path length
5. Maximum path deflection angle
