# Splined Voronoi Planner

This Package contains a global path planner for multi robot formations based on voronoi-diagrams and path smoothing.

## Installation

Need tinyspline and nlopt built from source which are contained as git submodules

```bash
cd path/to/match_path_planning
git submodule update --init --recursive
cd splined_voronoi
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

cd path/to/match_path_planning
cd splined_voronoi
cd tinyspline
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=True ..
cmake --build .
sudo cmake --build . --target install
```

## Overview

Implements global path planner splined_voronoi/SplinedVoronoiPlanner which can be used in move base and move base flex.

Performs:
- costmap inflation for multi robot formation
- voronoi generation via boost library
- path planning on voronoi diagram
- selection of waypoints from path
- smoothing of path with quintic Bézier-Splines
- optimization of Splines for collision avoidance and curvature constraint
- sampling of optimized curve and output
