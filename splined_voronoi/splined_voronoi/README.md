# Splined Voronoi Planner

This Package contains a global path planner based on voronoi-diagrams which smoothens the path. 

## Installation

Need tinyspline and nlopt built from source

```bash
cd catkin_ws/src
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

git clone https://github.com/msteinbeck/tinyspline.git
cd tinyspline
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=True ..
cmake --build .
sudo cmake --build . --target install
```
