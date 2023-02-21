# Naming Conventions for Path Planners

## Folder and File naming
- Package name: Just the algorithm name attached with `_global_planner` or `_local_planner` (f.e. prm_global_planner, rrt_star_global_planner) in lower case
- Folder name: See `package name`
- Header- and cpp-file: For example: `prm_planner_ros` and `prm_planner`. Algorithm name with planner and ROS or non-ROS implementation.

## Class naming
(Ideally the ROS-implementation is separated from the algorithm implementation itself. The algorithm itself doesn't use any ROS functions and can be used anywhere. The ROS-implementation implements the interface for a global or local planner and uses the function of the algorithm.)
- Namespace: `global_planner`
- ROS-implementation class name: For example `PRMPlannerROS` (Insert your algorithm name for `PRM` and add `PlannerROS`)
- Non-ROS-implementation class name: `PRMPlanner` (Insert your algorithm name for `PRM` and add `Planner`)

## CMakeList naming
Use package_name and name it explicitly here.
```
catkin_package(
  [...]
  LIBRARIES prm_global_planner
  [...]
)
```

## Plugin naming
Plugin definition:
- Library name: `lib/libpackage_name` (Insert package name for `package_name`)
- class name: `global_planner/PRMPlannerROS`
- type name: `global_planner::PRMPlannerROS`
