#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/SetBool.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <string>
#include <memory>  // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <cmath>
#include <Eigen/Dense>

#include <relaxed_a_star/path_planner_types.h>
#include <relaxed_a_star/ras_param_manager.h>

namespace global_planner
{
    class RelaxedAStar : public nav_core::BaseGlobalPlanner
    {
    public:
        /**
         * @brief Default constructor of the Relaxed A Star class
         *
         */
        RelaxedAStar();

        /**
         * @brief Construct a new Relaxed A Star object
         *
         * @param name The name of this planner
         * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        RelaxedAStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief Initialization method for the CostmapPlanner
         *
         * @param name The name of this planner
         * @param costmap A pointer to the costmap that will be used for planning
         * @param global_frame Global frame of the costmap
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
         *        in x and y before failing
         * @param plan The plan... filled by the planner
         * @param cost The cost for the the plan
         * @param message Optional more detailed outcome as a string
         * @return Result code as described on GetPath action result:
         *         SUCCESS         = 0
         *         1..9 are reserved as plugin specific non-error results
         *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
         *         CANCELED        = 51
         *         INVALID_START   = 52
         *         INVALID_GOAL    = 53
         *         NO_PATH_FOUND   = 54
         *         PAT_EXCEEDED    = 55
         *         EMPTY_PATH      = 56
         *         TF_ERROR        = 57
         *         NOT_INITIALIZED = 58
         *         INVALID_PLUGIN  = 59
         *         INTERNAL_ERROR  = 60
         *         71..99 are reserved as plugin specific errors
         */
        uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                        std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message);

        /**
         * @brief Requests the planner to cancel, e.g. if it takes too much time.
         * @remark New on MBF API
         * @return True if a cancel has been successfully requested, false if not implemented.
         */
        bool cancel();

    protected:
        // Defines if the path planner was initialized. If not initialized the planner can not find path
        bool initialized_;

        // ROS wrapper for the 2DCostmap object. Probably unnessecary to also store this next to the 2DCostmap object
        costmap_2d::Costmap2DROS* costmap_ros_;
        // This object is a pointer to the 2D costmap
        costmap_2d::Costmap2D* costmap_;
        // One dimensional represantation of the map. True = occupied, false = free
        std::shared_ptr<bool[]> occupancy_map_;

        // Publisher for the global plan the robot has to follow
        ros::Publisher plan_publisher_;

    private:
        /**
         * @brief This methods help splitting up the "makePlan" method
         * This method searches from a start point to a goal point and filles the g_score array
         *
         * @param array_start_cell
         * @param array_goal_cell
         * @param g_score
         */
        void findPlan(int array_start_cell, int array_goal_cell, std::shared_ptr<float[]> g_score);
        /**
         * @brief This methods help splitting up the "makePlan" method
         * This method searches for the shortest path in the g_score array
         *
         * @param array_start_cell Index of the start cell in the 1D-array representation (start location of the mobile
         * robot)
         * @param array_goal_cell Index of the goal cell in the 1D-array representation (goal location of the mobile robot)
         * @param g_score G_score array which was filled by the findPlan method
         * @return std::vector<int> List of indexes in the 1D-array representation to define the cells the robot has to
         * travel through
         */
        std::vector<int> createPlan(int array_start_cell, int array_goal_cell, std::shared_ptr<float[]> g_score);

        void createPoseArrayForPlan(std::vector<int> array_plan, std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Gets all free neighbor cells adjacent to the current cell
         *
         * @param array_current_cell Index of the current cell in the 1D-array from where the neighbor cell should be
         * calculated
         * @return std::vector<int> List with all indexes in the 1D-array that are the neighbors of the input cell
         */
        std::vector<int> getFreeNeighborCells(int array_current_cell);

        /**
         * @brief Returns the index of the cell next to the current cell with the lowest g_score
         *
         * @param array_current_cell Index of the current cell in the 1D-representation
         * @param g_score G_Score array with all values
         * @return int Index of the cell with the lowest g_score in the 1D-representation
         */
        int getMinGScoreNeighborCell(int array_current_cell, std::shared_ptr<float[]> g_score);

        /**
         * @brief Helper method for easy creation of the 1D array that defines which spot in the map is occupied.
         *
         */
        void createOccupancyMap();

        /**
         * @brief Method for publishing the plan that the robot has to follow
         *
         * @param plan List with all Poses that belong to the global plan
         */
        void publishPlan(std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Calculates the movement between two cells. Usally used to calculate between a cell and a neighbor cell.
         *
         * @param array_current_cell Index of the current cell in the 1D-representation
         * @param array_target_cell Index of the target cell the movement is supposed to end in the 1D-representation
         * @return float Movement cost to move from current to target cell
         */
        float calcMoveCost(int array_current_cell, int array_target_cell);
        /**
         * @brief Calculates the movement between two cells. Usally used to calculate between a cell and a neighbor cell.
         *
         * @param map_current_cell Point of the current cell in the 2D-representation of the costmap
         * @param map_target_cell Point of the target cell the movement is supposed to end in the 2D-representation of the
         * costmap
         * @return float Movement cost to move from current to target cell
         */
        float calcMoveCost(int* map_current_cell, int* map_target_cell);
        /**
         * @brief Calculates the movement between two cells. Usally used to calculate between a cell and a neighbor cell.
         *
         * @param map_current_cell_x x coordinate of the current cell in the 2D-representation of the costmap
         * @param map_current_cell_y y coordinate of the current cell in the 2D-representation of the costmap
         * @param map_target_cell_x x coordinate of the target cell the movement is supposed to end in the 2D-representation
         * of the costmap
         * @param map_target_cell_y y coordinate of the target cell the movement is supposed to end in the 2D-representation
         * of the costmap
         * @return float Movement cost to move from current to target cell
         */
        float calcMoveCost(int map_current_cell_x, int map_current_cell_y, int map_target_cell_x, int map_target_cell_y);

        /**
         * @brief Method for calulcating the g score for the target cell.
         *
         * @param current_cell_g_cost g score for the current cell
         * @param array_current_cell Index for the current cell in the one dimensional representation of the costmap
         * @param array_target_cell Index for the target cell in the one dimensional representation of the costmap
         * @return float
         */
        float calcGCost(float current_cell_g_cost, int array_current_cell, int array_target_cell);

        /**
         * @brief Calculates the heuristic cost from the selected cell to the goal_cell.
         * Currently the cost will be approximated with the euclidean distance.
         *
         * @param map_selected_cell Selected cell from where the heuristic cost should be calculated to the goal
         * @param map_goal_cell Goal of the path planning
         * @return float
         */
        float calcHCost(int* map_selected_cell, int* map_goal_cell);

        /**
         * @brief Calculates the heuristic cost from the selected cell to the goal_cell.
         * Currently the cost will be approximated with the euclidean distance.
         *
         * @param array_selected_cell Selected cell from where the heuristic cost should be calculated to the goal.
         * One-dimensional array index.
         * @param array_goal_cell Goal of the path planning. One-dimensional array index.
         * @return float
         */
        float calcHCost(int array_selected_cell, int array_goal_cell);

        /**
         * @brief Helper method for easy calculation of the F-score
         *
         * @param current_cell_g_score G-score of the current cell
         * @param array_current_cell Array index of the current cell
         * @param array_target_cell Array index of the target cell (mostly the neighbor cell)
         * @param array_goal_cell Array index of the global goal cell
         * @return float F-score
         */
        float calcFCost(float current_cell_g_score, int array_current_cell, int array_target_cell, int array_goal_cell);

        /**
         * @brief Gets the index of the 2D position in the 1D representing array
         * Here the costmap is represented by a 1 dimensional array.
         *
         * @param map_point Array that contains x and y index of the costmap
         * @return int Cell index in one dimensional representation
         */
        int getArrayIndexByCostmapCell(int* map_cell);

        /**
         * @brief Gets the index of the 2D position in the 1D representing array
         * Here the costmap is represented by a 1 dimensional array.
         *
         * @param map_cell_x X coordinate of the point in the map as int
         * @param map_cell_y Y coordinate of the point in the map as int
         * @return int Cell index in one dimensional representation
         */
        int getArrayIndexByCostmapCell(int map_cell_x, int map_cell_y);

        /**
         * @brief Gets the index of the 1D array in the 2D costmap
         * Here the costmap is represented by a 1 dimensional array.
         *
         * @param array_index Index of the position in the 1D-array
         * @param map_cell Cell in the two dimensional costmap
         */
        void getCostmapPointByArrayIndex(int array_index, int* map_cell);
        /**
         * @brief Gets the index of the 1D array in the 2D costmap
         * Here the costmap is represented by a 1 dimensional array.
         *
         * @param array_index Index of the position in the 1D-array
         * @param map_cell_x X-coordinate of the cell in the 2D-costmap
         * @param map_cell_y Y-coordinate of the cell in the 2D-costmap
         */
        void getCostmapPointByArrayIndex(int array_index, int& map_cell_x, int& map_cell_y);

        /**
         * @brief Helper method for checking if a specific cell is free.
         * This eases the usage of the occupancy_map array
         *
         * @param array_cell_index Index of the cell in the 1D-array format that should be checked for obstacles
         * @return true Cell is free, no obstacle in this cell
         * @return false Cell is occupied, obstacle is in this cell
         */
        bool isCellFree(int array_cell_index);

        /**
         * @brief Helper method for easy creation of a geometry_msgs::Pose object by the cell index in the 1D-representation
         * Notice that a default quaternion 0/0/0/1 is inserted and z=0.0.
         *
         * @param array_cell Index of the cell in the 1D-representation
         * @return geometry_msgs::Pose Pose of the cell
         */
        geometry_msgs::Pose createGeometryPose(int array_cell);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher point_pub_;


        //! Global frame of the robot
        std::string global_frame_;
        //! tf_prefix that was defined in the launch files for the robot
        std::string tf_prefix_;

        // Parameter list
        std::shared_ptr<ras_data_classes::RASParams> ras_params_;
        std::shared_ptr<ras_data_classes::RASParamManager> ras_param_manager_;

        // Process information
        /**
         * @brief This value defines the size of the arrays that are the 1D-representation of the costmap
         * This is calculated by multiplying the width and height of the costmap
         *
         */
        int array_size_;
        /**
         * @brief Storing the start position as pose from where the robot is starting
         *
         */
        geometry_msgs::PoseStamped start_;
        /**
         * @brief Storing the goal position as pose where the robot should arrive after following the calculated path
         *
         */
        geometry_msgs::PoseStamped goal_;
    };
}