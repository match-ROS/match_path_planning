#pragma once

#include <chrono>
#include <angles/angles.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64MultiArray.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <navfn/MakeNavPlan.h>
#include <dynamic_reconfigure/server.h>
#include <splined_voronoi/voronoi_generation.h>
#include <splined_voronoi/path_planning.h>
#include <splined_voronoi/path_smoothing.h>
#include <splined_voronoi/SplinedVoronoiPlannerConfig.h>

namespace splined_voronoi
{

class SplinedVoronoiPlanner : public nav_core::BaseGlobalPlanner
{
public:
    SplinedVoronoiPlanner();
    SplinedVoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ~SplinedVoronoiPlanner();

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& res);


    /**
     * @brief Requests the planner to cancel, e.g. if it takes too much time.
     * @remark New on MBF API
     * @return True if a cancel has been successfully requested, false if not implemented.
     */
    bool cancel();

private:
    bool initialized_;
    bool optimize_lengths_;
    std::shared_ptr<costmap_2d::Costmap2D> costmap_;
    std::string costmap_global_frame_;
    cv::Point2d map_origin_;
    int costmap_size_x_;
    int costmap_size_y_;
    double costmap_resolution_;  // m / px
    double free_cell_threshold_;
    double angle_threshold_;
    double min_distance_control_points_m_;
    double plan_resolution_;
    double max_curvature_;
    double curvature_safety_margin_;
    double free_space_factor_;
    double max_optimization_time_;
    std::vector<std::pair<double, double>> robot_formation_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_service_;
    ros::Publisher plan_pub_;
    ros::Publisher original_plan_pub_;
    ros::Publisher sparse_plan_pub_;
    ros::Publisher optimized_plan_pub_;
    ros::Publisher optimized_lengths_pub_;
    ros::Publisher voronoi_map_pub_;

    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    cv::Point2i start_map_;
    cv::Point2i goal_map_;
    cv::Point2d start_world_;
    cv::Point2d goal_world_;

    cv::Mat voronoi_img_;
    cv::Mat obstacle_img_;

    // output for service response
    std::vector<geometry_msgs::PoseStamped> astar_path_;
    std::vector<geometry_msgs::PoseStamped> sparse_path_;
    std::vector<geometry_msgs::PoseStamped> optimized_sparse_path_;
    std::vector<double> spline_tangent_lengths_;
    std::vector<double> time_taken_by_task_;
    double time_taken_;

    dynamic_reconfigure::Server<splined_voronoi::SplinedVoronoiPlannerConfig>* dsrv_;

    void reconfigureCB(splined_voronoi::SplinedVoronoiPlannerConfig &config, uint32_t level);

    /** @brief convertes path to ros message type for plan. Fills in Orientation for each position.
     *
     */
    void sparsePath(std::vector<cv::Point2i>& path, std::vector<cv::Point2i>& sparse_path);

    /** @brief publishes Plan for visualization with rviz; taken from https://github.com/frontw/voronoi_planner
     *
     * @param which_path which publisher should be used: 0 -> original plan; 1 -> sparse plan; everything else ->
     * final plan
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);
};
};  // namespace splined_voronoi
