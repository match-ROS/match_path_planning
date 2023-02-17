#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <splined_voronoi/splined_voronoi_planner.h>


int main(int argc, char**argv)
{
	ros::init(argc, argv, "move_base_node");
    ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

    // Subscriber on position
    // subscriber to goal
    // create instace of SplinedVoronoiPlanner
    // init instance
    // call make Plan function

    // subscriber to map
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    nav_msgs::OccupancyGrid map;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh);
    if(map_ptr != NULL){
        map = *map_ptr;
    }

    // subscriber to costmap
    boost::shared_ptr<nav_msgs::OccupancyGrid const> costmap_ptr;
    nav_msgs::OccupancyGrid costmap;
    costmap_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap", nh);
    if(costmap_ptr != NULL){
        costmap = *costmap_ptr;
    }

    tf2_ros::Buffer* buffer = new tf2_ros::Buffer(ros::Duration(10));
    tf2_ros::TransformListener* tf = new tf2_ros::TransformListener(*buffer);
    costmap_2d::Costmap2DROS* planner_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", *buffer);
    planner_costmap_ros->pause();

    splined_voronoi::SplinedVoronoiPlanner global_planner;
    
    global_planner.initialize("SplinedVoronoiPlanner", planner_costmap_ros);

    planner_costmap_ros->start();


    while (ros::ok())
    {
        ROS_INFO("Ready for planning!");
        // subscribe to start pose
        /*
        */
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> start_pose_ptr;
        geometry_msgs::PoseWithCovarianceStamped start_pose_with_cov;
        start_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", nh);
        if(start_pose_ptr != NULL){
            start_pose_with_cov = *start_pose_ptr;
        }
        geometry_msgs::PoseStamped start_pose;
        start_pose.header = start_pose_with_cov.header;
        start_pose.pose = start_pose_with_cov.pose.pose;
        start_pose.header.frame_id = "map";
        // vortrag example
        // start_pose.pose.orientation.w = 1;
        // start_pose.pose.position.x = 10.028;
        // start_pose.pose.position.y = 10.7379;
        // willowx2 long dijkstra time
        // start_pose.pose.orientation.w = 1;
        // start_pose.pose.position.x = 110.57;
        // start_pose.pose.position.y = 88.79;
        ROS_INFO_STREAM("Start pose: " << start_pose.pose.position.x << ", " << start_pose.pose.position.y);
        // subscribe to goal pose
        geometry_msgs::PoseStamped goal_pose;
        /*
        */
        boost::shared_ptr<geometry_msgs::PoseStamped const> goal_pose_ptr;
        goal_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/move_base_simple/goal", nh);
        if(goal_pose_ptr != NULL){
            goal_pose = *goal_pose_ptr;
        }
        goal_pose.header.frame_id = "map";
        // vortrag example
        // goal_pose.pose.orientation.w = 1;
        // goal_pose.pose.position.x = -12.5606;
        // goal_pose.pose.position.y = -6.31814;
        // willowx2 long dijkstra time
        // goal_pose.pose.orientation.w = 1;
        // goal_pose.pose.position.x = 86.21;
        // goal_pose.pose.position.y = 89.39;
        ROS_INFO_STREAM("Goal pose: " << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y);

        std::vector<geometry_msgs::PoseStamped> path;
        global_planner.makePlan(start_pose, goal_pose, path);
        // break;
    }

	return 0;
}

