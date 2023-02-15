#pragma once

#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h> 

/** for global path planner interface **/
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** include standard libraries **/
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <queue>
#include <utility>
#include <boost/random.hpp>
#include <random>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using std::string;



/**
 * @brief Node struct
 * 
*/
struct Node {
  float x;
  float y;
  int node_id;
  int parent_id;
  float f; // f = g + h
  float g;
  float h;
  bool operator ==(const Node& node) 
  {
     return (x == node.x) && (y == node.y) && (node_id == node.node_id) ;
  }

  bool operator !=(const Node& node) 
  {
    if((x != node.x) || (y != node.y) || (node_id != node.node_id))
      return true;
    else
      return false;
  }
}; 

struct cmp 
{
   bool operator() ( Node n1,  Node n2) { return n1.f > n2.f;}
};

struct cmp1 
{
   bool operator() (std::pair<int,float> a, std::pair<int,float> b) { return a.second > b.second;}
};


namespace global_planner 
{

  class PRMPlannerROS : public nav_core::BaseGlobalPlanner 
  {

    public:
      /**
      * @brief Default constructor of the plugin
      */
      PRMPlannerROS();
      PRMPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
      * @brief  Initialization function for the PlannerCore object
      * @param  name The name of this planner
      * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
      */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
      /**
       * @brief Publish the plan
       * @param plan The plan... filled by the planner
      */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
      
      /**
       * @brief Randomly sample a point in the free space
       * @return A random point in the free space of the cartesian plane
      */
      std::pair<float, float> sampleFree();
      
      /**
       * @brief check if the node collides with obstacles
       * @param wx Coordinate x of node in world coordinate
       * @param wy Coordinate y of node in world coordinate
       * @return True if the node collides with obstacles, false otherwise
      */
      bool isPointCollision(float wx, float wy);

      /**
       * @brief check if the edge connected by two nodes collides with obstacles or is longer than the max distance
       * @param px1 Coordinate x of first node in world coordinate
       * @param py1 Coordinate y of first node in world coordinate
       * @param px2 Coordinate x of second node in world coordinate
       * @param py2 Coordinate y of second node in world coordinate
       * @return True if edge connected by two nodes collides with obstacles or is longer than the max distance, false otherwise
      */
      bool isLineCollision(float px1, float py1, float px2, float py2);
      
      /**
       * @brief calculate the euclidean distance between two nodes
       * @param px1 Coordinate x of first node in world coordinate
       * @param py1 Coordinate y of first node in world coordinate
       * @param px2 Coordinate x of second node in world coordinate
       * @param py2 Coordinate y of second node in world coordinate
       * @return the euclidean distance between two nodes
      */
      float GetEuclideanDistance(float px1, float py1, float px2, float py2);
      
      /**
       * @brief Find a path using A* algorithm
       * @param nodes All nodes in roadmap
       * @param neighbours_list It stores neighbours'id for each node
       * @param start_point The start pose
       * @param goal_point The goal pose
       * @return True if a valid path was found, false otherwise
       */
      bool findPath(std::vector<Node>& nodes, std::vector<std::vector<int>>& neighbours_list, Node& start_point, Node& goal_point);
      
      /**
       * @brief Search the parent node from goal point until start point if a path using A* algorithm was found
       * @param path Feasible path contains path nodes' coordinate
       * @param nodes All nodes in roadmap
       * @param start_pose The start pose
       * @param goal_pose The goal pose
       * @return The found path from start point to goal point
       */      
      void getPath(std::vector<std::pair<float, float>> & path,std::vector<Node>& nodes, Node start_pose, Node goal_pose);
      
      /**
       * @brief check if a node is in the open/closed list, called by findPath
       * @param Node_id Id of the node to be checked
       * @param List Open/closed list
       * @return True if the node is in the list, false otherwise
       */      
      bool isInList(int Node_id, std::priority_queue<Node,std::vector<Node>, cmp> List);

      
    protected:

      /**
      * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
      */
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
    

    private:
    
      int num_samples_; // number of sampling points
      float max_distance_; // maximum distance between two nodes can be connected
      int num_edges_; // max number of edges that a node can have 
      
      ros::Publisher plan_pub_;
      ros::Publisher point_pub_;

      std::string frame_id_;
      float originX;
      float originY;
      float resolution;
      bool initialized_;
      int width;
      int height;

  };
}; // PRM_planner namespace
