#ifndef RRTSTAR_ROS_CPP
#define RRTSTAR_ROS_CPP

#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h> 

/** for global path planner interface **/
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
#include <utility>
#include <boost/random.hpp>
#include <random>

#include <visualization_msgs/MarkerArray.h>

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
  float cost;
  
  bool operator ==(const Node& node) 
  {
	  return (x == node.x) && (y == node.y) && (node_id == node.node_id) && (parent_id == node.parent_id) && (cost == node.cost) ;
  }

  bool operator !=(const Node& node) 
  {
    if((x != node.x) || (y != node.y) || (node_id != node.node_id) || (parent_id != node.parent_id) || (cost != node.cost))
      return true;
    else
      return false;
  }
}; 


namespace global_planner 
{

  class RRTstarPlannerROS : public nav_core::BaseGlobalPlanner 
  {

    public:
      /**
      * @brief Default constructor of the plugin
      */
      RRTstarPlannerROS();

      /**
       * @brief Construct a new RRTstarPlannerROS object
       * 
       * @param name The name of this planner
       * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */

      RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

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
       * @param plan The plan filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
      
      /**
       * @brief Publish the plan for visualization purposes
       * @param path The plan filled by the planner
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
      
      /*
      * @brief Compute the euclidean distance between two points
      * @param px1 The x coordinate of the first point
      * @param py1 The y coordinate of the first point
      * @param px2 The x coordinate of the second point
      * @param py2 The y coordinate of the second point
      * @return the distance computed
      */
      float distance(float px1, float py1, float px2, float py2);

      /**
       * @brief Randomly sample a point in the free space
       * @param x_goal The x coordinate of goal pose
       * @param y_goal The y coordinate of goal pose
       * @return A random point in the free space
      */
      std::pair<float, float> sampleFree(float x_goal, float y_goal);

      /**
       * @brief Check if the point collides with obstacles)
       * @param wx The x coordinate of the point (in the world frame)
       * @param wy The y coordinate of the point (in the world frame)
       * @return True if the point collides with an obstacle, false otherwise
      */
      bool isPointCollision(float wx, float wy);

      /**
       * @brief Find the nearest node to the random sampled point
       * @param nodes The set of all exist nodes
       * @param p_rand The coordinate of random sampled point (x,y)
       * @return The nearest node in the all exist nodes to the random point
      */
      Node getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand);

      /**
       * @brief Select the best parent node of the new generated node
       * @param nn The parent node of the new generated node
       * @param newnode The new generated node
       * @param nodes The set of all nodes
       * @return the new generated node with the best parent node
       * 
      */
      Node chooseParent(Node nn, Node newnode, std::vector<Node> nodes);


      /**
       * @brief Check if the cost of the parents of all nodes around is still less than the newnode. 
       * If there is a node with parent with higher cost the new parent of this node is the newnode now.
       * @param nodes The set of all nodes
       * @param newnode the new generated node
       * @return The nodes set after rewiring
      */
      std::vector<Node> rewire(std::vector<Node> nodes, Node newnode);

      /**
       * @brief Generate the new point along the line connected by random point and nearest node. 
       * @param px1 The x coordinate of the nearest node 
       * @param py1 The y coordinate of the nearest node 
       * @param px2 The x coordinate of the random point 
       * @param py2 The y coordinate of the random point
       * @return The new generated point
      */
      std::pair<float, float> steer(float x1, float y1, float x2, float y2);
      
      /**
       * @brief Check if the line connected by the point and its nearest node collides with obstacles)
       * @param node_nearest The nearest node of the point
       * @param px The x coordinate of the point 
       * @param py The y coordinate of the point 
       * @return True if this line collides with an obstacle, false otherwise
      */
      bool isLineCollision(Node node_nearest, float px, float py);

      /**
       * @brief Check if the new node reach the goal point with their distance less than the GOAL_TOLERANCE. 
       * @param px1 The x coordinate of the new generated node 
       * @param py1 The y coordinate of the new generated node 
       * @param px2 The x coordinate of the goal point 
       * @param py2 The y coordinate of the goal point
       * *@return True if distance is less than the GOAL_TOLERANCE, False otherwise
      */
      bool goalReach(float x1, float y1, float x2, float y2);

      
    protected:

      /**
      * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
      */
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;

    private:
      int max_num_nodes_;      // The maximum number of new nodes that can be generated when planning a path
      float probability_threshold_; // The threshold for determining whether regard the goal point as the random point or sample a point randomly
      float radius_;          // The radius range of nearby nodes of new generated node during optimization
      float goal_tolerance_;  // Tolerance that allows new generated node to be considered as goal point
      float epsilon_min_;     // The minimum value of the step length used when generating new node
      float epsilon_max_;     // The maximum value of the step length used when generating new node
      
      ros::Publisher plan_pub_;

      float originX;
      float originY;
      float resolution;
      bool initialized_;
      std::string frame_id_;
      float map_width_;
      float map_height_;

      ros::Publisher point_pub_;

  };
}; // RRTstar_planner namespace
#endif
