/*
  RRTstar_ros.cpp
*/
#include <rrt_star_global_planner/RRTstar_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::RRTstarPlannerROS, nav_core::BaseGlobalPlanner)

namespace global_planner
{
  RRTstarPlannerROS::RRTstarPlannerROS() : initialized_(false) 
  {
     ROS_INFO("RRT* DEFAULT CONSTRUCTOR");
  }

  RRTstarPlannerROS::RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :  initialized_(false)
  {
     ROS_INFO("RRT* OVERLOADED CONSTRUCTOR");
     initialize(name, costmap_ros);
  }

  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
  
    if (!initialized_)
    {
      ROS_INFO("Initializing RRT* planner.");
      costmap_ = costmap_ros->getCostmap();
      frame_id_ = costmap_ros->getGlobalFrameID();
  
      ros::NodeHandle private_nh("~/" + name);
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      this->point_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker", 1000);

  
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();
	    resolution = costmap_->getResolution();
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();

      private_nh.param<float>("probability_threshold", this->probability_threshold_, 0.9);
      private_nh.param<float>("radius", this->radius_, 1.0);
      private_nh.param<float>("goal_tolerance", this->goal_tolerance_, 0.2);
      private_nh.param<float>("epsilon_min", this->epsilon_min_, 0.1);
      private_nh.param<float>("epsilon_max", this->epsilon_max_, 0.5);
      private_nh.param<int>("max_num_nodes", this->max_num_nodes_, 10000);


      initialized_ = true;
      ROS_INFO("RRT* planner initialized successfully");
    }
    else
    {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  bool RRTstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // record start time for planning to compute time taken later
    auto plan_start = std::chrono::steady_clock::now();

    // Check if the planner has been initialized
    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    
    std::vector<std::pair<float, float>> path;
    std::vector<Node> nodes;

    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;
    nodes.push_back(start_node);
    
    std::pair<float, float> p_rand;
    std::pair<float, float> p_new;
    std::chrono::microseconds sampling_total{0};
    std::chrono::microseconds getNew_total{0};
    std::chrono::microseconds optimization_total{0};

    Node node_nearest;
    while (nodes.size() < this->max_num_nodes_)
    {
      // ROS_INFO_STREAM("1");
      bool found_next = false;
      while (found_next == false)
      {
        // ROS_INFO_STREAM("2");
        auto sampling_start = std::chrono::steady_clock::now();
        p_rand = sampleFree(goal.pose.position.x, goal.pose.position.y); // get a random point in the free space
        // ROS_INFO_STREAM("2.1");
        auto sampling_end = std::chrono::steady_clock::now();
        sampling_total += std::chrono::duration_cast<std::chrono::microseconds>(sampling_end - sampling_start);
        auto getNew_start = std::chrono::steady_clock::now();
        node_nearest = getNearest(nodes, p_rand); // The nearest node of the random point
        // ROS_INFO_STREAM("2.2");
        p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate
        auto getNew_end = std::chrono::steady_clock::now();
        getNew_total += std::chrono::duration_cast<std::chrono::microseconds>(getNew_end - getNew_start);
        // ROS_INFO_STREAM("2.3");
        if (!isPointCollision(p_new.first, p_new.second))
        {
          // ROS_INFO_STREAM("3");
          Node newnode;
          newnode.x = p_new.first;
          newnode.y = p_new.second;
          newnode.node_id = nodes.size(); // index of the last element after the push_bask below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // Optimization
          auto optimization_start = std::chrono::steady_clock::now();

          newnode = chooseParent(node_nearest, newnode, nodes); // Select the best parent node for the new generated node
          nodes.push_back(newnode);
          nodes = rewire(nodes, newnode); 

          auto optimization_end = std::chrono::steady_clock::now();
          optimization_total += std::chrono::duration_cast<std::chrono::microseconds>(optimization_end - optimization_start);

          found_next = true;
        }

        // int counter = 0;
        // visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();
        // for(auto nodes : nodes)
        // {
        //   visualization_msgs::Marker start_position;
        //   start_position.id = counter;
        //   start_position.header.frame_id="map";
        //   start_position.header.stamp = ros::Time::now();
        //   start_position.type = visualization_msgs::Marker::CUBE;
        //   start_position.lifetime = ros::Duration(0);
        //   start_position.pose.position.x = nodes.x;
        //   start_position.pose.position.y = nodes.y;
        //   start_position.pose.position.z = 0.0;
        //   start_position.pose.orientation.x = 0.0;
        //   start_position.pose.orientation.y = 0.0;
        //   start_position.pose.orientation.z = 0.0;
        //   start_position.pose.orientation.w = 1.0;
        //   start_position.color.r = 1.0;
        //   start_position.color.a = 1.0;
        //   start_position.scale.x = 1.0;
        //   start_position.scale.y = 1.0;
        //   start_position.scale.z = 1.0;

        //   marker_array.markers.push_back(start_position);
        //   counter++;
        // }
        // point_pub_.publish(marker_array);


        // ROS_INFO_STREAM(nodes.size());

        

      }

      // Check if the new node reach the goal point with the distance less than the GOAL_TOLERANCE
      if (goalReach(p_new.first, p_new.second, goal.pose.position.x , goal.pose.position.y))
      {
        ROS_INFO("Time cost of sampling: %f ms", sampling_total.count()/1000.0);
        ROS_INFO("Time cost of finding new node: %f ms", getNew_total.count()/1000.0);
        ROS_INFO("Time cost of optimization: %f ms", optimization_total.count()/1000.0);

        std::pair<float, float> point;
        
        // New goal inside of the goal tolerance
        Node new_goal_node = nodes[nodes.size() - 1];
        Node current_node = new_goal_node;

        current_node = new_goal_node;
        // Final Path
        while (current_node.parent_id != -1)
        {
          point.first = current_node.x;
          point.second = current_node.y;
          path.insert(path.begin(), point); 
      
          current_node = nodes[current_node.parent_id];
        }
    
        //if the global planner find a path, publish it
        plan.clear();  //clear the plan, just in case it is not empty
        if (path.size() > 0)
        {
          auto plan_end = std::chrono::steady_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(plan_end - plan_start);
          ROS_INFO("RRT* Global Planner: Path found!!!!");
          ROS_INFO("Time cost of whole RRT* path planning: %d ms", duration.count());
          plan.push_back(start);

          // convert the points to poses
          ros::Time plan_time = ros::Time::now();
          for (int i = 0; i < path.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = frame_id_;
            pose.pose.position.x = path[i].first;
            pose.pose.position.y = path[i].second;
            plan.push_back(pose);
          }
          plan.push_back(goal);
          publishPlan(plan);
          
          return true;
        }
        else
        {
          ROS_WARN("RRT* planner failed to find a path, no elements in the path");
          return false;
        }
      }
    }
    ROS_WARN("RRT* planner failed to find a path, try to increase the max number of iterations");
    return false;
  }

  bool RRTstarPlannerROS::goalReach(float x1, float y1, float x2, float y2)
  {
    float dist = distance(x1, y1, x2, y2);
    if (dist < this->goal_tolerance_)
      return true;
    else
      return false;
  }

  float RRTstarPlannerROS::distance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
    return dist;
  }

  std::pair<float, float> RRTstarPlannerROS::sampleFree(float x_goal, float y_goal)
  {
    std::pair<float, float> random_point;
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> probability (0.0, 1.0);
    if (probability(gen) > this->probability_threshold_)
    {
      random_point.first = x_goal;
      random_point.second = y_goal;
      return random_point;
    }
    else
    {
      for (int i = 0; i < 10000; i++)
      {
        float map_width = this->map_width_;
        float map_height = this->map_height_;

        std::uniform_real_distribution<> x(-map_width, map_width);
        std::uniform_real_distribution<> y(-map_height, map_height);

        random_point.first = x(gen);
        random_point.second = y(gen);

        if (!isPointCollision(random_point.first, random_point.second))
        {
          return random_point;
        }
      }
    }
  }

  void RRTstarPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
  {
    if (plan.size() < 1)
    {
        ROS_ERROR("Plan has no elements!");
        return;
    }
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = plan;

    plan_pub_.publish(gui_path);
  }

  bool RRTstarPlannerROS::isPointCollision(float wx, float wy)
  {
    unsigned int mx, my;

    if(this->costmap_->worldToMap(wx, wy, mx, my))
    {
      unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(mx, my));
      if (cell_cost > 0)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    return true;
  }
  
  Node RRTstarPlannerROS::getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand)
  {
    Node node = nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second) < distance(node.x, node.y, p_rand.first, p_rand.second))
        node = nodes[i];
    }
  
    return node;
  }

  Node RRTstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {
    

    for (int i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < this->radius_ &&
         nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y) &&
         isLineCollision(nodes[i], nn.x, nn.y))
      {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    newnode.parent_id = nn.node_id;
  
    return newnode;
  }

  std::vector<Node> RRTstarPlannerROS::rewire(std::vector<Node> nodes, Node newnode)
  {
    Node node;
    for (int i = 0; i < nodes.size(); i++)
    {
      node = nodes[i];
      if (node != nodes[newnode.parent_id] && distance(node.x, node.y, newnode.x, newnode.y) < this->radius_ &&
          newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost && isLineCollision(node, newnode.x, newnode.y))
      {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
      }
    }
    return nodes;
  }

  std::pair<float, float> RRTstarPlannerROS::steer(float x1, float y1, float x2, float y2)
  {
    std::pair<float, float> p_new;
    float dist = distance(x1, y1, x2, y2);
    if (dist < this->epsilon_max_ && dist > this->epsilon_min_)
    {
      p_new.first = x1;
      p_new.second = y1;
      return p_new;
    }
    else
    {
      float theta = atan2(y2-y1, x2-x1);
      p_new.first = x1 + this->epsilon_max_*cos(theta);
      p_new.second = y1 + this->epsilon_max_*sin(theta);
      return p_new;
    }
  }

  bool RRTstarPlannerROS::isLineCollision(Node node_nearest, float px, float py)
  {
    int n = 1;
    float theta;

    std::pair<float, float> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    float dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution)
    {
      if (isPointCollision(px, py))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist/resolution));
      float theta;
      for (int i = 0;i < value; i++)
      {
        theta = atan2(node_nearest.y - py, node_nearest.x - px);
        p_n.first = node_nearest.x + n*resolution*cos(theta);
        p_n.second = node_nearest.y + n*resolution*sin(theta);
        if (isPointCollision(p_n.first, p_n.second))
          return false;
        
        n++;
      }
      return true;
    }
  }

}; // RRTstar_planner namespace
