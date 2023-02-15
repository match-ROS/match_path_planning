/*
  PRM_ros.cpp
*/
#include <prm_global_planner/PRM_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::PRMPlannerROS, nav_core::BaseGlobalPlanner)

namespace global_planner
{
  PRMPlannerROS::PRMPlannerROS() 
        : initialized_(false) { }

  PRMPlannerROS::PRMPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
        : costmap_ros_(costmap_ros)
  {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  void PRMPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
  
    if (!initialized_)
    {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
  
      ros::NodeHandle private_nh = ros::NodeHandle("~/" + name);
  
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();
	    width = costmap_->getSizeInCellsX();
	    height = costmap_->getSizeInCellsY();
	    resolution = costmap_->getResolution();
      frame_id_ = costmap_ros->getGlobalFrameID();
      
      private_nh.param<int>("num_samples", this->num_samples_, 1000);
      private_nh.param<float>("max_distance", this->max_distance_, 10.0);
  		private_nh.param<int>("num_edges", this->num_edges_, 10);

      ROS_INFO_STREAM("total number of sampling points: " << this->num_samples_);
      ROS_INFO_STREAM("max distance of connected edge: " << this->max_distance_);
      ROS_INFO_STREAM("max number of connected edge: " << this->num_edges_);

      ROS_INFO("PRM planner initialized successfully");
      initialized_ = true;

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      point_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("prm_marker", 1000);
    }
    else
      ROS_WARN("This PRM planner has already been initialized... doing nothing");
  }

  bool PRMPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // record time cost of PRM path planning
    std::chrono::microseconds learning{0};
    std::chrono::microseconds query{0};
    std::chrono::microseconds planning{0};
    auto planning_start = std::chrono::steady_clock::now();
    
    // Check if the planner is initialized
    if (!initialized_)
    {
      ROS_ERROR("This PRM planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    //create a vector to contain all sampled points and start and goal points
    std::vector<Node> nodes;
    nodes.reserve(this->num_samples_ + 2); 
    std::string global_frame = frame_id_;

    // Add start point into the nodes vector
    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0; // start node id is 0
    start_node.g= 0.0;
    start_node.h= 0.0;
    start_node.f = 0.0;
    nodes.push_back(start_node);

    // Generate the sampling nodes in free space
    auto learning_start = std::chrono::steady_clock::now();
    std::pair<float, float> rand_p; 
    for (int i = 0; i < this->num_samples_; i++)
    {
      rand_p = sampleFree();
      Node new_node;
      new_node.x = rand_p.first;
      new_node.y = rand_p.second;
      new_node.node_id = i+1;  // id of sampling points start from 1
      nodes.push_back(new_node);
    }

    ROS_ERROR_STREAM(nodes.size());
    
    visualization_msgs::MarkerArray target_position_list;
		int counter = 0;
		for(auto node : nodes)
		{
			visualization_msgs::Marker start_position;
			start_position.id = counter;
			start_position.header.frame_id="map";
			start_position.header.stamp = ros::Time::now();
			start_position.type = visualization_msgs::Marker::CUBE;
			start_position.lifetime = ros::Duration(0);
			start_position.pose.position.x = node.x;
			start_position.pose.position.y = node.y;
			start_position.pose.position.z = 0.0;
			start_position.pose.orientation.x = 0.0;
			start_position.pose.orientation.y = 0.0;
			start_position.pose.orientation.z = 0.0;
			start_position.pose.orientation.w = 1.0;
			start_position.color.r = 1.0;
			start_position.color.a = 1.0;
			start_position.scale.x = 1.0;
			start_position.scale.y = 1.0;
			start_position.scale.z = 1.0;

			target_position_list.markers.push_back(start_position);
			counter++;
		}
		this->point_pub_.publish(target_position_list);


    // Add goal point into the nodes vector
    Node goal_node;
    goal_node.x = goal.pose.position.x;
    goal_node.y = goal.pose.position.y;
    goal_node.node_id = this->num_samples_ + 1;
    nodes.push_back(goal_node);

    // Find neighbours (available edges) for each node
    auto findNeighbour_start = std::chrono::steady_clock::now();

    std::vector< std::vector<int> > neighbours_list; //it stores neighbours'id for each node
    neighbours_list.reserve(this->num_samples_ + 2);

    for (int i = 0; i < nodes.size(); i++)
    {
      std::priority_queue<std::pair<int, float>, std::vector< std::pair<int,float> >, cmp1> node_pair_list;
      for (int j = 0; j < nodes.size(); j++)
      {
        std::pair<int, float> node_pair;
        node_pair.first = j;
        node_pair.second = GetEuclideanDistance(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y);
        node_pair_list.push(node_pair);
      }

      std::vector<int> neighbours;
      int edge_count = 0;
      while (!node_pair_list.empty() && edge_count <= this->num_edges_)
      {
        std::pair<int, float> node_nearest = node_pair_list.top();
        if (node_nearest.second <= this->max_distance_ && node_nearest.first != i && !isLineCollision(nodes[i].x, nodes[i].y, nodes[node_nearest.first].x, nodes[node_nearest.first].y))
        {
          neighbours.push_back(node_nearest.first);
          node_pair_list.pop();
          edge_count = edge_count + 1;
        }
        else
        {
          node_pair_list.pop();
        }
      }
      neighbours_list.push_back(neighbours);
    }

    auto learning_end = std::chrono::steady_clock::now();
    learning = std::chrono::duration_cast<std::chrono::microseconds>(learning_end - learning_start);
    ROS_INFO("Time cost of learning phase: %f ms", learning.count()/1000.0);
    
    // Find the path using A* algorithm
    auto query_start = std::chrono::steady_clock::now();
    std::vector<std::pair<float, float>> path;
    if (findPath(nodes, neighbours_list, start_node, goal_node))
    {
      getPath(path, nodes, start_node, goal_node);
    }
    else
    {
      ROS_WARN("Failed to find a path using PRM!");
    }
    auto query_end = std::chrono::steady_clock::now();
    query = std::chrono::duration_cast<std::chrono::microseconds>(query_end - query_start);
    ROS_INFO("Time cost of query phase: %f ms", query.count()/1000.0);

    //create plan message
    plan.clear();
    plan.push_back(start);
    if (path.size() > 0)
    {
      // show planning time
      auto planning_end = std::chrono::steady_clock::now();
      planning = std::chrono::duration_cast<std::chrono::microseconds>(planning_end - planning_start);
      ROS_INFO("Time cost of whole PRM path planning: %f ms", planning.count()/1000.0);

      // convert the point coordinate to pose
      ros::Time plan_time = ros::Time::now();
      for (int i = 1; i < path.size() -1; i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = path[i].first;
        pose.pose.position.y = path[i].second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
      }
      plan.push_back(goal);
      publishPlan(plan);
      return true;
    }
    else
    {
      ROS_WARN("Failed to find a path! There is no path elements!");
      return false;
    }
  }


  std::pair<float, float> PRMPlannerROS::sampleFree()
  {
    std::pair<float, float> random_point;
    float map_width = costmap_->getSizeInMetersX();
    float map_height = costmap_->getSizeInMetersY();
    // float map_width = 30.0;
    // float map_height = 30.0;
    bool findNode = false;
    while(!findNode)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis_x(originX, originX + map_width);
      std::uniform_real_distribution<> dis_y(originY, originY + map_height);
      // std::uniform_real_distribution<> dis_x(0.0-map_width, map_width);
      // std::uniform_real_distribution<> dis_y(0.0-map_height, map_height);
      random_point.first = dis_x(gen);
      random_point.second = dis_y(gen);
      if (!isPointCollision(random_point.first, random_point.second))
      {
        findNode = true;
      }
    }
    return random_point;
  }

  bool PRMPlannerROS::isPointCollision(float wx, float wy)
  {
    unsigned int mx, my;
    this->costmap_->worldToMap(wx, wy, mx, my);
    unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(mx, my));
    if (cell_cost > 0 && cell_cost != -1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool PRMPlannerROS::isLineCollision(float px1, float py1, float px2, float py2)
  {
    float Dist = GetEuclideanDistance(px1, py1, px2, py2);
    float dx = (px2 - px1) / Dist;
    float dy = (py2 - py1) / Dist;
    float x = px1;
    float y = py1;
    float n = 0.0;
    float step = 0.1;
    while (n < Dist)
    {
      if (isPointCollision(x, y))
      {
        return true;
      }
      x += dx * step;
      y += dy * step;
      n += step;
    }

  }

  float PRMPlannerROS::GetEuclideanDistance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt(pow((px1 - px2),2) + pow((py1 - py2),2));
    return dist;
  }


  bool PRMPlannerROS::findPath(std::vector<Node>& nodes, std::vector<std::vector<int>>& neighbours_list, Node& start_point, Node& goal_point)
  {
    std::priority_queue<Node,std::vector<Node>, cmp> openList;
	  std::priority_queue<Node,std::vector<Node>, cmp> closeList;
    openList.push(start_point);
    while (!openList.empty())
    {
      // get node with lowest cost in openlist as current node
      Node current_node = openList.top();
      // remove it from openlist so that it won't be visited again
      openList.pop();
      closeList.push(current_node);
      if (current_node.node_id == goal_point.node_id)
      {
        ROS_INFO("Find the path in PRM!");
        return true;
      }
      // get all neighbours(id) of current node
      std::vector<int> current_neighbours = neighbours_list[current_node.node_id];
      if (current_neighbours.size() == 0)
      {
        ROS_WARN("Failed to find a path! There is no available edge for a certain node!");
        return false;
      }
      for (int i = 0; i < current_neighbours.size(); i++)
      {
        // get one neighbour of current node
        int neighbour_id = current_neighbours[i];
        if (isInList(neighbour_id, openList) == false && isInList(neighbour_id, closeList) == false)
        {
          nodes[neighbour_id].parent_id = current_node.node_id;
          nodes[neighbour_id].g = current_node.g + GetEuclideanDistance(current_node.x, current_node.y, nodes[neighbour_id].x, nodes[neighbour_id].y);
          nodes[neighbour_id].h = GetEuclideanDistance(nodes[neighbour_id].x, nodes[neighbour_id].y, goal_point.x, goal_point.y);
          nodes[neighbour_id].f = nodes[neighbour_id].g + nodes[neighbour_id].h;
          openList.push(nodes[neighbour_id]);
        }
        else if (isInList(neighbour_id, openList) == true)
        {
          float g_new = current_node.g + GetEuclideanDistance(current_node.x, current_node.y, nodes[neighbour_id].x, nodes[neighbour_id].y);
          if (g_new < nodes[neighbour_id].g)
          {
            nodes[neighbour_id].parent_id = current_node.node_id;
            nodes[neighbour_id].g = g_new;
            nodes[neighbour_id].f = nodes[neighbour_id].g + nodes[neighbour_id].h;
          }
        }
      }
    }
    return false;
  }

  // 
  void PRMPlannerROS::getPath(std::vector<std::pair<float, float>> & path, std::vector<Node>& nodes, Node start_pose, Node goal_pose)
  {
    Node current_point;
    current_point.node_id = goal_pose.node_id;
    current_point.x = goal_pose.x;
    current_point.y = goal_pose.y;
    while (current_point != start_pose)
    {
      for (int i = 0;i < nodes.size();i++)
      {
        if (current_point.node_id == nodes[i].node_id)
        {
          std::pair<float, float> path_point;
          path_point.first = nodes[i].x;
          path_point.second = nodes[i].y;
          path.push_back(path_point);
          int current_id = nodes[i].parent_id;
          current_point.x = nodes[current_id].x;
          current_point.y = nodes[current_id].y;
          current_point.node_id = current_id;
          break;
        }
      }
    }
    std::pair<float, float> path_point;
    path_point.first = current_point.x;
    path_point.second = current_point.y;
    path.push_back(path_point);
    std::reverse(path.begin(),path.end());
  }

  bool PRMPlannerROS::isInList(int Node_id, std::priority_queue<Node,std::vector<Node>, cmp> List)
  {
    while (!List.empty())
    {
      if (Node_id == List.top().node_id)
      {
        return true;
      }
      else 
      {
        List.pop();
      }
    }
    return false;
  }

  void PRMPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
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

}; // PRM_planner namespace
