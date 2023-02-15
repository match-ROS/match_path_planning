#pragma once

#include "ros/ros.h"

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <relaxed_a_star/path_planner_types.h>

namespace ras_data_classes
{
    struct RASParams
    {
        // Settings for RelaxedAStar planner
        float default_tolerance;
        ras_data_classes::NeighborType neighbor_type;
        int free_cell_threshold;

        // Settings to make start and end easier for the mobile robots
        float start_straight_distance;
        float end_straight_distance;
    };

    class RASParamManager
    {
        public:
            RASParamManager(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

            void readParams(std::string formation_planner_name);
            void reportParamError(std::string param_name);
            void printInfo();

        #pragma region Getter / Setter
            std::shared_ptr<ras_data_classes::RASParams> getRASParams();
        #pragma endregion

        private:
            ros::NodeHandle& nh_;
            ros::NodeHandle& private_nh_;

            std::shared_ptr<ras_data_classes::RASParams> ras_params_;
    };
}