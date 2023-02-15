#include <relaxed_a_star/ras_param_manager.h>

namespace ras_data_classes
{
    RASParamManager::RASParamManager(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(private_nh)
    {
        this->ras_params_ = std::make_shared<RASParams>();
    }

    void RASParamManager::readParams(std::string formation_planner_name)
    {
        std::string path_planner_key;
        if (this->private_nh_.searchParam(formation_planner_name, path_planner_key))
        {
            if (!this->private_nh_.getParam(path_planner_key + "/default_tolerance", this->ras_params_->default_tolerance))
            {
                this->reportParamError("default_tolerance");
            }
            int neighbor_type;
            if (!this->private_nh_.getParam(path_planner_key + "/neighbor_type", neighbor_type))
            {
                this->reportParamError("neighbor_type");
            }
            this->ras_params_->neighbor_type = (ras_data_classes::NeighborType)neighbor_type;
            if (!this->private_nh_.getParam(path_planner_key + "/free_cell_threshold",
                                            this->ras_params_->free_cell_threshold))
            {
                this->reportParamError("free_cell_threshold");
            }
            if (!this->private_nh_.getParam(path_planner_key + "/start_straight_distance",
                                            this->ras_params_->start_straight_distance))
            {
                this->reportParamError("start_straight_distance");
            }
            if (!this->private_nh_.getParam(path_planner_key + "/end_straight_distance",
                                            this->ras_params_->end_straight_distance))
            {
                this->reportParamError("end_straight_distance");
            }
        }
        else
        {
            ROS_ERROR_STREAM("RASParamManager: Path planner for the FormationPathPlanner not found in the config file.");
        }
    }

    void RASParamManager::reportParamError(std::string param_name)
    {
        ROS_ERROR_STREAM("The parameter: " << param_name << " was not set or found! This will result in faulty behaviour");
    }

    void RASParamManager::printInfo()
    {
        ROS_INFO_STREAM("RAS Params:");
        ROS_INFO_STREAM("Default Tolerance: " << this->ras_params_->default_tolerance);
        ROS_INFO_STREAM("Neighbor Type: " << this->ras_params_->neighbor_type);
        ROS_INFO_STREAM("Free Cell Threshold: " << this->ras_params_->free_cell_threshold);
        ROS_INFO_STREAM("Start Straight Distance: " << this->ras_params_->start_straight_distance);
        ROS_INFO_STREAM("End Straight Distance: " << this->ras_params_->end_straight_distance);
    }

    std::shared_ptr<ras_data_classes::RASParams> RASParamManager::getRASParams()
    {
        return this->ras_params_;
    }
} 