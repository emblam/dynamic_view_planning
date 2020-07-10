#pragma once

#include "ros/ros.h"
#include "ig_active_reconstruction/robot_communication_interface.hpp"

#include "ig_active_reconstruction_msgs/ViewRequest.h"
#include "ig_active_reconstruction_msgs/RetrieveData.h"
#include "ig_active_reconstruction_msgs/MovementCostCalculation.h"
#include "ig_active_reconstruction_msgs/MoveToOrder.h"

namespace robot
{
class virtualRobot: public ig_active_reconstruction::robot::CommunicationInterface
{
public: 

    typedef typename ig_active_reconstruction::views::View View;
    typedef typename ig_active_reconstruction::robot::CommunicationInterface::ReceptionInfo ReceptionInfo;
    typedef typename ig_active_reconstruction::robot::MovementCost MovementCost;

private:

    ros::NodeHandle nh_priv;
    ros::ServiceClient change_camera_;
    std::string current_camera_;
    View current_view_;

public: 

    virtualRobot(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

    View getCurrentView();

    ReceptionInfo retrieveData();

    MovementCost movementCost(View& target_view);

    MovementCost movementCost(View& start_view, View& target_view, 
                            bool fill_additional_information);

    bool moveTo(View& target_view);

};

}