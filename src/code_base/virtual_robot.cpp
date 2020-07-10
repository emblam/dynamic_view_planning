#include "dynamic_view_planning/virtual_robot.hpp"
#include "dynamic_view_planning/tools.hpp"
#include "dynamic_view_planning/ChangeCamera.h"

namespace robot
{
    typedef typename ig_active_reconstruction::views::View View;
    typedef typename ig_active_reconstruction::robot::CommunicationInterface::ReceptionInfo ReceptionInfo;
    typedef typename ig_active_reconstruction::robot::MovementCost MovementCost;

virtualRobot::virtualRobot(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
    change_camera_ = nh_priv.serviceClient<dynamic_view_planning::ChangeCamera>("change_camera");
}

View virtualRobot::getCurrentView()
{
    return current_view_;
}

ReceptionInfo virtualRobot::retrieveData()
{
    /* At the moment, we only feed the correct camera stream into ufomap_server, so  
     * that the map can be updated continuously up until the ig is calculated. 
     * Ig and evaluation is performed on that map from world_rep module. 
     * Let's hope it works. 
     * 
     * If it doesn't, we might have to change this function. 
     */

    return ReceptionInfo::SUCCEEDED;
}

MovementCost virtualRobot::movementCost(View& target_view)
{
    MovementCost move_cost;
    move_cost.cost = 0;

    return move_cost;
}

MovementCost virtualRobot::movementCost(View& start_view, View& target_view,
                                        bool fill_additional_information)
{
    MovementCost move_cost;
    move_cost.cost = 0;

    return move_cost;
}

bool virtualRobot::moveTo(View& target_view)
{
    bool success;

    std::string target_camera = target_view.additionalFieldsNames()[0];

    dynamic_view_planning::ChangeCamera srv;
    srv.request.new_camera = target_camera;

    if (change_camera_.call(srv))
    {
        current_view_ = target_view;
        current_camera_ = target_camera;

        ROS_INFO("Changed to %ld", current_camera_);

        return true;
    }
    else 
    {
        ROS_WARN("Couldn't move!");
        return false;
    }

    return success;
}
}