#include "dynamic_view_planning/virtual_robot.hpp"
#include "dynamic_view_planning/tools.hpp"

namespace virtual_robot
{
    typedef typename ig_active_reconstruction::views::View View;
    typedef typename ig_active_reconstruction::robot::CommunicationInterface::ReceptionInfo ReceptionInfo;
    typedef typename ig_active_reconstruction::robot::MovementCost MovementCost;

virtualRobot::virtualRobot(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
{
    //What should construct do?
    camera_client_ = nh_priv.serviceClient<dynamic_view_planning::ChangeCamera>("change_camera");
}

View virtualRobot::getCurrentView()
{
    View view = viewFromTFtopic();

    return view;
}

ReceptionInfo virtualRobot::retrieveData()
{
    //Should this function do something? Or should we record data continuously?
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

bool moveTo(View& target_view)
{
    std::string new_topic = viewToTFtopic(target_view);

    dynamic_view_planning::ChangeCamera srv;

    if 
    
}

}