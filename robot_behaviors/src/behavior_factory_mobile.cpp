#include <robot_behaviors/behavior_factory_mobile.hpp>

#include <robot_behaviors/behavior_avoid_obstacles.hpp>
#include <robot_behaviors/behavior_move_forward.hpp>
#include <robot_behaviors/behavior_stop.hpp>

using namespace robot_behaviors;

BehaviorFactoryMobile::BehaviorFactoryMobile(std::string behavior_factory_name) : BaseType(behavior_factory_name)
{
}

BehaviorFactoryMobile::~BehaviorFactoryMobile()
{
}

robot_common::Behavior<geometry_msgs::Twist>* BehaviorFactoryMobile::create(std::string behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np)
{
    if (behavior_name == "avoid")
        return new BehaviorAvoidObstacles(nh, np);
    else if (behavior_name == "move")
        return new BehaviorMoveForward(nh, np);
    else if (behavior_name == "stop")
        return new BehaviorStop(nh, np);
    else
        return NULL;
}
