#include <robot_behaviors/behavior_move_forward.hpp>

using namespace robot_behaviors;

BehaviorMoveForward::BehaviorMoveForward(ros::NodeHandle& nh, std::string behavior_name) : BaseType(nh, behavior_name)
{
}

BehaviorMoveForward::~BehaviorMoveForward()
{
}

void BehaviorMoveForward::update(geometry_msgs::Twist& cmd)
{
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    cmd.linear.x = MAX_LINEAR_VELOCITY_X;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
}
