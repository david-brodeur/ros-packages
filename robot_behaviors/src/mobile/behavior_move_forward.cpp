#include <robot_behaviors/mobile/behavior_move_forward.hpp>

using namespace robot_behaviors;

REGISTERIMPL_BEHAVIOR_MOBILE(BehaviorMoveForward);

BehaviorMoveForward::BehaviorMoveForward(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_name) : BaseType(nh, behavior_name)
{
}

BehaviorMoveForward::~BehaviorMoveForward()
{
    reset();
}

void BehaviorMoveForward::init()
{
    reset();
}

void BehaviorMoveForward::reset()
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
