#include <robot_behaviors/mobile/behavior_stop.hpp>

using namespace robot_behaviors;

REGISTERIMPL_BEHAVIOR_MOBILE(BehaviorStop);

BehaviorStop::BehaviorStop(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_name) : BaseType(nh, behavior_name)
{
}

BehaviorStop::~BehaviorStop()
{
    reset();
}

void BehaviorStop::init()
{
    reset();
}

void BehaviorStop::reset()
{
}

void BehaviorStop::update(geometry_msgs::Twist& cmd)
{
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
}
