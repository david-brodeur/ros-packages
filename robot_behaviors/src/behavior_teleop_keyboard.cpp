#include <robot_behaviors/behavior_teleop_keyboard.hpp>

using namespace robot_behaviors;

REGISTERIMPL(geometry_msgs::Twist, BehaviorTeleopKeyboard);

BehaviorTeleopKeyboard::BehaviorTeleopKeyboard(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_name) : BaseType(nh, behavior_name)
{
    np.param("control_keyboard_topic_name", control_keyboard_topic_name_, std::string("/teleop/keyboard"));

    keyboard_ = new robot_control::ControlVelocity(nh, control_keyboard_topic_name_);
}

BehaviorTeleopKeyboard::~BehaviorTeleopKeyboard()
{
    reset();

    delete keyboard_;
}

void BehaviorTeleopKeyboard::init()
{
    reset();
}

void BehaviorTeleopKeyboard::reset()
{
}

void BehaviorTeleopKeyboard::update(geometry_msgs::Twist& cmd)
{
    cmd.linear.x = keyboard_->x();
    cmd.linear.y = keyboard_->y();
    cmd.linear.z = keyboard_->z();

    cmd.angular.x = keyboard_->xd();
    cmd.angular.y = keyboard_->yd();
    cmd.angular.z = keyboard_->zd();
}
