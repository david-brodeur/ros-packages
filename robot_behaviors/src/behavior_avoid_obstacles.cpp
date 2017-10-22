#include <robot_behaviors/behavior_avoid_obstacles.hpp>

using namespace robot_behaviors;

BehaviorAvoidObstacles::BehaviorAvoidObstacles(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_name) : BaseType(nh, behavior_name)
{
    np.param("perception_range_topic_name", perception_range_topic_name_, std::string("/scan"));

    range_ = new robot_perceptions::PerceptionRangeDistance(nh, perception_range_topic_name_);
}

BehaviorAvoidObstacles::~BehaviorAvoidObstacles()
{
    delete range_;
}

void BehaviorAvoidObstacles::update(geometry_msgs::Twist& command)
{
    unsigned int iSample;
    unsigned int nSamples = range_->data().ranges.size();

    bool right_obstacle = detectObstacle(nSamples/8, 3*nSamples/8, OBSTACLE_MARGIN);
    bool front_obstacle = detectObstacle(3*nSamples/8, 5*nSamples/8, OBSTACLE_MARGIN);
    bool left_obstacle = detectObstacle(5*nSamples/8, 7*nSamples/8, OBSTACLE_MARGIN);

    if (front_obstacle)
    {
        command.linear.x = 0.0;
        command.angular.z = MAX_ANGULAR_VELOCITY_Z;
        //ROS_INFO("Obstacle: FRONT");
    }

    else if (right_obstacle)
    {
        command.linear.x = 0.0;
        command.angular.z = -MAX_ANGULAR_VELOCITY_Z;
        //ROS_INFO("Obstacle: RIGHT");
    }

    else if (left_obstacle)
    {
        command.linear.x = 0.0;
        command.angular.z = MAX_ANGULAR_VELOCITY_Z;
        //ROS_INFO("Obstacle: LEFT");
    }

    else
    {
        // No command updated.
    }
}

bool BehaviorAvoidObstacles::detectObstacle(unsigned int iBegin, unsigned int iEnd, float margin)
{
    unsigned int iSample;

    for (iSample = iBegin; iSample < iEnd; iSample++)
    {
        if (range_->data().ranges[iSample] <= margin)
        {
            //ROS_INFO("%f", range_->data().ranges[iSample]);
            return true;
        }
    }

    return false;
}
