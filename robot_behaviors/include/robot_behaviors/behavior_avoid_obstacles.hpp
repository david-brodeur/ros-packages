#ifndef BEHAVIOR_AVOID_OBSTACLES_HPP
#define BEHAVIOR_AVOID_OBSTACLES_HPP

#include <robot_common/definitions.hpp>
#include <robot_common/behavior.hpp>
#include <robot_perceptions/perception_range_distance.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <string>

/*! 
 *  \brief     BehaviorAvoidObstacles
 *  \details   This class updates the value of a command with a zero linear 
 *             and angular velocity in all directions when an obstacle is 
 *             detected. 
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_behaviors
{
    class BehaviorAvoidObstacles: public robot_common::Behavior<geometry_msgs::Twist>
    {
        typedef robot_common::Behavior<geometry_msgs::Twist> BaseType; ///< Base class type definition.

        public:

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param np Private NodeHandle.
            ///\param behavior_name Name of the Behavior.
            BehaviorAvoidObstacles(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_name = "/behavior/avoid_obstacles");

            ///\brief Class destructor.
            ~BehaviorAvoidObstacles();

            ///\brief Initialize the behavior.
            void init();

            ///\brief Reset the behavior.
            void reset();

            ///\brief Update the command sent by the arbitration module.
            ///\param command Command to update.
            void update(geometry_msgs::Twist& command);

        private:

            ///\brief Detect if there is an obstacle within distance margin.
            ///\param iBegin First sample to scan in the data range array.
            ///\param iEnd Last sample to scan in the data range array.
            ///\param margin Minimal distance margin between the robot and an obstacle.
            ///\return true if an obstacle is detected.
            bool detectObstacle(unsigned int iBegin, unsigned int iEnd, float margin);

            std::string perception_range_topic_name_;           ///< ROS parameter - Name of the perception ROS topic.

            robot_perceptions::PerceptionRangeDistance* range_; ///< Range distance Perception.
    };
}

#endif // BEHAVIOR_AVOID_OBSTACLES_HPP
