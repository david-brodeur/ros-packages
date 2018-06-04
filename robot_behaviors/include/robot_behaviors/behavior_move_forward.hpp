#ifndef BEHAVIOR_MOVE_FORWARD_HPP
#define BEHAVIOR_MOVE_FORWARD_HPP

#include <robot_common/definitions.hpp>
#include <robot_common/behavior.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/*! 
 *  \brief     BehaviorMoveForward
 *  \details   This class updates the value of a command to move the robot 
 *             forward at constant velocity. 
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_behaviors
{
    class BehaviorMoveForward: public robot_common::Behavior<geometry_msgs::Twist>
    {
        typedef robot_common::Behavior<geometry_msgs::Twist> BaseType; ///< Base class type definition.

        public:

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param np Private NodeHandle.
            ///\param behavior_name Name of the Behavior.
            BehaviorMoveForward(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_name = "/behavior/move_forward");

            ///\brief Class destructor.
            ~BehaviorMoveForward();

            ///\brief Initialize the behavior.
            void init();

            ///\brief Reset the behavior.
            void reset();

            ///\brief Updates the value of a command to move the robot forward at constant velocity.
            void update(geometry_msgs::Twist& command);

            REGISTER(geometry_msgs::Twist, BehaviorMoveForward);
    };
}

#endif // BEHAVIOR_MOVE_FORWARD_HPP
