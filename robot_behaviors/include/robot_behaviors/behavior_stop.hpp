#ifndef BEHAVIOR_STOP_HPP
#define BEHAVIOR_STOP_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <robot_common/behavior.hpp>

/*! 
 *  \brief     BehaviorStop
 *  \details   This class updates the value of a command with a zero linear 
 *             and angular velocity in all directions. 
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_behaviors
{
    class BehaviorStop: public robot_common::Behavior<geometry_msgs::Twist>
    {
        typedef robot_common::Behavior<geometry_msgs::Twist> BaseType; ///< Base class type definition.

        public:

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param behavior_name Name of the Behavior.
            BehaviorStop(ros::NodeHandle& nh, std::string behavior_name = "/behavior/stop");

            ///\brief Class destructor.
            ~BehaviorStop();

            ///\brief Initialize the behavior.
            void init();

            ///\brief Reset the behavior.
            void reset();

            ///\brief Updates the value of a command with a zero linear and angular velocity in all directions.
            void update(geometry_msgs::Twist& command);
    };
}

#endif // BEHAVIOR_STOP_HPP
