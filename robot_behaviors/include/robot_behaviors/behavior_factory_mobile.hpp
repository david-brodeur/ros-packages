#ifndef BEHAVIOR_FACTORY_MOBILE_HPP
#define BEHAVIOR_FACTORY_MOBILE_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <robot_common/behavior.hpp>
#include <robot_common/behavior_factory.hpp>

#include <string>

/*! 
 *  \brief     FactoryMobile
 *  \details   This class creates behaviors for a mobile robot base.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_behaviors
{
    class BehaviorFactoryMobile: public robot_common::BehaviorFactory<geometry_msgs::Twist>
    {
        typedef robot_common::BehaviorFactory<geometry_msgs::Twist> BaseType; ///< Base class type definition.

        public:

            ///\brief Class constructor.
            ///\param behavior_factory_name Name of the BehaviorFactory. Default = "/behavior_factory/mobile".
            BehaviorFactoryMobile(std::string behavior_factory_name = "/behavior_factory/mobile");

            ///\brief Class destructor.
            ~BehaviorFactoryMobile();

            ///\brief Create a Behavior.
            ///\param behavior_name Name of the Behavior.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle.
            robot_common::Behavior<geometry_msgs::Twist>* create(std::string behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np);
    };
}

#endif // BEHAVIOR_FACTORY_MOBILE_HPP
