#ifndef BEHAVIOR_LAYER_MOBILE_HPP
#define BEHAVIOR_LAYER_MOBILE_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <robot_common/behavior_layer.hpp>

/*! 
 *  \brief     BehaviorLayerMobile
 *  \details   This class is a BehaviorLayer for behavior-based mobile 
 *             robot applications. 
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_behaviors
{
    class BehaviorLayerMobile: public robot_common::BehaviorLayer<geometry_msgs::Twist>
    {
        typedef robot_common::BehaviorLayer<geometry_msgs::Twist> BaseType; ///< Base class type definition.

        public:

            ///\brief Class constructor.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle.
            ///\param behavior_name Name of the Behavior.
            BehaviorLayerMobile(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_layer_name = "/behavior_layer/mobile");

            ///\brief Class destructor.
            ~BehaviorLayerMobile();
    };
}

#endif // BEHAVIOR_LAYER_MOBILE_HPP
