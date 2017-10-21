#ifndef PERCEPTION_ODOMETRY_HPP
#define PERCEPTION_ODOMETRY_HPP

#include <ros/ros.h>

#include <nav_msgs/Odometry.hpp>
#include <tf/tf.h>

#include <robot_common/perception.hpp>

/*! 
 *  \brief     PerceptionOdometry
 *  \details   This class subscribes to an odometry message. Odometry is the 
 *             use of data from motion sensors to estimate change in position 
 *             over time.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_perceptions
{
    class PerceptionOdometry: public robot_common::Perception<nav_msgs::Odometry>
    {
        public:

            typedef robot_common::Perception<nav_msgs::Odometry> BaseType; ///< Base class type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param perception_name Name of the Perception.
            PerceptionOdometry(ros::NodeHandle& nh, std::string perception_name = "/odom"): BaseType(nh, perception_name)
            {
            }

            ///\brief Return the absolute position in x.
            double x() const { return data().pose.pose.position.x; }

            ///\brief Return the absolute position in y.
            double y() const { return data().pose.pose.position.y; }

            ///\brief Return the absolute position in z.
            double z() const { return data().pose.pose.position.z; }

            ///\brief Return the relative linear velocity in x (x dot).
            double xd() const { return data().twist.twist.linear.x; }

            ///\brief Return the relative linear velocity in y (y dot).
            double yd() const { return data().twist.twist.linear.y; }

            ///\brief Return the relative angular velocity around z (theta dot).
            double td() const { return data().twist.twist.angular.z; }
    };
}

#endif // PERCEPTION_ODOMETRY_HPP
