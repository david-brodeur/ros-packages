#ifndef PERCEPTION_RANGE_DISTANCE_HPP
#define PERCEPTION_RANGE_DISTANCE_HPP

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <robot_common/perception.hpp>

/*! 
 *  \brief     PerceptionRangeDistance
 *  \details   This class subscribes to a laserscan message. A laserscan gives 
 *             the distance from the robot to every obstacle within the field 
 *             of view of a laser range sensor.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_perceptions
{
    class PerceptionRangeDistance: public robot_common::Perception<sensor_msgs::LaserScan>
    {
        public:

            typedef robot_common::Perception<sensor_msgs::LaserScan> BaseType;    ///< Base class type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param perception_name Name of the Perception.
            PerceptionRangeDistance(ros::NodeHandle& nh, std::string perception_name = "/scan"): BaseType(nh, perception_name)
            {
            }
    };
}

#endif // PERCEPTION_RANGE_DISTANCE_HPP
