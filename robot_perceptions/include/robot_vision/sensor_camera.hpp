#ifndef SENSOR_CAMERA_HPP
#define SENSOR_CAMERA_HPP

#include <ros/ros.h>

#include <robot_vision/driver_camera.hpp>
#include <robot_vision/factory_camera.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <string>

/*! 
 *  \brief     SensorCamera
 *  \details   This class is a ROS canera interface.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    class SensorCamera
    {
        public:

            ///\brief Constructor.
            ///\param nh ROS node handle.
            ///\param np ROS private node handle.
            SensorCamera(ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Destructor.
            ~SensorCamera();

            ///\brief Get the name of the camera driver used.
            ///\return the name of the camera driver used.
            std::string name();

        protected:

            ///\brief Timer callback to publish images at a fixed rate.
            ///\param event Timer event.
            void timer_cb(const ros::TimerEvent& event);

            ros::Timer timer_; ///< ROS timer.

            image_transport::ImageTransport* it_; ///< ROS image transport.
            image_transport::Publisher pub_;      ///< ROS image publisher.

            int driver_id_;    ///< Camera driver ID.
            int device_id_;    ///< Device ID number.
            int fps_;          ///< Frames per second.
            int frame_width_;  ///< Frame width.
            int frame_height_; ///< Frame height.

            std::string image_topic_name_; ///< Name of the image ROS topic.
            double pub_rate_;              ///< Rate at which new frames are published on the ROS topic.

            DriverCamera* driver_camera_;  ///< Camera driver.
            FactoryCamera factory_camera_; ///< Camera factory.
    };
}

#endif // SENSOR_CAMERA_HPP
