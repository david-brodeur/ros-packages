#ifndef DETECTOR_HAAR_HPP
#define DETECTOR_HAAR_HPP

#include <ros/ros.h>

#include <robot_vision/algorithm_vision_haar.hpp>
#include <robot_perceptions/RegionOfInterestArray.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <string>

/*! 
 *  \brief     DetectorHaar
 *  \details   This class is a ROS interface for the Haar cascade classifier
 *             The detector takes an image as input and outputs a region of 
 *             interest (ROI).
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    class DetectorHaar
    {
        public:

            ///\brief Constructor.
            ///\param nh ROS node handle.
            ///\param np ROS private node handle.
            DetectorHaar(ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Destructor.
            ~DetectorHaar();

            ///\brief Get the name of the vision algorithm used.
            ///\return the name of the vision algorithm used.
            std::string name();

        private:

            ///\brief Subscriber's callback to process input images and outputs Haar detections.
            void sub_cb(const sensor_msgs::ImageConstPtr& msg);

            image_transport::ImageTransport* it_; ///< ROS image transport.
            image_transport::Subscriber sub_;     ///< ROS image subscriber.
            ros::Publisher pub_;                  ///< ROS region of interest publisher.

            std::string model_name_; ///< Name of the model file.
            double scale_factor_;    ///< Parameter specifying how much the image size is reduced at each image scale.
            int min_neighbors_;      ///< Parameter specifying how many neighbors each candidate rectangle should have to retain it.
            int flags_;              ///< Parameter with the same meaning for an old cascade as in the function cvHaarDetectObjects. It is not used for a new cascade.
            int min_;                ///< Minimum possible object size. Objects smaller than that are ignored.
            int max_;                ///< Maximum possible object size. Objects larger than that are ignored.

            std::string image_topic_name_; ///< Name of the image ROS topic.
            std::string roi_topic_name_;   ///< Name of the region of interest ROS topic.

            AlgorithmVisionHaar* algorithm_vision_haar_; ///< Haar algorithm.
            cv_bridge::CvImagePtr cv_ptr_;               ///< ROS to cv::Mat bridge
    };
}

#endif // DETECTOR_HAAR_HPP
