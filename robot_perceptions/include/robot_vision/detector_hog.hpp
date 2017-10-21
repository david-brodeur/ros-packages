#ifndef DETECTOR_HOG_HPP
#define DETECTOR_HOG_HPP

#include <ros/ros.h>

#include <robot_vision/algorithm_vision_hog.hpp>
#include <robot_perceptions/RegionOfInterestArray.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <string>

/*! 
 *  \brief     DetectorHOG
 *  \details   This class is a ROS interface for the histogram of gradient (HOG)
 *             algorithm. The detector takes an image as input and outputs a 
 *             region of interest (ROI).
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    class DetectorHOG
    {
        public:

            ///\brief Constructor.
            ///\param nh ROS node handle.
            ///\param np ROS private node handle.
            DetectorHOG(ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Destructor.
            ~DetectorHOG();

            ///\brief Get the name of the vision algorithm used.
            ///\return the name of the vision algorithm used.
            std::string name();

        private:

            ///\brief Subscriber's callback to process input images and outputs HOG detections.
            void sub_cb(const sensor_msgs::ImageConstPtr& msg);

            image_transport::ImageTransport* it_; ///< ROS image transport.
            image_transport::Subscriber sub_;     ///< ROS image subscriber.
            ros::Publisher pub_;                  ///< ROS region of interest publisher.

            double hit_threshold_; ///< Threshold for the distance between features and SVM classifying plane.
            int win_stride_;       ///< Window stride. It must be a multiple of block stride.
            int padding_;          ///< Mock parameter to keep the CPU interface compatibility. It must be (0,0).
            double scale_zero_;    ///< Coefficient of the detection window increase.
            int group_threshold_;  ///< Coefficient to regulate the similarity threshold. 0 means not to perform grouping.

            std::string image_topic_name_; ///< Name of the image ROS topic.
            std::string roi_topic_name_;   ///< Name of the region of interest ROS topic.

            AlgorithmVisionHOG* algorithm_vision_hog_; ///< HOG algorithm.
            cv_bridge::CvImagePtr cv_ptr_;             ///< ROS to cv::Mat bridge.
    };
}

#endif // DETECTOR_HOG_HPP
