#ifndef TOOL_DRAW_BOUNDING_BOX_HPP
#define TOOL_DRAW_BOUNDING_BOX_HPP

#include <ros/ros.h>

#include <robot_perceptions/RegionOfInterestArray.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>

/*! 
 *  \brief     ToolDrawBoundingBox
 *  \details   This class is a ROS interface to draw a bounding box on an image. 
               The tool takes an image and a ROI as input and outputs a new  
 *             image with the bounding box drawn.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_perceptions
{
    class ToolDrawBoundingBox
    {
        public:

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_perceptions::RegionOfInterestArray> SyncPolicyType;

            ///\brief Contructor
            ToolDrawBoundingBox(ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Destructor
            ~ToolDrawBoundingBox();

        private:

            ///\brief Subscriber's callback to draw a bounding box over input images where roi were detected.
            void sub_cb(const sensor_msgs::ImageConstPtr& image, const robot_perceptions::RegionOfInterestArrayConstPtr& roi);

            image_transport::ImageTransport* it_; ///< ROS image transport.

            message_filters::Subscriber<sensor_msgs::Image> image_filter_;                     ///< ROS image message filter.
            message_filters::Subscriber<robot_perceptions::RegionOfInterestArray> roi_filter_; ///< ROS roi message filter.
            message_filters::Synchronizer<SyncPolicyType>* sync_;                              ///< ROS message filter synchronization policy.

            image_transport::Publisher pub_;     ///< ROS output image publisher.

            std::string image_topic_name_;       ///< Name of the input image ROS topic.
            std::string roi_topic_name_;         ///< Name of the region of interest ROS topic.
            std::string tool_output_topic_name_; ///< Name of the output image ROS topic.

            cv_bridge::CvImagePtr cv_ptr_;       ///< ROS to cv::Mat bridge.
    };
}

#endif // TOOL_DRAW_BOUNDING_BOX_HPP
