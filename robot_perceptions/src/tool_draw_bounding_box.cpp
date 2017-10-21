#include <robot_vision/tool_draw_bounding_box.hpp>

using namespace robot_perceptions;

ToolDrawBoundingBox::ToolDrawBoundingBox(ros::NodeHandle& nh, ros::NodeHandle& np)
{
    np.param("image_topic_name", image_topic_name_, std::string("/sensor_camera/image_raw"));
    np.param("roi_topic_name", roi_topic_name_, std::string("/detector/roi"));
    np.param("tool_output_topic_name", tool_output_topic_name_, std::string("/tool/draw_bounding_box/image"));

    it_ = new image_transport::ImageTransport(nh);

    image_filter_.subscribe(nh, image_topic_name_, 1);
    roi_filter_.subscribe(nh, roi_topic_name_, 1);

    sync_ = new message_filters::Synchronizer<SyncPolicyType>(SyncPolicyType(10), image_filter_, roi_filter_);
    sync_->registerCallback(boost::bind(&ToolDrawBoundingBox::sub_cb, this, _1, _2));

    pub_ = it_->advertise(tool_output_topic_name_, 1);
}

ToolDrawBoundingBox::~ToolDrawBoundingBox()
{
    delete it_;
    delete sync_;
}

void ToolDrawBoundingBox::sub_cb(const sensor_msgs::ImageConstPtr& image, const robot_perceptions::RegionOfInterestArrayConstPtr& roi)
{
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg;

    try
    {
        if (sensor_msgs::image_encodings::isColor(image->encoding))
        {
            cv_ptr_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }

        else
        {
            cv_ptr_ = cv_bridge::toCvCopy(image, image->encoding);
        }
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    for (unsigned int i = 0; i < roi->roi.size(); i++)
    {
        cv::Rect rect(roi->roi[i].x_offset, roi->roi[i].y_offset, roi->roi[i].width, roi->roi[i].height);
        cv::rectangle(cv_ptr_->image, rect, cv::Scalar(255), 4, 8, 0);
    }

    header.stamp = image->header.stamp;
    header.frame_id = image->header.frame_id;

    if (sensor_msgs::image_encodings::isColor(image->encoding))
    {
        msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_ptr_->image).toImageMsg();
    }

    else
    {
        msg = cv_bridge::CvImage(header, image->encoding, cv_ptr_->image).toImageMsg();
    }

    pub_.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tool_draw_bounding_box");

    ros::NodeHandle nh, np("~");

    ToolDrawBoundingBox tool(nh, np);

    ros::spin();
}
