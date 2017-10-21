#include <robot_vision/detector_hog.hpp>

using namespace robot_vision;

DetectorHOG::DetectorHOG(ros::NodeHandle& nh, ros::NodeHandle& np)
{
    int ret;

    np.param("hit_threshold", hit_threshold_, 0.3);
    np.param("win_stride", win_stride_, 8);
    np.param("padding", padding_, 0);
    np.param("scale_zero", scale_zero_, 1.05);
    np.param("group_threshold", group_threshold_, 4);

    np.param("image_topic_name", image_topic_name_, std::string("/sensor_camera/image_raw"));
    np.param("roi_topic_name", roi_topic_name_, std::string("/detector/hog/roi"));

    ParametersVisionHOG parameters;
    parameters.p_hit_threshold = hit_threshold_;
    parameters.p_win_stride = win_stride_;
    parameters.p_padding = padding_;
    parameters.p_scale_zero = scale_zero_;
    parameters.p_group_threshold = group_threshold_;

    algorithm_vision_hog_ = new AlgorithmVisionHOG(&parameters);
    algorithm_vision_hog_->init();

    if (ret != AlgorithmVision::SUCCESS)
    {
        ROS_ERROR("Failed to initialize the HOG algorithm");
    }

    it_ = new image_transport::ImageTransport(nh);
    sub_ = it_->subscribe(image_topic_name_, 1, &DetectorHOG::sub_cb, this);
    pub_ = nh.advertise<robot_perceptions::RegionOfInterestArray>(roi_topic_name_, 1);
}

DetectorHOG::~DetectorHOG()
{
    delete algorithm_vision_hog_;
    delete it_;
}

void DetectorHOG::sub_cb(const sensor_msgs::ImageConstPtr& msg)
{
    int ret;
    std_msgs::Header header;
    robot_perceptions::RegionOfInterestArray roi_msg;

    try
    {
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        else
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
        }
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    ret = algorithm_vision_hog_->process(cv_ptr_->image);

    if (ret != AlgorithmVision::SUCCESS)
    {
        ROS_ERROR("Failed to process the HOG algorithm");
    }

    header.stamp = msg->header.stamp;
    header.frame_id = msg->header.frame_id;

    roi_msg.header = header;

    if (algorithm_vision_hog_->getNFiltered() > 0)
    {        
        for (unsigned int i = 0; i < algorithm_vision_hog_->getNFiltered(); i++)
        {
            sensor_msgs::RegionOfInterest roi;
            roi.x_offset = algorithm_vision_hog_->getX(i);
            roi.y_offset = algorithm_vision_hog_->getY(i);
            roi.width = algorithm_vision_hog_->getWidth(i);
            roi.height = algorithm_vision_hog_->getHeight(i);
            roi.do_rectify = false;

            roi_msg.roi.push_back(roi);
        }
    }

    pub_.publish(roi_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detector_hog");

    ros::NodeHandle nh, np("~");

    DetectorHOG detector(nh, np);

    ros::spin();
}
