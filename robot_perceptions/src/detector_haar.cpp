#include <robot_vision/detector_haar.hpp>

using namespace robot_vision;

DetectorHaar::DetectorHaar(ros::NodeHandle& nh, ros::NodeHandle& np)
{
    int ret;

    np.param("model_name", model_name_, std::string("haarcascade_frontalface_alt.xml"));
    np.param("scale_factor", scale_factor_, 1.5);
    np.param("min_neighbors", min_neighbors_, 2);
    np.param("flags", flags_, 0);
    np.param("min", min_, 30);
    np.param("max", max_, 200);

    np.param("image_topic_name", image_topic_name_, std::string("/sensor_camera/image_raw"));
    np.param("roi_topic_name", roi_topic_name_, std::string("/detector/haar/roi"));

    ParametersVisionHaar parameters;
    parameters.p_model_name = model_name_;
    parameters.p_scale_factor = scale_factor_;
    parameters.p_min_neighbors = min_neighbors_;
    parameters.p_flags = flags_;
    parameters.p_min = min_;
    parameters.p_max = max_;

    algorithm_vision_haar_ = new AlgorithmVisionHaar(parameters);
    ret = algorithm_vision_haar_->init();

    if (ret != AlgorithmVision::SUCCESS)
    {
        ROS_ERROR("Failed to initialize the Haar algorithm");
    }

    it_ = new image_transport::ImageTransport(nh);
    sub_ = it_->subscribe(image_topic_name_, 1, &DetectorHaar::sub_cb, this);
    pub_ = nh.advertise<robot_perceptions::RegionOfInterestArray>(roi_topic_name_, 1);
}

DetectorHaar::~DetectorHaar()
{
    delete algorithm_vision_haar_;
    delete it_;
}

void DetectorHaar::sub_cb(const sensor_msgs::ImageConstPtr& msg)
{
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

    algorithm_vision_haar_->process(cv_ptr_->image);

    header.stamp = msg->header.stamp;
    header.frame_id = msg->header.frame_id;

    roi_msg.header = header;

    if (algorithm_vision_haar_->getNFiltered() > 0)
    {        
        for (unsigned int i = 0; i < algorithm_vision_haar_->getNFiltered(); i++)
        {
            sensor_msgs::RegionOfInterest roi;
            roi.x_offset = algorithm_vision_haar_->getX(i);
            roi.y_offset = algorithm_vision_haar_->getY(i);
            roi.width = algorithm_vision_haar_->getWidth(i);
            roi.height = algorithm_vision_haar_->getHeight(i);
            roi.do_rectify = false;

            roi_msg.roi.push_back(roi);
        }
    }

    pub_.publish(roi_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detector_haar");

    ros::NodeHandle nh, np("~");

    DetectorHaar detector(nh, np);

    ros::spin();
}
