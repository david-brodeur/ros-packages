#include <robot_vision/sensor_camera.hpp>

using namespace robot_vision;

SensorCamera::SensorCamera(ros::NodeHandle& nh, ros::NodeHandle& np)
{
    int ret;

    np.param("driver_id", driver_id_, 0);
    np.param("device_id", device_id_, 0);
    np.param("fps", fps_, 25);
    np.param("frame_width", frame_width_, 640);
    np.param("frame_height", frame_height_, 480);

    np.param("image_topic_name", image_topic_name_, std::string("/sensor_camera/image_raw"));
    np.param("pub_rate", pub_rate_, 25.0);

    ParametersCamera parameters;
    parameters.p_device_id = device_id_;
    parameters.p_fps = fps_;
    parameters.p_frame_width = (unsigned int) frame_width_;
    parameters.p_frame_height = (unsigned int) frame_height_;

    driver_camera_ = factory_camera_.create(static_cast<FactoryCamera::DriverCameraID>(driver_id_), &parameters);
    ret = driver_camera_->open();

    if (ret != DriverCamera::SUCCESS)
    {
        if (ret == DriverCamera::SET_PARAMETER_FAILED)
        {
            ROS_WARN("%s", DriverCamera::getErrorMessage(ret));
            ROS_WARN("Using default parameters instead");
        }

        else
        {
            ROS_ERROR("%s", DriverCamera::getErrorMessage(ret));
        }
    }

    it_ = new image_transport::ImageTransport(nh);
    pub_ = it_->advertise(image_topic_name_, 1);

    timer_ = nh.createTimer(ros::Duration(1 / pub_rate_), &SensorCamera::timer_cb, this);
}

SensorCamera::~SensorCamera()
{
    delete driver_camera_;
    delete it_;
}

std::string SensorCamera::name()
{
    if (driver_camera_ != NULL)
    {
        return driver_camera_->name();
    }

    else
    {
        return "";
    }
}

void SensorCamera::timer_cb(const ros::TimerEvent&)
{
    int ret;
    cv::Mat frame;

    std_msgs::Header header;
    sensor_msgs::ImagePtr msg;

    ret = driver_camera_->getFrame(frame);

    if (ret != DriverCamera::SUCCESS)
    {
        ROS_ERROR("%s", DriverCamera::getErrorMessage(ret));
    }

    else
    {
        header.stamp = ros::Time::now();
        header.frame_id = "sensor_camera_frame";

        msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        pub_.publish(msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_camera");

    ros::NodeHandle nh, np("~");

    SensorCamera sensor(nh, np);

    ros::spin();
}
