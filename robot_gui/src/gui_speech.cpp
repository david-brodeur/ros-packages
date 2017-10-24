#include <robot_gui/gui_speech.hpp>

using namespace robot_gui;

GuiSpeech::GuiSpeech(ros::NodeHandle& nh, ros::NodeHandle& np)
{
    np.param("gui_name", gui_name_, std::string("/gui/speech"));
    np.param("gui_logo", gui_logo_, std::string(":/images/demo_title.png"));

    np.param("image_topic_name", image_topic_name_, std::string("/sensor_camera/image_raw"));
    np.param("arbitration_name", arbitration_name_, std::string("/arbitration"));

    ParametersGuiMainWindow parameters;
    parameters.p_file_menu_name = "File";
    parameters.p_edit_menu_name = "Edit";
    parameters.p_view_menu_name = "View";
    parameters.p_param_menu_name = "Parameters";
    parameters.p_about_menu_name = "About";
    parameters.p_update_time_interval = 40;

    main_window_ = new GuiMainWindow(&parameters, gui_name_);

    display_camera_ = new GuiDisplayCamera();
    view_behaviors_ = new GuiViewBehaviors();

    main_window_->setLogo(gui_logo_);

    QIcon icon = main_window_->style()->standardIcon(QStyle::SP_ComputerIcon);
    main_window_->insertTabPage(0, display_camera_, "Camera", &icon);
    main_window_->insertTabPage(1, view_behaviors_, "Behaviors", &icon);

    main_window_->show();

    it_ = new image_transport::ImageTransport(nh);
    image_sub_ = it_->subscribe(image_topic_name_, 1, &GuiSpeech::image_cb, this);

    behavior_map_info_client_ = nh.serviceClient<robot_common::BehaviorMapInfo>(arbitration_name_ + "/list");

    behavior_update_timer_ = new QTimer(this);
    connect(behavior_update_timer_, SIGNAL(timeout()), this, SLOT(updateBehaviorMapInfo()));
    behavior_update_timer_->start(parameters.p_update_time_interval);
}

GuiSpeech::~GuiSpeech()
{
}

std::string GuiSpeech::name()
{
    if (main_window_ != NULL)
        return main_window_->name();
    else
        return "";
}

void GuiSpeech::updateBehaviorMapInfo()
{
    robot_common::BehaviorMapInfo srv;

    if (behavior_map_info_client_.call(srv))
    {
        view_behaviors_->setNames(srv.response.name);
        view_behaviors_->setPriorities(srv.response.priority);
        view_behaviors_->setActivations(srv.response.activation);
    }

    else
    {
        ROS_WARN("Failed to call service %s/list", arbitration_name_.c_str());
    }
}

void GuiSpeech::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    QImage image;

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

        cv::cvtColor(cv_ptr_->image, cv_ptr_->image, CV_BGR2RGB);
        image = QImage((const unsigned char*)(cv_ptr_->image.data), cv_ptr_->image.cols, cv_ptr_->image.rows, QImage::Format_RGB888);
        display_camera_->setImage(image);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gui_speech");

    ros::NodeHandle nh, np("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    QApplication app(argc, argv);

    GuiSpeech gui(nh, np);

    return app.exec();
}
