#include <robot_gui/gui_speech.hpp>

#include <std_srvs/SetBool.h>

#include <QPushButton>

using namespace robot_gui;

GuiSpeech::GuiSpeech(ros::NodeHandle& nh, ros::NodeHandle& np)
{
    np.param("gui_name", gui_name_, std::string("/gui/speech"));
    np.param("gui_logo", gui_logo_, std::string(":/images/demo_title.png"));

    np.param("image_topic_name", image_topic_name_, std::string("/sensor_camera/image_raw"));
    np.param("arbitration_name", arbitration_name_, std::string("/arbitration"));

    it_ = new image_transport::ImageTransport(nh);
    image_sub_ = it_->subscribe(image_topic_name_, 1, &GuiSpeech::image_cb, this);

    behavior_map_info_client_ = nh.serviceClient<robot_common::BehaviorMapInfo>(arbitration_name_ + "/list");

    robot_common::BehaviorMapInfo srv;
    std::vector<std::string>::iterator it;

    if (behavior_map_info_client_.call(srv))
    {
        for (unsigned int iBehavior = 0; iBehavior < srv.response.name.size(); iBehavior++)
        {
            behavior_activation_clients_.push_back(nh.serviceClient<std_srvs::SetBool>(srv.response.name[iBehavior] + "/activate"));
        }
    }

    else
    {
        ROS_WARN("Failed to call service %s/list", arbitration_name_.c_str());
    }

}

GuiSpeech::~GuiSpeech()
{
    reset();

    delete it_;
}

std::string GuiSpeech::name()
{
    if (main_window_ != NULL)
        return main_window_->name();
    else
        return "";
}

void GuiSpeech::init()
{
    reset();

    ParametersGuiMainWindow parameters;
    parameters.p_file_menu_name = "File";
    parameters.p_edit_menu_name = "Edit";
    parameters.p_view_menu_name = "View";
    parameters.p_param_menu_name = "Parameters";
    parameters.p_about_menu_name = "About";
    parameters.p_update_time_interval = 40;

    main_window_ = new GuiMainWindow(gui_name_);
    main_window_->init(&parameters);
    main_window_->setLogo(gui_logo_);

    view_perceptions_ = new GuiViewPerceptions();
    view_behaviors_ = new GuiViewBehaviors();
    updateBehaviorMapInfo();
    view_behaviors_->init();

    std::vector<QPushButton*>::iterator it;

    for (it = view_behaviors_->pushbuttons().begin(); it != view_behaviors_->pushbuttons().end(); it++)
    {
        connect(*it, SIGNAL(clicked(bool)), this, SLOT(setBehaviorActivation(bool)));
    }

    QIcon icon = main_window_->style()->standardIcon(QStyle::SP_ComputerIcon);
    main_window_->insertTabPage(0, view_perceptions_, "Perceptions", &icon);
    main_window_->insertTabPage(1, view_behaviors_, "Behaviors", &icon);

    main_window_->show();

    behavior_update_timer_ = new QTimer(this);
    connect(behavior_update_timer_, SIGNAL(timeout()), this, SLOT(updateBehaviorMapInfo()));
    behavior_update_timer_->start(parameters.p_update_time_interval);
}

void GuiSpeech::reset()
{
}

void GuiSpeech::setBehaviorActivation(bool activate)
{
    int index;
    std_srvs::SetBool srv;
    srv.request.data = !activate;

    QPushButton* pushbutton = qobject_cast<QPushButton*>(QObject::sender());

    std::vector<QPushButton*>::iterator it;

    for (it = view_behaviors_->pushbuttons().begin(); it != view_behaviors_->pushbuttons().end(); it++)
    {
        if (*it == pushbutton)
        {
            index = std::distance(view_behaviors_->pushbuttons().begin(), it);
            behavior_activation_clients_[index].call(srv);
            break;
        }
    }   
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
        view_perceptions_->setImage(image);
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
    gui.init();

    return app.exec();
}
