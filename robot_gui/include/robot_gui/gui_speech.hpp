#ifndef GUI_SPEECH_HPP
#define GUI_SPEECH_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <robot_common/BehaviorMapInfo.h>

#include <robot_gui/gui_mainwindow.hpp>
#include <robot_gui/gui_view_behaviors.hpp>
#include <robot_gui/gui_view_perceptions.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <QApplication>
#include <QIcon>
#include <QObject>
#include <QStyle>
#include <QTimer>
#endif

/*! 
 *  \brief     GuiSpeech
 *  \details   This class is a ROS based GUI application for 
 *             robotic speech interactions.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_gui
{
    class GuiSpeech : public QObject
    {
        Q_OBJECT

        public:

            ///\brief Constructor.
            ///\param nh ROS node handle.
            ///\param np ROS private node handle.
            GuiSpeech(ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Destructor.
            ~GuiSpeech();

            ///\brief Get the name of the GUI application.
            ///\return the name of the GUI application.
            std::string name();

            ///\brief Initialize the GUI.
            void init();

            ///\brief Reset the GUI.
            void reset();

        private Q_SLOTS:

            ///\brief Update Behavior Map Info.
            void updateBehaviorMapInfo();

        private:

            ///\brief Subscriber's callback to process input images.
            void image_cb(const sensor_msgs::ImageConstPtr& msg);

            std::string gui_name_;         ///< Name of the GUI application.
            std::string gui_logo_;         ///< Path to the logo file.
            std::string image_topic_name_; ///< Name of the image ROS topic.
            std::string arbitration_name_; ///< Name of the arbitration module.

            cv_bridge::CvImagePtr cv_ptr_;          ///< ROS to cv::Mat bridge.
            image_transport::ImageTransport* it_;   ///< ROS image transport.
            image_transport::Subscriber image_sub_; ///< ROS image subscriber.

            std::vector<ros::ServiceClient*> behavior_activation_client_;    ///< ROS clients to set behaviors activation.
            ros::ServiceClient behavior_map_info_client_;                    ///< ROS client to get information from the behaviors map.

            QTimer* behavior_update_timer_; /// < Timer to update the Behavior architecture info.

            GuiMainWindow* main_window_;           ///< Qt mainwindow.
            GuiViewPerceptions* view_perceptions_; ///< Perceptions Qt view.
            GuiViewBehaviors* view_behaviors_;     ///< Behaviors Qt view.
    };
}

#endif // GUI_SPEECH_HPP
