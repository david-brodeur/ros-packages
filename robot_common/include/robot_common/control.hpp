#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <ros/ros.h>

#include <string>

/*! 
 *  \brief     Control
 *  \details   This class subscribes to a user input and outputs the value 
 *             in a more simple way.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>

    class Control
    {
        public:

            typedef Control<T> ThisType; ///< This type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param control_name Name of the Control ROS topic.
            Control(ros::NodeHandle& nh, std::string control_name = "/control");

            ///\brief Class destructor.
            ~Control();

            ///\brief Get the name of the Control.
            ///\return the name of the Control.
            std::string name() { return control_name_; }

            ///\brief Return the last valid data received from the Control.
            const T& data() const { return data_; }

            ///\brief Return true if the Control is ready.
            bool ready() const { return ready_; }

        private:

            ///\brief Subscriber callback to update the Control data.
            void sub_cb(const boost::shared_ptr<const T>& msg);

            ros::Subscriber sub_;      ///< ROS subscriber.

            std::string control_name_; ///< ROS parameter - Name of the user input ROS topic.

            bool ready_;               ///< Flag indicating that the Control is receiving data.

            T data_;                   ///< Control data.
    };

    template <class T>
    Control<T>::Control(ros::NodeHandle& nh, std::string control_name)
    {
        control_name_ = control_name;
        sub_ = nh.subscribe(control_name_, 1, &ThisType::sub_cb, this);
        ready_ = false;
    }

    template <class T>
    Control<T>::~Control()
    {
    }

    template <class T>
    void Control<T>::sub_cb(const boost::shared_ptr<const T>& msg)
    {
        ready_ = true;
        data_ = *msg;
    }
}

#endif // CONTROL_HPP
