#ifndef STATUS_HPP
#define STATUS_HPP

#include <ros/ros.h>

#include <string>

/*! 
 *  \brief     Status
 *  \details   This class subscribes to an actuator and outputs status 
 *             value in a more simple way.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class Status
    {
        public:

            typedef Status<T> ThisType; ///< This type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param status_name Name of the Status.
            Status(ros::NodeHandle& nh, std::string status_name = "/status");

            ///\brief Class destructor.
            ~Status();

            ///\brief Get the name of the Status.
            ///\return the name of the Status.
            std::string name() { return status_name_; }

            ///\brief Get the last valid data received from the actuator.
            ///\return the last valid data received from the actuator.
            const T& data() const { return data_; }

            ///\brief Get true if valid data are being received from the actuator.
            ///\return true valid data are being received from the actuator.
            bool ready() const { return ready_; }

        private:

            ///\brief Subscriber callback to update the Status data.
            void sub_cb(const boost::shared_ptr<const T>& msg);

            ros::Subscriber sub_;     ///< ROS subscriber.

            std::string status_name_; ///< ROS parameter - Name of the sensor's ROS topic.

            bool ready_;              ///< Flag indicating that the Status is receiving data.

            T data_;                  ///< Status data.
    };

    template <class T>
    Status<T>::Status(ros::NodeHandle& nh, std::string status_name)
    {
        status_name_ = status_name;
        sub_ = nh.subscribe(status_name_, 1, &ThisType::sub_cb, this);
        ready_ = false;
    }

    template <class T>
    Status<T>::~Status()
    {
    }

    template <class T>
    void Status<T>::sub_cb(const boost::shared_ptr<const T>& msg)
    {
        ready_ = true;
        data_ = *msg;
    }
}

#endif // STATUS_HPP
