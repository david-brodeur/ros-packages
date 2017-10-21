#ifndef PERCEPTION_HPP
#define PERCEPTION_HPP

#include <ros/ros.h>

#include <string>

/*! 
 *  \brief     Perception
 *  \details   This class subscribes to a sensor or detector input and 
 *             outputs the value in a more simple way.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \author    François Ferland
 *  \remark    Based on code by François Ferland for course GEI740 at 
 *             Univerity of Sherbrooke
 *  \version   0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class Perception
    {
        public:

            typedef Perception<T> ThisType; ///< This type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param perception_name Name of the Perception.
            Perception(ros::NodeHandle& nh, std::string perception_name = "/perception");

            ///\brief Class destructor.
            ~Perception();

            ///\brief Get the name of the Perception.
            ///\return the name of the Perception.
            std::string name() { return perception_name_; }

            ///\brief Get the last valid data received from the sensor.
            ///\return the last valid data received from the sensor.
            const T& data() const { return data_; }

            ///\brief Get true if valid data are being received from the sensor.
            ///\return true valid data are being received from the sensor.
            bool ready() const { return ready_; }

        private:

            ///\brief Subscriber callback to update the Perception data.
            void sub_cb(const boost::shared_ptr<const T>& msg);

            ros::Subscriber sub_;         ///< ROS subscriber.

            std::string perception_name_; ///< ROS parameter - Name of the sensor's ROS topic.

            bool ready_;                  ///< Flag indicating that the Percetion is receiving data.

            T data_;                      ///< Perception data.
    };

    template <class T>
    Perception<T>::Perception(ros::NodeHandle& nh, std::string perception_name)
    {
        perception_name_ = perception_name;
        sub_ = nh.subscribe(perception_name_, 1, &ThisType::sub_cb, this);
        ready_ = false;
    }

    template <class T>
    Perception<T>::~Perception()
    {
    }

    template <class T>
    void Perception<T>::sub_cb(const boost::shared_ptr<const T>& msg)
    {
        ready_ = true;
        data_ = *msg;
    }
}

#endif // PERCEPTION_HPP
