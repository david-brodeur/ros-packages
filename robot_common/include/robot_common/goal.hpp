#ifndef GOAL_HPP
#define GOAL_HPP

#include <ros/ros.h>

#include <string>

/*! 
 *  \brief     Goal
 *  \details   This class subscribes to a planner goal input and 
 *             outputs the value in a more simple way.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class Goal
    {
        public:

            typedef Goal<T> ThisType;    ///< This type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param goal_name Name of the Goal.
            Goal(ros::NodeHandle& nh, std::string goal_name = "/goal");

            ///\brief Class destructor.
            ~Goal();

            ///\brief Get the name of the Goal.
            ///\return the name of the Goal.
            std::string name() { return goal_name_; }

            ///\brief Get the last goal from the planner.
            ///\return the last goal from the planner.
            const T& goal();

            ///\brief Get true if a new goal has been received.
            ///\return true if a new goal has been received.
            bool ready() const { return ready_; }

        private:

            ///\brief Subscriber callback to update the Goal data.
            void sub_cb(const boost::shared_ptr<const T>& msg);

            ros::Subscriber sub_;   ///< ROS subscriber.

            std::string goal_name_; ///< ROS parameter - Name of the sensor's ROS topic.

            bool ready_;            ///< Flag indicating that the Percetion is receiving data.

            T data_;                ///< Goal data.
    };

    template <class T>
    Goal<T>::Goal(ros::NodeHandle& nh, std::string goal_name)
    {
        goal_name_ = goal_name;
        sub_ = nh.subscribe(goal_name_, 1, &ThisType::sub_cb, this);
        ready_ = false;
    }

    template <class T>
    Goal<T>::~Goal()
    {
    }

    template <class T>
    const T& Goal<T>::goal()
    {
        ready_ = false;
        return data_;
    }

    template <class T>
    void Goal<T>::sub_cb(const boost::shared_ptr<const T>& msg)
    {
        ready_ = true;
        data_ = *msg;
    }
}

#endif // GOAL_HPP
