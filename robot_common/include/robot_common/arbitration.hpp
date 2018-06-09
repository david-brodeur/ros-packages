#ifndef ARBITRATION_HPP
#define ARBITRATION_HPP

#include <ros/ros.h>

#include <robot_common/behavior.hpp>
#include <robot_common/BehaviorMapInfo.h>

#include <map>

/*! 
 *  \brief     Arbitration
 *  \details   This class keeps a list of Behavior objects requesting control 
 *             of the same actuator and selects which one is allowed to send 
 *             a command based on priorities.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \author    François Ferland
 *  \remark    Based on code by François Ferland for course GEI740 at 
 *             Univerity of Sherbrooke
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class Arbitration
    {
        public:

            typedef std::map<int, Behavior<T>*> BehaviorMap; ///< BehaviorMap type definition.

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param command_topic_name Name of the command sent to the actuator.
            ///\param rate Rate at which the command is updated.
            Arbitration(ros::NodeHandle& nh, std::string command_topic_name = "/cmd", std::string arbitration_name = "/arbitration", float rate = 10.0);

            ///\brief Class destructor.
            ~Arbitration();

            ///\brief Get the name of the Arbitration.
            ///\return the name of the Arbitration.
            std::string name() { return arbitration_name_; }

            ///\brief Get the number of Behaviors handled by the Arbitration.
            ///\return the number of Behaviors handled by the Arbitration.
            unsigned int size() { return map_.size(); }

            ///\brief Reset the behavior.
            void reset();

            ///\brief Add a Behavior to the list.
            ///\param p Priority. A higher number value means a higher priority.
            ///\param b Behavior instance pointer.
            void add(int p, Behavior<T>* b);

            ///\brief Get the command.
            ///\return the command.
            const T& command() const { return command_; }

        private:

            ///\brief Service to get the list of Behaviors with priorities and activation info.
            bool list_cb(robot_common::BehaviorMapInfo::Request& req, robot_common::BehaviorMapInfo::Response& res);

            ///\brief Timer callback to update at a constant rate the command sent to the actuator.
            void timer_cb(const ros::TimerEvent& event);

            ros::Timer timer_;             ///< Timer.

            ros::Publisher pub_;           ///< ROS publisher.

            ros::ServiceServer srv_;       ///< ROS server.

            std::string arbitration_name_; ///< ROS parameter - Name of the Arbitration.

            T command_;                    ///< Command sent to the actuator.

            BehaviorMap map_;              ///< Map of Behaviors and priority values.
    };

    template <class T>
    Arbitration<T>::Arbitration(ros::NodeHandle& nh, std::string command_topic_name, std::string arbitration_name, float rate)
    {
        arbitration_name_ = arbitration_name;

        pub_ = nh.advertise<T>(command_topic_name, 1);
        srv_ = nh.advertiseService(arbitration_name + "/list", &Arbitration::list_cb, this);
        timer_ = nh.createTimer(ros::Duration(1/rate), &Arbitration::timer_cb, this);
    }

    template <class T>
    Arbitration<T>::~Arbitration()
    {
        reset();
    }

    template <class T>
    void Arbitration<T>::reset()
    {
        typedef typename BehaviorMap::iterator it;

        for (it i = map_.begin(); i != map_.end(); ++i)
        {
            delete i->second;
        }
    }

    template <class T>
    void Arbitration<T>::add(int p, Behavior<T>* b)
    {
        map_[p] = b;
    }

    template <class T>
    bool Arbitration<T>::list_cb(robot_common::BehaviorMapInfo::Request& req, robot_common::BehaviorMapInfo::Response& res)
    {
        typedef typename BehaviorMap::iterator it;

        for (it i = map_.begin(); i != map_.end(); ++i)
        {
            res.name.push_back(i->second->name());
            res.priority.push_back(i->first);
            res.activation.push_back(i->second->active());
        }

        return true;
    }

    template <class T>
    void Arbitration<T>::timer_cb(const ros::TimerEvent& event)
    {
        typedef typename BehaviorMap::iterator it;

        for (it i = map_.begin(); i != map_.end(); ++i)
        {
            if (i->second->active())
            {
                i->second->update(command_);
            }
        }

        pub_.publish(command_);
    }
}

#endif // ARBITRATION_HPP
