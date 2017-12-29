#ifndef BEHAVIOR_HPP
#define BEHAVIOR_HPP

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <stdlib.h>
#include <string.h>

/*! 
 *  \brief     Behavior
 *  \details   This class updates the value of a command to control an 
 *             actuator. A Behavior object can be activated or desactivated.
 *
 *             Derived classes need to reimplement the virtual function
 *             update().
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
    class Behavior
    {
        public:

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param behavior_name Name of the Behavior.
            Behavior(ros::NodeHandle& nh, std::string behavior_name = "/behavior", bool active = true);

            ///\brief Class destructor.
            ~Behavior();

            ///\brief Get the name of the Behavior.
            ///\return the name of the Behavior.
            std::string name() { return behavior_name_; }

            ///\brief Get true if the behavior is activated.
            ///\return true if the behavior is activated.
            bool active() { return active_; }

            ///\brief Activate the behavior.
            void activate() { active_ = true; }

            ///\brief Disable the behavior.
            void disable() { active_ = false; }

            ///\brief Initialize the behavior.
            virtual void init() = 0;

            ///\brief Reset the behavior.
            virtual void reset() = 0;

            ///\brief Updates the value of a command to control an actuator.
            ///\param command Command to be updated.
            virtual void update(T& command) = 0;

        private:

            ///\brief Service to activate or desactivate the behavior.
            bool activation_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

            ros::ServiceServer srv_;    ///< ROS server.

            std::string behavior_name_; ///< ROS parameter - Name of the Behavior.

            bool active_;               ///< Activation flag.
    };

    template <class T>
    Behavior<T>::Behavior(ros::NodeHandle& nh, std::string behavior_name, bool active)
    {
        behavior_name_ = behavior_name;
        srv_ = nh.advertiseService(behavior_name_ + "/activate", &Behavior::activation_cb, this);
        active_ = active;
    }

    template <class T>
    Behavior<T>::~Behavior()
    {
    }

    template <class T>
    bool Behavior<T>::activation_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        active_ = req.data;
        res.success = true;

        if (active())
        {
            res.message = "Behavior " + name() + " is activated.";
        }

        else
        {
            res.message = "Behavior " + name() + " is unactivated.";
        }

        return true;
    }
}


#endif // BEHAVIOR_HPP
