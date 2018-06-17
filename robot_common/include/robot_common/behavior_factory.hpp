#ifndef BEHAVIOR_FACTORY_HPP
#define BEHAVIOR_FACTORY_HPP

#include <ros/ros.h>

#include <robot_common/behavior.hpp>

#include <map>
#include <string.h>

/*! 
 *  \brief     BehaviorFactory
 *  \details   This class creates Behaviors.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \remark    Inspired from http://blog.fourthwoods.com/2011/06/04/factory-design-pattern-in-c/
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class BehaviorFactory
    {
        public:

            ///\brief Class constructor.
            ///\param behavior_factory_name Name of the Behavior factory.
            BehaviorFactory(std::string behavior_factory_name = "/behavior_factory");

            ///\brief Class destructor.
            ~BehaviorFactory();

            ///\brief Set the factory name.
            ///\param behavior_factory_name Name of the Behavior factory.
            void setName(std::string& behavior_factory_name) { behavior_factory_name_ = behavior_factory_name; }

            ///\brief Get the name of the Behavior.
            ///\return the name of the Behavior.
            std::string name() { return behavior_factory_name_; }

            ///\brief Create a Behavior.
            ///\param behavior_name Name of the Behavior.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle. Default = nullptr.
            virtual Behavior<T>* create(std::string& behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np) = 0;

        private:

            std::string behavior_factory_name_;    ///< ROS parameter - Name of the BehaviorFactory.
    };

    template <class T>
    BehaviorFactory<T>::BehaviorFactory(std::string behavior_factory_name)
    {
        behavior_factory_name_ = behavior_factory_name;
    }

    template <class T>
    BehaviorFactory<T>::~BehaviorFactory()
    {
    }
}

#endif // BEHAVIOR_FACTORY_HPP
