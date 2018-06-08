#ifndef BEHAVIOR_FACTORY_HPP
#define BEHAVIOR_FACTORY_HPP

#include <ros/ros.h>

#include <robot_common/behavior_creator.hpp>
#include <robot_common/behavior_creator_impl.hpp>

#include <map>
#include <string.h>

/*! 
 *  \brief     BehaviorFactory
 *  \details   This class creates Behaviors.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class Behavior;

    template <class T>
    class BehaviorCreator;

    template <class T, class T2>
    class BehaviorCreatorImpl;

    template <class T>
    class BehaviorFactory
    {
        public:

            typedef std::map<std::string, BehaviorCreator<T>*> BehaviorCreatorMap; ///< BehaviorCreatorMap type definition.

            ///\brief Set the factory name
            static void setName(std::string& behavior_factory_name) { behavior_factory_name_ = behavior_factory_name; }

            ///\brief Get the name of the Behavior.
            ///\return the name of the Behavior.
            static std::string name() { return behavior_factory_name_; }

            ///\brief Create a Behavior.
            ///\param behavior_name Name of the Behavior.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle. Default = nullptr.
            static Behavior<T>* create(std::string& behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np = nullptr);

            ///\brief Register a Behavior class.
            ///\param behavior_class_name Name of the Behavior derived class.
            static void registerit(const std::string& behavior_class_name, BehaviorCreator<T>* creator);

        private:

            static std::string behavior_factory_name_;    ///< ROS parameter - Name of the BehaviorFactory.

            ///\brief Get the BehaviorCreator map
            static BehaviorCreatorMap& get_map();

    };

    template <class T>
    Behavior<T>* BehaviorFactory<T>::create(std::string& behavior_class_name, ros::NodeHandle& nh, ros::NodeHandle& np)
    {
        typename BehaviorCreatorMap::iterator itr = get_map().find(behavior_class_name);

        if (itr != get_map().end())
            return itr->second->create(nh, np);
        else
        {
            ROS_WARN("%s is not a valid behavior class name.", behavior_class_name.c_str());
            return (Behavior<T>*) nullptr;
        }
    }

    template <class T>
    void BehaviorFactory<T>::registerit(const std::string& behavior_class_name, BehaviorCreator<T>* creator)
    {
        ROS_INFO("Registering %s", behavior_class_name.c_str());
        get_map()[behavior_class_name] = creator;
    }

    template <class T>
    std::map<std::string, BehaviorCreator<T>*>& BehaviorFactory<T>::get_map()
    {
        static BehaviorCreatorMap creator_map;
        return creator_map;
    }


    #define REGISTER(behavior_msg_type, behavior_class_name) \
        private: \
        static const robot_common::BehaviorCreatorImpl<behavior_msg_type, behavior_class_name> creator;

    #define REGISTERIMPL(behavior_msg_type, behavior_class_name) \
        const robot_common::BehaviorCreatorImpl<behavior_msg_type, behavior_class_name> behavior_class_name::creator(#behavior_class_name);
}

#endif // BEHAVIOR_FACTORY_HPP
