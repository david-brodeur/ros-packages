#ifndef BEHAVIOR_FACTORY2_HPP
#define BEHAVIOR_FACTORY2_HPP

#include <ros/ros.h>

#include <robot_common/behavior_creator.hpp>
#include <robot_common/behavior_creator_impl.hpp>


#include <map>
#include <string.h>

/*! 
 *  \brief     BehaviorFactory2
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

    template <class T>
    class BehaviorFactory2
    {
        public:

            ///\brief Set the factory name
            static void setName(std::string& behavior_factory_name) { behavior_factory_name_ = behavior_factory_name; }

            ///\brief Get the name of the Behavior.
            ///\return the name of the Behavior.
            static std::string name() { return behavior_factory_name_; }

            ///\brief Create a Behavior.
            ///\param behavior_name Name of the Behavior.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle. Default = NULL.
            static Behavior<T>* create(std::string& behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Register a Behavior class.
            ///\param behavior_class_name Name of the Behavior derived class.
            static void registerit(const std::string& behavior_class_name, BehaviorCreator<T>* creator);

        private:

            static std::string behavior_factory_name_;    ///< ROS parameter - Name of the BehaviorFactory.
            static std::map<std::string, BehaviorCreator<T>*>& get_table();

    };

    template <class T>
    Behavior<T>* BehaviorFactory2<T>::create(std::string& behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np)
    {
        typename std::map<std::string, BehaviorCreator<T>*>::iterator i;
        i = get_table().find(behavior_name);

        if (i != get_table().end())
            return i->second()->create();
        else
            return (Behavior<T>*) nullptr;
    }

    template <class T>
    void BehaviorFactory2<T>::registerit(const std::string& behavior_class_name, BehaviorCreator<T>* creator)
    {
        get_table[behavior_class_name] = creator;
    }

    template <class T>
    std::map<std::string, BehaviorCreator<T>*>& BehaviorFactory2<T>::get_table()
    {
        static std::map<std::string, BehaviorCreator<T>*> table;
        return table;
    }


    #define REGISTER(behavior_class_name) \
        private: \
        static const CreatorImpl<behavior_class_name> creator;

    #define REGISTERIMPL(behavior_class_name) \
        const CreatorImpl<behavior_class_name> behavior_class::creator(#behavior_class_name);
}

#endif // BEHAVIOR_FACTORY2_HPP
