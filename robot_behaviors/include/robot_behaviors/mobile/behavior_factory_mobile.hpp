#ifndef BEHAVIOR_FACTORY_MOBILE_HPP
#define BEHAVIOR_FACTORY_MOBILE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <robot_common/behavior.hpp>
#include <robot_common/behavior_factory.hpp>

#include <map>
#include <string.h>

/*! 
 *  \brief     BehaviorFactoryMobile
 *  \details   This class creates mobile platform Behaviors (geometry_msgs::Twist).
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \remark    Inspired from http://blog.fourthwoods.com/2011/06/04/factory-design-pattern-in-c/
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_behaviors
{
    class BehaviorFactoryMobile;

    typedef robot_common::BehaviorFactory<geometry_msgs::Twist> BaseType; ///< Base type definition.
    typedef robot_common::Behavior<geometry_msgs::Twist> BehaviorType; ///< Behavior class type definition.

    class BehaviorCreatorMobile
    {
        public:

            ///\brief Constructor
            ///\param behavior_class_name Name of the Behavior class to create
            BehaviorCreatorMobile(const std::string& behavior_class_name);

            ///\brief Destructor
            virtual ~BehaviorCreatorMobile();

            ///\brief Create a behavior instance
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle. Default = nullptr.
            virtual BehaviorType* create(ros::NodeHandle& nh, ros::NodeHandle& np) = 0;
    };

    template <class T>
    class BehaviorCreatorImplMobile: public BehaviorCreatorMobile
    {
        public:

            ///\brief Constructor
            ///\param behavior_class_name Name of the Behavior class to create
            BehaviorCreatorImplMobile(const std::string& behavior_class_name);

            ///\brief Destructor
            virtual ~BehaviorCreatorImplMobile();

            ///\brief Create a Behavior instance
            virtual BehaviorType* create(ros::NodeHandle& nh, ros::NodeHandle& np);
    };

    template <class T>
    BehaviorCreatorImplMobile<T>::BehaviorCreatorImplMobile(const std::string& behavior_class_name) : BehaviorCreatorMobile(behavior_class_name)
    {
    }

    template <class T>
    BehaviorCreatorImplMobile<T>::~BehaviorCreatorImplMobile()
    {
    }

    template <class T>
    BehaviorType* BehaviorCreatorImplMobile<T>::create(ros::NodeHandle& nh, ros::NodeHandle& np)
    {
        return new T(nh, np);
    }

    class BehaviorFactoryMobile : public robot_common::BehaviorFactory<geometry_msgs::Twist>
    {
        public:

            typedef std::map<std::string, BehaviorCreatorMobile*> BehaviorCreatorMap; ///< BehaviorCreatorMap type definition.

            ///\brief Class constructor.
            ///\param behavior_factory_name Name of the Behavior factory.
            BehaviorFactoryMobile(std::string behavior_factory_name = "/behavior_factory/mobile");

            ///\brief Class destructor.
            ~BehaviorFactoryMobile();

            ///\brief Create a Behavior.
            ///\param behavior_name Name of the Behavior.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle. Default = nullptr.
            BehaviorType* create(std::string& behavior_name, ros::NodeHandle& nh, ros::NodeHandle& np);

            ///\brief Register a Behavior class.
            ///\param behavior_class_name Name of the Behavior derived class.
            static void registerit(const std::string& behavior_class_name, BehaviorCreatorMobile* creator);

        private:

            ///\brief Get the BehaviorCreator map
            static BehaviorCreatorMap& get_map();
    };

    #define REGISTER_BEHAVIOR_MOBILE(behavior_class_name) \
        private: \
        static const robot_behaviors::BehaviorCreatorImplMobile<behavior_class_name> creator;

    #define REGISTERIMPL_BEHAVIOR_MOBILE(behavior_class_name) \
        const robot_behaviors::BehaviorCreatorImplMobile<behavior_class_name> behavior_class_name::creator(#behavior_class_name);
}

#endif // BEHAVIOR_FACTORY_MOBILE_HPP
