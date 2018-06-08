#ifndef BEHAVIOR_CREATOR_IMPL_HPP
#define BEHAVIOR_CREATOR_IMPL_HPP

#include <ros/ros.h>

#include <robot_common/behavior_creator.hpp>

#include <string.h>

/*! 
 *  \brief     BehaviorCreatorImpl
 *  \details   This class instantiate a Behavior.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \remark    Inspired from http://blog.fourthwoods.com/2011/06/04/factory-design-pattern-in-c/
 *  \version   0.1
 *  \date      2018
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T, class T2>
    class BehaviorCreatorImpl: public BehaviorCreator<T>
    {
        public:

            ///\brief Constructor
            ///\param behavior_class_name Name of the Behavior class to create
            BehaviorCreatorImpl(const std::string& behavior_class_name);

            ///\brief Destructor
            virtual ~BehaviorCreatorImpl();

            ///\brief Create a Behavior instance
            virtual Behavior<T>* create(ros::NodeHandle& nh, ros::NodeHandle& np);
    };

    template <class T, class T2>
    BehaviorCreatorImpl<T,T2>::BehaviorCreatorImpl(const std::string& behavior_class_name) : BehaviorCreator<T>(behavior_class_name)
    {
        ROS_INFO("BehaviorCreatorImpl Constructor");
    }

    template <class T, class T2>
    BehaviorCreatorImpl<T,T2>::~BehaviorCreatorImpl()
    {
    }

    template <class T, class T2>
    Behavior<T>* BehaviorCreatorImpl<T,T2>::create(ros::NodeHandle& nh, ros::NodeHandle& np)
    {
        return new T2(nh, np);
    }
}

#endif // BEHAVIOR_CREATOR_IMPL_HPP
