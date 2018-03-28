#ifndef BEHAVIOR_CREATOR_HPP
#define BEHAVIOR_CREATOR_HPP

#include <robot_common/behavior_factory2.hpp>

#include <string.h>

/*! 
 *  \brief     BehaviorCreator
 *  \details   This class is the base class to create a Behavior.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.1
 *  \date      2018
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class Behavior;

    template <class T>
    class BehaviorFactory2;

    template <class T>
    class BehaviorCreator
    {
        public:

            ///\brief Constructor
            ///\param behavior_class_name Name of the Behavior class to instantiate
            BehaviorCreator(const std::string& behavior_class_name);

            ///\brief Destructor
            virtual ~BehaviorCreator() {};

            ///\brief Create a class instance
            virtual Behavior<T>* create() = 0;
    };

    template <class T>
    BehaviorCreator<T>::BehaviorCreator(const std::string& behavior_class_name)
    {
        BehaviorFactory2<T>::registerit(behavior_class_name, this);
    }
}

#endif // BEHAVIOR_CREATOR_HPP
