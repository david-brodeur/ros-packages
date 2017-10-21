#ifndef BEHAVIOR_LAYER_HPP
#define BEHAVIOR_LAYER_HPP

#include <ros/ros.h>

#include <robot_common/arbitration.hpp>
#include <robot_common/behavior.hpp>
#include <robot_common/behavior_factory.hpp>

#include <stdlib.h>
#include <string.h>

/*! 
 *  \brief     BehaviorLayer
 *  \details   This class provides a complete behavior-based 
 *             layer for a robotic application.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_common
{
    template <class T>
    class BehaviorLayer
    {
        public:

            ///\brief Class constructor.
            ///\param nh ROS node handle.
            ///\param nh ROS private node handle.
            ///\param behavior_name Name of the Behavior. Default = "/behavior_layer".
            BehaviorLayer(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_layer_name = "/behavior_layer");

            ///\brief Class destructor.
            ~BehaviorLayer();

            ///\brief Get the name of the Behavior.
            ///\return the name of the Behavior.
            std::string name() { return behavior_layer_name_; }

            ///\brief Initialize the Behaviors map.
            ///\return error value.
            int init();

            ///\brief Reset the BehaviorLayer
            void reset();

        protected:

            BehaviorFactory<T>* factory_; ///< BehaviorFactory module.

        private:

            ros::NodeHandle nh_;                      ///< ROS node handle reference.
            ros::NodeHandle np_;                      ///< ROS private node handle reference.

            std::string behavior_layer_name_;         ///< ROS parameter - Name of the BehaviorLayer.

            std::string command_topic_name_;          ///< ROS parameter - Name of the command ROS topic.
            std::string arbitration_name_;            ///< ROS parameter - Name of the arbitration module.
            double arbitration_rate_;                 ///< ROS parameter - Rate at which commands are published.

            std::vector<std::string> behavior_names_; ///< ROS parameter - Map of Behavior names.
            std::vector<int> behavior_priorities_;    ///< ROS parameter - Map of Behavior priorities.

            Arbitration<T>* arbitration_;             ///< Arbitration module.
    };

    template <class T>
    BehaviorLayer<T>::BehaviorLayer(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_layer_name)
    {
        // Set the name of the BehaviorLayer.
        behavior_layer_name_ = behavior_layer_name;

        // Keep a reference to ROS node handles.
        nh_ = nh;
        np_ = np;

        // Get Arbitration parameters.
        np.param("command_topic_name", command_topic_name_, std::string("/cmd"));
        np.param("arbitration_name", arbitration_name_, std::string("/arbitration"));
        np.param("arbitration_rate", arbitration_rate_, 10.0);

        np.getParam("behavior_names", behavior_names_);
        np.getParam("behavior_priorities", behavior_priorities_);

        arbitration_ = NULL;
    }

    template <class T>
    BehaviorLayer<T>::~BehaviorLayer()
    {
        // Delete the Arbitration module.
        if (arbitration_ != NULL)
        {
            delete arbitration_;
        }
    }

    template <class T>
    void BehaviorLayer<T>::init()
    {
        reset();

        if (factory_ == NULL)
        {
            ROS_ERROR("No BehaviorFactory available.");
            return 1;
        }

        // Create all Behaviors for the layer
        for (unsigned int i = 0; i < behavior_names_.size(); i++)
        {
            Behavior<T>* behavior = factory_->create(behavior_names_[i], nh_, np_);

            if (behavior)
            {
                arbitration_->add(behavior_priorities_[i], behavior);
            }

            else
            {
                ROS_ERROR("Unable to create behavior %s", behavior_names_[i].c_str());
            }
        }

        return 0;
    }

    template <class T>
    int BehaviorLayer<T>::reset()
    {
        if (arbitration_ != NULL)
        {
            delete arbitration_;
        }

        arbitration_ = new robot_common::Arbitration<T>(nh_, command_topic_name_, arbitration_name_, arbitration_rate_);
    }
}

#endif // BEHAVIOR_LAYER_HPP
