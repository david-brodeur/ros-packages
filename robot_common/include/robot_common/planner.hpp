#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <ros/ros.h>

#include <queue>
#include <string>

/*! 
 *  \brief     Planner
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
    class Planner
    {
        public:

            typedef std::queue<T> PlanType;

            ///\brief Class constructor.
            ///\param nh NodeHandle.
            ///\param planner_name Planner namespace.
            Planner(ros::NodeHandle& nh, std::string planner_name = "/planner");

            ///\brief Class destructor.
            ~Planner();

        protected:

            ///\brief Get the active goal.
            T* goal() { return &goal_; }

            ///\brief Get the plan.
            PlanType* plan() { return &plan_; }

            ///\brief Return true if the active goal is achieved.
            virtual bool achieved() =0;

            ///\brief Add a new goal to the goal list.
            virtual void add() =0;

        private:

            ///\brief Update periodically the actual goal to be achieved.
            void timer_cb(const ros::TimerEvent& event);

            ros::Timer timer_;   ///< Timer.

            ros::Publisher pub_; ///< ROS Publisher.

            PlanType plan_;      ///< Queue of goals to be performed.

            T goal_;             ///< Active goal.
    };

    template <class T>
    Planner<T>::Planner(ros::NodeHandle& nh, std::string planner_name)
    {
        pub_ = nh.advertise<T>(planner_name + "/goal", 1);
        timer_ = nh.createTimer(ros::Duration(0.1), &Planner::timer_cb, this);
    }

    template <class T>
    Planner<T>::~Planner()
    {
    }

    template <class T>
    void Planner<T>::timer_cb(const ros::TimerEvent& event)
    {
        if (achieved())
        {
            if (!plan_.empty())
            {
                goal_ = plan_.front();
                pub_.publish(goal_);
            }
        }
    }
}

#endif // PLANNER_HPP
