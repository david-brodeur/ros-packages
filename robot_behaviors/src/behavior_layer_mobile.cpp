#include <robot_behaviors/behavior_layer_mobile.hpp>

#include <robot_behaviors/behavior_avoid_obstacles.hpp>
#include <robot_common/behavior_creator_impl.hpp>
using namespace robot_behaviors;

BehaviorLayerMobile::BehaviorLayerMobile(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_layer_name) : BaseType(nh, np, behavior_layer_name)
{
}

BehaviorLayerMobile::~BehaviorLayerMobile()
{
}

int main(int argc, char** argv)
{
	int ret;

	ros::init(argc, argv, "behavior_layer_mobile");

	ros::NodeHandle nh, np("~");

    robot_common::BehaviorCreatorImpl<geometry_msgs::Twist, BehaviorAvoidObstacles> creator("BehaviorAvoidObstacles");

	BehaviorLayerMobile behavior_layer(nh, np);

	ret = behavior_layer.init();

	if (ret != 0) {

		ROS_ERROR("Could not initialize behavior_layer_mobile");
		return ret;
	}

	ros::spin();
}
