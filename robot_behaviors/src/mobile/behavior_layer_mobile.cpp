#include <robot_behaviors/mobile/behavior_layer_mobile.hpp>

#include <robot_behaviors/mobile/behavior_factory_mobile.hpp>

using namespace robot_behaviors;

BehaviorLayerMobile::BehaviorLayerMobile(ros::NodeHandle& nh, ros::NodeHandle& np, std::string behavior_layer_name) : BaseType(nh, np, behavior_layer_name)
{
    factory_ = new BehaviorFactoryMobile();
}

BehaviorLayerMobile::~BehaviorLayerMobile()
{
    delete factory_;
}

int main(int argc, char** argv)
{
	int ret;

	ros::init(argc, argv, "behavior_layer_mobile");

	ros::NodeHandle nh, np("~");

	BehaviorLayerMobile behavior_layer(nh, np);

	ret = behavior_layer.init();

	if (ret != 0) {

		ROS_ERROR("Could not initialize behavior_layer_mobile");
		return ret;
	}

	ros::spin();
}
