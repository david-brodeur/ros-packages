#include <robot_behaviors/mobile/behavior_factory_mobile.hpp>

using namespace robot_behaviors;

BehaviorCreatorMobile::BehaviorCreatorMobile(const std::string& behavior_class_name)
{
    BehaviorFactoryMobile::registerit(behavior_class_name, this);
}

BehaviorCreatorMobile::~BehaviorCreatorMobile()
{
}

BehaviorFactoryMobile::BehaviorFactoryMobile(std::string behavior_factory_name) : BaseType(behavior_factory_name)
{
}

BehaviorFactoryMobile::~BehaviorFactoryMobile()
{
}

BehaviorType* BehaviorFactoryMobile::create(std::string& behavior_class_name, ros::NodeHandle& nh, ros::NodeHandle& np)
{
    BehaviorCreatorMap::iterator itr = get_map().find(behavior_class_name);

    if (itr != get_map().end())
        return itr->second->create(nh, np);
    else
    {
        ROS_WARN("%s is not a valid behavior class name.", behavior_class_name.c_str());
        return (BehaviorType*) nullptr;
    }
}

void BehaviorFactoryMobile::registerit(const std::string& behavior_class_name, BehaviorCreatorMobile* creator)
{
    ROS_INFO("Registering %s", behavior_class_name.c_str());
    get_map()[behavior_class_name] = creator;
}

std::map<std::string, BehaviorCreatorMobile*>& BehaviorFactoryMobile::get_map()
{
    static BehaviorCreatorMap creator_map;
    return creator_map;
}
