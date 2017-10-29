#include <robot_vision/factory_camera.hpp>

using namespace robot_vision;

FactoryCamera::FactoryCamera()
{
}

FactoryCamera::~FactoryCamera()
{
}

DriverCamera* FactoryCamera::create(DriverCameraID driver_id, ParametersCamera& parameters)
{
	switch (driver_id)
    {
		case WEB:
			return new DriverCameraWEB(parameters);
		default:
			return NULL;
	}
}
