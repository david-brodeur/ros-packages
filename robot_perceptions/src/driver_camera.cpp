#include <robot_vision/driver_camera.hpp>

using namespace robot_vision;

DriverCamera::DriverCamera(ParametersCamera& parameters, std::string driver_name)
{
    device_id_ = parameters.p_device_id;
    fps_ = parameters.p_fps;
    frame_width_ = parameters.p_frame_width;
    frame_height_ = parameters.p_frame_height;

    driver_name_ = driver_name;
}

DriverCamera::~DriverCamera()
{
}

const char* DriverCamera::getErrorMessage(int return_value)
{
    switch (return_value)
    {
        case SUCCESS:
            return "success";
        case CAMERA_CLOSED:
            return "camera closed";
        case GRAB_FAILED:
            return "grab failed";
        case RETRIEVE_FAILED:
            return "retrieve failed";
        case SET_PARAMETER_FAILED:
            return "set parameter failed";
        default:
            return "error not handled";
    }
}
