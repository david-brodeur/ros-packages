#include <robot_vision/driver_camera_web.hpp>

using namespace robot_vision;

DriverCameraWEB::DriverCameraWEB(ParametersCamera& parameters, std::string driver_name) : DriverCamera(parameters, driver_name)
{
}

DriverCameraWEB::~DriverCameraWEB()
{
    close();
}

int DriverCameraWEB::getFrame(cv::Mat& frame)
{
    if (isOpened())
    {
        if (capture_.grab())
        {
            if (capture_.retrieve(frame))
            {
                return SUCCESS;
            }

            else
            {
                return RETRIEVE_FAILED;
            }
        }

        else
        {
            return GRAB_FAILED;;
        }
    }

    else
    {
        return CAMERA_CLOSED;
    }

}

int DriverCameraWEB::open()
{
    if (isOpened())
    {
        capture_.release();
    }

    capture_.open(device_id_);

    if (isOpened())
    {

        if (!capture_.set(CV_CAP_PROP_FPS, fps_))
        {
            return SET_PARAMETER_FAILED;
        }

        if (!capture_.set(CV_CAP_PROP_FRAME_WIDTH, frame_width_))
        {
            return SET_PARAMETER_FAILED;
        }

        if (!capture_.set(CV_CAP_PROP_FRAME_HEIGHT, frame_height_))
        {
            return SET_PARAMETER_FAILED;
        }

        return SUCCESS;
    }

    else
    {
        return CAMERA_CLOSED;
    }
}

int DriverCameraWEB::close()
{
    if (isOpened())
    {
        capture_.release();
    }

    return SUCCESS;
}
