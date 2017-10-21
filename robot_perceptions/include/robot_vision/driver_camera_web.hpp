#ifndef DRIVER_CAMERA_WEB_HPP
#define DRIVER_CAMERA_WEB_HPP

#include <robot_vision/driver_camera.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

/*! 
 *  \brief     DriverCameraWEB
 *  \details   This class is a webcam driver.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    class DriverCameraWEB: public DriverCamera
    {
        public:

            ///\brief Class constructor.
            ///\param parameters Parameters of the camera driver.
            ///\param driver_name Name of the camera driver.
            DriverCameraWEB(ParametersCamera* parameters, std::string driver_name = "/driver/camera/web");

            ///\brief Class destructor.
            ~DriverCameraWEB();

            ///\brief Get a camera frame.
            ///\param[out] frame Camera frame.
            ///\return an error value.
            int getFrame(cv::Mat& frame);

            ///\brief Open the camera device.
            ///\return an error value.
            int open();

            ///\brief Close the camera device.
            ///\return an error value.
            int close();

            ///\brief Indicate if the camera device is opened.
            ///\return True if the camera device is opened.
            bool isOpened() { return capture_.isOpened(); }

        private:

            cv::VideoCapture capture_; ///< Camera device.
    };
}

#endif // DRIVER_CAMERA_WEB_HPP
