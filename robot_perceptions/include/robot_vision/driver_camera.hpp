#ifndef DRIVER_CAMERA_HPP
#define DRIVER_CAMERA_HPP

#include <opencv2/opencv.hpp>

#include <string>

/*! 
 *  \brief     DriverCamera
 *  \details   This class is a generic class for camera drivers.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    struct ParametersCamera
    {
        int p_device_id;             ///< Device ID number.
        int p_fps;                   ///< Frames per second.
        unsigned int p_frame_width;  ///< Frame width.
        unsigned int p_frame_height; ///< Frame height.
    };

    class DriverCamera
    {
        public:

            enum Error { SUCCESS = 0, 
                         CAMERA_CLOSED, 
                         GRAB_FAILED, 
                         RETRIEVE_FAILED, 
                         SET_PARAMETER_FAILED };

            ///\brief Class constructor.
            ///\param parameters Parameters of the camera driver.
            ///\param driver_name Name of the camera driver.
            DriverCamera(ParametersCamera& parameters, std::string driver_name = "/driver/camera");

            ///\brief Class destructor.
            ~DriverCamera();

            ///\brief Get the name of the driver.
            ///\return the name of the driver.
            std::string name() { return driver_name_; }

            ///\brief Get a camera frame.
            ///\param[out] frame Camera frame.
            ///\return an error value.
            virtual int getFrame(cv::Mat& frame) = 0;

            ///\brief Open the camera device.
            ///\return an error value.
            virtual int open() = 0;

            ///\brief Close the camera device.
            ///\return an error value.
            virtual int close() = 0;

            ///\brief Indicate if the camera device is opened.
            ///\return True if the camera device is opened.
            virtual bool isOpened() = 0;

            ///\brief Get Error message
            static const char* getErrorMessage(int return_value);

        protected:

            int device_id_;             ///< Device ID number.
            int fps_;                   ///< Frames per second.
            unsigned int frame_width_;  ///< Frame width.
            unsigned int frame_height_; ///< Frame height.
            std::string driver_name_;   ///< Name of the driver.
    };
}

#endif // DRIVER_CAMERA_HPP
