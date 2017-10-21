#ifndef FACTORY_CAMERA_HPP
#define FACTORY_CAMERA_HPP

#include <robot_vision/driver_camera.hpp>
#include <robot_vision/driver_camera_web.hpp>

#include <string>

/*! 
 *  \brief     FactoryCamera
 *  \details   This class creates camera drivers.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

 // TODO implement Registration pattern

namespace robot_vision
{
    class FactoryCamera
    {
        public:

            enum DriverCameraID { WEB = 0 };    ///< Available camera drivers

            ///\brief Class constructor.
            FactoryCamera();

            ///\brief Class destructor.
            ~FactoryCamera();

            ///\brief Create a camera driver.
            ///\param driver_id Camera driver ID.
            ///\param parameters Parameters of the camera driver.
            DriverCamera* create(DriverCameraID driver_id, ParametersCamera* parameters);
    };
}

#endif // FACTORY_CAMERA_HPP
