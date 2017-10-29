#ifndef ALGORITHM_VISION_HPP
#define ALGORITHM_VISION_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string>

/*! 
 *  \brief     AlgorithmVision
 *  \details   This class is a base class for image processing and 
 *             computer vision algorithms.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    class AlgorithmVision
    {
        public:

            enum Error { SUCCESS = 0, 
                            FAILURE };

            ///\brief Class constructor.
            AlgorithmVision(std::string algorithm_name = "/algorithm/vision");

            ///\brief Class destructor.
            ~AlgorithmVision();

            ///\brief Get the name of the algorithm.
            ///\return the name of the algorithm.
            std::string name() { return algorithm_name_; }

            ///\brief Initialize the algorithm.
            ///\return an error value.
            virtual int init() = 0;

            ///\brief Process a video frame.
            ///\param frame Video frame.
            ///\return an error value.
            virtual int process(cv::Mat frame) = 0;

            ///\brief Get Error message
            static const char* getErrorMessage(int return_value);

        protected:

            std::string algorithm_name_; ///< Name of the algorithm.
    };
}

#endif // ALGORITHM_VISION_HPP
