#ifndef ALGORITHM_VISION_HAAR_HPP
#define ALGORITHM_VISION_HAAR_HPP

#include <robot_vision/algorithm_vision.hpp>

#include "opencv2/objdetect/objdetect.hpp"

#include <vector>

/*! 
 *  \brief     AlgorithmVisionHaar
 *  \details   This class implements OpenCV's Haar Cascade classifier 
 *             algorithm with the default people detector.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \remark    See for more details: https://docs.opencv.org/2.4/modules/objdetect/doc/cascade_classification.html
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    struct ParametersVisionHaar
    {
        std::string p_model_name; ///< Name of the model file.
        float p_scale_factor;     ///< Parameter specifying how much the image size is reduced at each image scale.
        int p_min_neighbors;      ///< Parameter specifying how many neighbors each candidate rectangle should have to retain it.
        int p_flags;              ///< Parameter with the same meaning for an old cascade as in the function cvHaarDetectObjects. It is not used for a new cascade.
        int p_min;                ///< Minimum possible object size. Objects smaller than that are ignored.
        int p_max;                ///< Maximum possible object size. Objects larger than that are ignored.
    };

    class AlgorithmVisionHaar: public AlgorithmVision
    {
        public:

            ///\brief Class constructor.
            ///\param parameters Parameter structure pointer to configure the Haar algorithm.
            AlgorithmVisionHaar(ParametersVisionHaar& parameters, std::string algorithm_name = "/algorithm/vision/Haar");

            ///\brief Class destructor.
            ~AlgorithmVisionHaar();

            ///\brief Initialize the algorithm.
            ///\return an error value.
            int init();

            ///\brief Process a video frame.
            ///\param frame Video frame.
            int process(cv::Mat frame);

            ///\brief Get the number of ROI candidates.
            ///\return number of ROI candidates.
            unsigned int getNCandidates() { return candidates_.size(); }

            ///\brief Get the number of filtered ROIs.
            ///\return number of filtered ROIs.
            unsigned int getNFiltered() { return filtered_.size(); }

            ///\brief Get the x-axis value of the top left point in the index-th filtered ROI.
            ///\param index Index of the filtered ROI.
            ///\return the x-axis value of the top left point in the index-th filtered ROI.
            unsigned int getX(unsigned int index) { return filtered_[index].x; }

            ///\brief Get the y-axis value of the top left point in the index-th filtered ROI.
            ///\param index Index of the filtered ROI.
            ///\return the y-axis value of the top left point in the index-th filtered ROI.
            unsigned int getY(unsigned int index) { return filtered_[index].y; }

            ///\brief Get the width of the index-th filtered ROI.
            ///\param index Index of the filtered ROI.
            ///\return the width of the index-th filtered ROI.
            unsigned int getWidth(unsigned int index) { return filtered_[index].width; }

            ///\brief Get the height of the index-th filtered ROI.
            ///\param index Index of the filtered ROI.
            ///\return the height of the index-th filtered ROI.
            unsigned int getHeight(unsigned int index) { return filtered_[index].height; }

        private:

            ///\brief Filter ROI candidates.
            ///\param candidates Regions of interest (ROI) candidates.
            ///\param filtered Filtered ROIs
            void filter(const std::vector<cv::Rect>& candidates, std::vector<cv::Rect>& filtered);

            cv::CascadeClassifier haar_;       ///< Haar classifier.
            std::vector<cv::Rect> candidates_; ///< Regions of interest (ROI) candidates.
            std::vector<cv::Rect> filtered_;   ///< Filtered ROIs.

            cv::String model_name_; ///< Name of the model file.
            float scale_factor_;    ///< Parameter specifying how much the image size is reduced at each image scale.
            int min_neighbors_;     ///< Parameter specifying how many neighbors each candidate rectangle should have to retain it.
            int flags_;             ///< Parameter with the same meaning for an old cascade as in the function cvHaarDetectObjects. It is not used for a new cascade.
            cv::Size min_;          ///< Minimum possible object size. Objects smaller than that are ignored.
            cv::Size max_;          ///< Maximum possible object size. Objects larger than that are ignored.
    };
}

#endif // ALGORITHM_VISION_HAAR_HPP
