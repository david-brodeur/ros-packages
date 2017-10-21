#ifndef ALGORITHM_VISION_HOG_HPP
#define ALGORITHM_VISION_HOG_HPP

#include <robot_vision/algorithm_vision.hpp>

#include "opencv2/objdetect/objdetect.hpp"

#include <vector>

/*! 
 *  \brief     AlgorithmVisionHOG
 *  \details   This class implements OpenCV's Histogram Of Gradient 
 *             algorithm with the default people detector.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \remark    See for more details: http://docs.opencv.org/2.4/modules/gpu/doc/object_detection.html
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_vision
{
    struct ParametersVisionHOG
    {
        double p_hit_threshold; ///< Threshold for the distance between features and SVM classifying plane.
        int p_win_stride;       ///< Window stride. It must be a multiple of block stride.
        int p_padding;          ///< Mock parameter to keep the CPU interface compatibility. It must be (0,0).
        double p_scale_zero;    ///< Coefficient of the detection window increase.
        int p_group_threshold;  ///< Coefficient to regulate the similarity threshold. 0 means not to perform grouping.
    };

    class AlgorithmVisionHOG: public AlgorithmVision
    {
        public:

            ///\brief Class constructor.
            ///\param parameters Parameter structure pointer to configure the HOG algorithm.
            AlgorithmVisionHOG(ParametersVisionHOG* parameters, std::string algorithm_name = "/algorithm/vision/HOG");

            ///\brief Class destructor.
            ~AlgorithmVisionHOG();

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

            cv::HOGDescriptor hog_;            ///< HOG Classifier.
            std::vector<cv::Rect> candidates_; ///< Regions of interest (ROI) candidates.
            std::vector<cv::Rect> filtered_;   ///< Filtered ROIs.

            double hit_threshold_; ///< Threshold for the distance between features and SVM classifying plane.
            cv::Size win_stride_;  ///< Window stride. It must be a multiple of block stride.
            cv::Size padding_;     ///< Mock parameter to keep the CPU interface compatibility. It must be (0,0).
            double scale_zero_;    ///< Coefficient of the detection window increase.
            int group_threshold_;  ///< Coefficient to regulate the similarity threshold. 0 means not to perform grouping.
    };
}

#endif // ALGORITHM_VISION_HOG_HPP
