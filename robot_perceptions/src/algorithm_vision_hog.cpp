#include <robot_vision/algorithm_vision_hog.hpp>

using namespace robot_vision;

AlgorithmVisionHOG::AlgorithmVisionHOG(ParametersVisionHOG* parameters, std::string algorithm_name) : AlgorithmVision(algorithm_name)
{
    hit_threshold_ = parameters->p_hit_threshold;
    win_stride_ = cv::Size(parameters->p_win_stride, parameters->p_win_stride);
    padding_ = cv::Size(parameters->p_padding, parameters->p_padding);
    scale_zero_ = parameters->p_scale_zero;
    group_threshold_ = parameters->p_group_threshold;
}

AlgorithmVisionHOG::~AlgorithmVisionHOG()
{
}

int AlgorithmVisionHOG::init()
{
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    return AlgorithmVision::SUCCESS;
}

int AlgorithmVisionHOG::process(cv::Mat frame)
{
    double min;
    double max;
    cv::Mat gray;

    if (frame.channels() == 3)
    {
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        cv::equalizeHist(gray, gray);
    }

    else
    {
        cv::minMaxIdx(frame, &min, &max);
        frame.convertTo(gray, CV_8UC1, 255 / (max-min), -min);
        cv::equalizeHist(gray, gray);
    }

    hog_.detectMultiScale(gray, candidates_, hit_threshold_, win_stride_, padding_, scale_zero_, group_threshold_);

    filtered_.clear();
    filter(candidates_, filtered_);

    return AlgorithmVision::SUCCESS;
}

void AlgorithmVisionHOG::filter(const std::vector<cv::Rect>& candidates, std::vector<cv::Rect>& filtered)
{
    size_t i, j;

    for (i = 0; i < candidates.size(); i++)
    {
        cv::Rect rect = candidates[i];

        for (j = 0; j < candidates.size(); j++)
        {
            if (j != i && (rect & candidates[j]) == rect)
            {
                break;
            }
        }

        if (j == candidates.size())
        {
            filtered.push_back(rect);
        }
    }
}
