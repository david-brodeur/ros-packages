#include <robot_vision/algorithm_vision_haar.hpp>

using namespace robot_vision;

AlgorithmVisionHaar::AlgorithmVisionHaar(ParametersVisionHaar& parameters, std::string algorithm_name) : AlgorithmVision(algorithm_name)
{
    model_name_ = parameters.p_model_name;
    scale_factor_ = parameters.p_scale_factor;
    min_neighbors_ = parameters.p_min_neighbors;
    flags_ = parameters.p_flags | CV_HAAR_SCALE_IMAGE;
    min_ = cv::Size(parameters.p_min, parameters.p_min);
    max_ = cv::Size(parameters.p_max, parameters.p_max);
}

AlgorithmVisionHaar::~AlgorithmVisionHaar()
{
}

int AlgorithmVisionHaar::init()
{
    if (!haar_.load(model_name_))
    {
        return AlgorithmVision::FAILURE;
    }

    return AlgorithmVision::SUCCESS;
}

int AlgorithmVisionHaar::process(cv::Mat frame)
{
    double min;
    double max;
    cv::Mat gray;

    if (frame.channels() == 3) {

        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        cv::equalizeHist(gray, gray);
    }

    else {

        cv::minMaxIdx(frame, &min, &max);
        frame.convertTo(gray, CV_8UC1, 255 / (max-min), -min);
        cv::equalizeHist(gray, gray);
    }

    haar_.detectMultiScale(gray, candidates_, scale_factor_, min_neighbors_, flags_, min_, max_);

    filtered_.clear();
    filter(candidates_, filtered_);

    return AlgorithmVision::SUCCESS;
}

void AlgorithmVisionHaar::filter(const std::vector<cv::Rect>& candidates, std::vector<cv::Rect>& filtered)
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
