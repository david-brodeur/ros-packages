#include <robot_vision/algorithm_vision.hpp>

using namespace robot_vision;

AlgorithmVision::AlgorithmVision(std::string algorithm_name)
{
    algorithm_name_ = algorithm_name;
}

AlgorithmVision::~AlgorithmVision()
{
}

const char* AlgorithmVision::getErrorMessage(int return_value)
{
    switch (return_value) {

        case SUCCESS:
            return "success";
        case FAILURE:
            return "failure";
        default:
            return "error not handled";
    }
}
