#include <robot_speech/speech_interpretation.hpp>

#include <fstream>
#include <iostream>

using namespace robot_speech;

SpeechInterpretation::SpeechInterpretation(const ParametersSpeechInterpretation& parameters)
{
    file_path_ = parameters.p_dictionary_file_path;
}

SpeechInterpretation::~SpeechInterpretation()
{
    reset();
}

SpeechInterpretation::Error SpeechInterpretation::init()
{
    reset();

    std::ifstream file(file_path_.c_str());

    if (file.is_open())
    {
        file >> (*json_);

        dictionary_name_ = (*json_)["dictionary"]["name"];
        dictionary_description_ = (*json_)["dictionary"]["description"];

        std::cout << "Using dictionary ";
        std::cout << dictionary_name_;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << dictionary_description_;
        std::cout << std::endl;
    }

    else
    {
        std::cout << "[SpeechInterpretation] Could not open dictionary with path : " + file_path_;
        return Error::ERROR;
    }

    return Error::SUCCESS;
}

void SpeechInterpretation::reset()
{
    json_.reset(new nlohmann::json());
}

SpeechInterpretation::Error SpeechInterpretation::process(const std::string& utterance, std::string& prompt)
{
    int error_code;
    unsigned int iLine;
    std::string line;

    if (utterance.empty())
        return Error::NO_UTTERANCE;

    for (unsigned int i = 0; i < (*json_)["dictionary"]["items"].size(); i++)
    {
        if ((*json_)["dictionary"]["items"][i] != nullptr)
        {
            if (isMatch(utterance, (*json_)["dictionary"]["items"][i]["utterance"]))
            {
                prompt = (*json_)["dictionary"]["items"][i]["prompt"]["default"];
                return Error::SUCCESS;
            }
        }
    }

    return Error::NO_MATCH;
}

const char* SpeechInterpretation::getErrorDescription(SpeechInterpretation::Error error_code)
{
    switch (error_code)
    {
        case Error::SUCCESS:
            return "success";
        case Error::ERROR:
            return "error";
        case Error::NO_UTTERANCE:
            return "no utterance";
        case Error::NO_MATCH:
            return "no match";
        default:
            return "NOT IMPLEMENTED";
    }    
}

bool SpeechInterpretation::isMatch(const std::string& utterance, const std::string& match)
{
    int error_code;
    bool reached_utterance_end;
    unsigned int iUtterance;

    reached_utterance_end = false;
    iUtterance; = 0;

    for (unsigned int iMatch = 0; iMatch < match.length(); iMatch++)
    {
        if (utterance[iUtterance] == match[iMatch] && !reached_utterance_end)
        {
            iUtterance++;
        }

        else if (match[iMatch] == '<' && !reached_utterance_end)
        {
            isRequiredMatch(iUtterance, iMatch, utterance, match);
        }

        else if (match[iMatch] == '[')
        {
            isOptionalMatch(iUtterance, iMatch, utterance, match);
        }

        else if (match[iMatch] == '(' && !reached_utterance_end)
        {
            isVariableMatch(iUtterance, iMatch, utterance, match);
        }

        else
        {
            return false;
        }

        if (iUtterance >= utterance.length())
            reached_utterance_end = true;
    }

    return true;
}

bool SpeechInterpretation::isRequiredMatch(unsigned int& iUtterance, unsigned int& iMatch, const std::string& utterance, const std::string& match)
{
    bool no_match;
    unsigned int iTempMatch;
    unsigned int iTempUtterance;

    no_match = false;
    reached_utterance_end = false;
    iTempUtterance = iUtterance;

    for (iTempMatch = iMatch+1; iMatch < match.length(); iTempMatch++)
    {
        if (utterance[iTempUtterance] == match[iTempMatch] && !no_match)
        {
            iTempUtterance++;
        }

        else if (match[iTempMatch] == '<' && !no_match)
        {
            isRequiredMatch(iTempUtterance, iTempMatch, utterance, match);
        }

        else if (match[iTempMatch] == '[' && !no_match)
        {
            isOptionalMatch(iTempUtterance, iTempMatch, utterance, match);
        }

        else if (match[iTempMatch] == '(' && !no_match)
        {
            isVariableMatch(iTempUtterance, iTempMatch, utterance, match);
        }

        else if (match[iTempMatch] == ',')
        {
            if (!no_match)
                break;

            iTempUtterance = iUtterance;
            no_match = false;
        }

        else if (match[iTempMatch] == '>')
        {
            if (!no_match)
                break;

            return false;
        }

        else
        {
            no_match = true;
        }
    }

    if (iTempMatch >= match.length())
    {
        std::cout << "[SpeechInterpretation] No closing tag '>' for required element." << std::endl;
        return false;
    }

    while (match[iTempMatch] != '>')
        iTempMatch++;

    iUtterance = iTempUtterance;
    iMatch = iTempMatch;

    return true;
}

bool SpeechInterpretation::isOptionalMatch(unsigned int& iUtterance, unsigned int& iMatch, const std::string& utterance, const std::string& match)
{
    bool no_match;
    unsigned int iTempUtterance;
    unsigned int iTempMatch;

    no_match = false;
    iTempUtterance = iUtterance;

    for (iTempMatch = iMatch+1; iMatch < match.length(); iTempMatch++)
    {
        if (utterance[iTempUtterance] == match[iTempMatch] && !no_match)
        {
            iTempUtterance++;
        }

        else if (match[iTempMatch] == '<' && !no_match)
        {
            isRequiredMatch(iTempUtterance, iTempMatch, utterance, match);
        }

        else if (match[iTempMatch] == '[' && !no_match)
        {
            isOptionalMatch(iTempUtterance, iTempMatch, utterance, match);
        }

        else if (match[iTempMatch] == '(' && !no_match)
        {
            isVariableMatch(iTempUtterance, iTempMatch, utterance, match);
        }

        else if (match[iTempMatch] == ']')
        {
            if (!no_match)
                break;

            return false;
        }

        else
        {
            no_match = true;
        }
    }

    if (iTempMatch >= match.length())
    {
        std::cout << "[SpeechInterpretation] No closing tag ']' for optional element." << std::endl;
        return false;
    }

    while (match[iTempMatch] != ']')
        iTempMatch++;

    iUtterance = iTempUtterance;
    iMatch = iTempMatch;

    return true;
}

bool SpeechInterpretation::isVariableMatch(unsigned int& iUtterance, unsigned int& iMatch, const std::string& utterance, const std::string& match)
{
    std::string temp;
    std::string type;
    std::string key;
    std::string value;

    size_t position;
    unsigned int iTempMatch;

    // Get variable type
    temp = match.substr(iMatch+1);
    position = temp.find(" ");

    if (position == std::string::npos)
        return false;

    type = temp.substr(0, position);

    // Get variable key
    temp = match.substr(iMatch+type.length()+2);
    position = temp.find(")");

    if (position == std::string::npos)
        return false;

    key = temp.substr(0, position);

    iMatch += match.substr(iMatch+1).find(")") + 1;

    // Get variable value
    temp = utterance.substr(iUtterance);
    position = temp.find(" ");

    if (position == std::string::npos)
        value = temp.substr(0, temp.length());
    else
        value = temp.substr(0, position);

    map_[key] = value;

    iUtterance += value.length();

    return true;
}

int main()
{
    SpeechInterpretation::Error error_code;
    std::string prompt;

    ParametersSpeechInterpretation parameters;
    parameters.p_dictionary_file_path = "src/ros-packages/robot_speech/data/test.json";

    SpeechInterpretation speech_interpretation(parameters);

    error_code = speech_interpretation.init();

    if (error_code != SpeechInterpretation::Error::SUCCESS)
    {
        std::cout << SpeechInterpretation::getErrorDescription(error_code) << std::endl;
        return 0;        
    }

    error_code = speech_interpretation.process("Bonjour David", prompt);

    if (error_code != SpeechInterpretation::Error::SUCCESS)
    {
        std::cout << SpeechInterpretation::getErrorDescription(error_code) << std::endl;
        return 0;
    }

    if (!prompt.empty())
        std::cout << prompt << std::endl;
    else
        std::cout << "No prompt" << std::endl;

    return 0;
}

