#include <robot_speech/speech_interpretation.hpp>

#include <json/json.hpp>
#include <fstream>
#include <iostream>

using namespace robot_speech;

SpeechInterpretation::SpeechInterpretation()
{
}

SpeechInterpretation::~SpeechInterpretation()
{
}

int SpeechInterpretation::process(const std::string& utterance, std::string& prompt)
{
    int error_code;
    unsigned int iLine;
    std::string line;

    if (utterance.empty())
        return NO_UTTERANCE;

    std::ifstream file(dictionary_name_.c_str());
    nlohmann::json json_parser;

    file >> json_parser;

    for (nlohmann::json::iterator it = json_parser.begin(); it != json_parser.end(); it++)
    {
        std::cout << it.key() << " : " << it.value() << std::endl;
    }

    return SUCCESS;
}
/*
bool SpeechInterpretation::isMatch(std::string& utterance, std::string& entry)
{
    int error_code;

    unsigned int iEntryChar;
    unsigned int iUtteranceChar = 0;

    for (iEntryChar = 0; iEntryChar < entry.length(); iEntryChar++)
    {
        if (entry[iEntryChar] == '<')
        {
            error_code = find_required(utterance, entry.substr(iCharacter));

            switch (error_code)
            {
                default:
                    break;
            }
        }

        else if (entry[iEntryChar] == '[')
        {
            error_code = find_optional(utterance, entry.substr(iEntryChar));

            switch (error_code)
            {
                default:
                    break;
            }
        }

        else if (entry[iEntryChar] == '(')
        {
            error_code = find_variable(utterance, entry.substr(iEntryChar));

            switch (error_code)
            {
                default:
                    break;
            }
        }

        else if (entry[iEntryChar] == utterance[iUtteranceChar])
        {

        }
    }
}
*/

int main()
{
    std::string prompt;
    robot_speech::SpeechInterpretation speech_interpretation;

    speech_interpretation.setDictionary("src/ros-packages/robot_speech/data/dictionary.json");
    speech_interpretation.process("Bonjour", prompt);

    return 0;
}

