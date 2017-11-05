#ifndef SPEECH_INTERPRETATION
#define SPEECH_INTERPRETATION

#include <json/json.hpp>

#include <map>
#include <string>

/*! 
 *  \brief     SpeechInerpretation
 *  \details   This class matches an utterance to a prompt.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_speech
{
    struct ParametersSpeechInterpretation
    {
        std::string p_dictionary_file_path;
    };

    struct SpeechContext
    {
    };

    class SpeechInterpretation
    {
        public:

            enum class Error : int 
            { 
                SUCCESS = 0, 
                ERROR,
                NO_UTTERANCE,
                NO_MATCH
            };

            typedef std::map<std::string, std::string> VariableMap; ///< VariableMap type definition.

            SpeechInterpretation(const ParametersSpeechInterpretation& parameters);

            ~SpeechInterpretation();

            SpeechInterpretation::Error init();

            void reset();

            SpeechInterpretation::Error process(const std::string& utterance, std::string& prompt);

            static const char* getErrorDescription(SpeechInterpretation::Error error_code);

        private:

            bool isMatch(const std::string& utterance, const std::string& match);
            bool isRequiredMatch(unsigned int& iUtterance, unsigned int& iMatch, const std::string& utterance, const std::string& match);
            bool isOptionalMatch(unsigned int& iUtterance, unsigned int& iMatch, const std::string& utterance, const std::string& match);
            bool isVariableMatch(unsigned int& iUtterance, unsigned int& iMatch, const std::string& utterance, const std::string& match);

            std::string file_path_;
            std::string dictionary_name_;
            std::string dictionary_description_;

            std::shared_ptr<nlohmann::json> json_;

            VariableMap map_;
    };
}

#endif // SPEECH_INTERPRETATION
