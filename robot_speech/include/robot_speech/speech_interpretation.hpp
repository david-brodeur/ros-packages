#ifndef SPEECH_INTERPRETATION
#define SPEECH_INTERPRETATION

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
    struct UtteranceVariable {

        char * name;
        char * value;

        struct UtteranceVariable * next;

    } UtteranceVariable;

    struct ParametersSpeechContext
    {
    };

    class SpeechInterpretation
    {
        public:

            enum Error { SUCCESS = 0, 
                         ERROR,
                         NO_UTTERANCE,
                         NO_MATCH };

            SpeechInterpretation();

            ~SpeechInterpretation();

            int init();

            void reset();

            void setDictionary(const std::string& dictionary_name) { dictionary_name_ = dictionary_name; }

            int setContext(ParametersSpeechContext& context);

            int process(const std::string& utterance, std::string& prompt);

        private:


/*
            int findMatch();

            int findOptional();
            int findRequired();
            int findVariable();
*/
            std::string dictionary_name_;
            std::string interpretation_;
/*
            unsigned int line_index_;
            bool do_store_variable_;
*/


    };
}

#endif // SPEECH_INTERPRETATION
