#ifndef SPEECH_TTS_QT_HPP
#define SPEECH_TTS_QT_HPP

/*! 
 *  \brief     SpeechTTSQt
 *  \details   This class implements a Text-to-Speech tool using the Qt library.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2018
 *  \copyright GNU Public License.
 */

namespace robot_speech
{
    class SpeechTTSQt
    {

        public:

            enum Error { SUCCESS = 0, 
                         FAILURE };

            ///\brief Class constructor.
            SpeechTTSQt();

            ///\brief Class destructor.
            ~SpeechTTSQt();

        private:

    };
}

#endif // SPEECH_TTS_QT_HPP
