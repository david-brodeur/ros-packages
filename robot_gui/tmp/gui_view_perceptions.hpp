#ifndef GUI_VIEW_PERCEPTIONS_HPP
#define GUI_VIEW_PERCEPTIONS_HPP

#include <robot_gui/gui_central_widget.hpp>

// Qt includes
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QObject>

#include <string>

namespace robot_gui {

	class GuiViewPerceptions : public GuiCentralWidget {

		Q_OBJECT

		public:

			///\brief Default constructor
			GuiViewPerceptions();

			///\brief Destructor
			~GuiViewPerceptions();

			///\brief Get reference to the scene.
			///\return a reference to the scene.
			QGraphicsScene* scene() { return scene_; }

			///\brief Get reference to the view.
			///\return a reference to the view.
			QGraphicsView* view() { return view_; }

			void setEmotion(unsigned int index_, std::string emotion);
			void setEmotion(unsigned int index_, double valence, double, arousal);
			void setGender(unsigned int index_, std::string gender);
			void setImage(QImage& image);
			void setName(unsigned int index_, std::string name);
			void setPrompt(std::string prompt, double x, double y);
			void setSoundSource(unsigned int index_, double x, double y);
			void setSoundType(unsigned int index_, std::string sound_type);
			void setUtterance(unsigned int index_, std::string utterance);

			///\brief Update the displayed scene.
			///\param frame Camera frame;
			void update();

		private:

	        QGraphicsScene* scene_;	///< Scene that contains all graphical elements to be displayed.
			QGraphicsView* view_;	///< View displayed by the widget.

			std::string emotion_;		///< Emotion category of the speaker.
			double valence_;			///< Emotion valence (positive-negave) of the speaker.
			double arousal_;			///< Emotion arousal (active-passive) of the speaker.
			std::string gender_;		///< Gender category of the speaker.
			QImage image_;				///< Image in the scene.
			std::string name_;			///< Name of the speaker.
			std::string prompt_;		///< Prompt played by the robot.
			double x_prompt_;			///< x-axis position of the prompt in the scene.
			double y_prompt_;			///< y-axis position of the prompt in the scene.
			double x_;					///< x-axis position of the sound source in the scene.
			double y_;					///< y-axis position of the sound source in the scene.
			std::string sound_type_;	///< Type of the sound source.
			std::string utterance_;		///< Utterance of the speaker.
	};
}

#endif  // GUI_DISPLAY_CAMERA_HPP
