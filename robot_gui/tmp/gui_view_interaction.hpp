#ifndef GUI_DISPLAY_CAMERA_HPP
#define GUI_DISPLAY_CAMERA_HPP

#include <robot_gui/gui_central_widget.hpp>

// Qt includes
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QObject>

namespace robot_gui {

	class GuiDisplayCamera : public GuiCentralWidget {

		Q_OBJECT

		public:

			///\brief Default constructor
			GuiDisplayCamera();

			///\brief Destructor
			~GuiDisplayCamera();

			///\brief Get reference to the scene.
			///\return a reference to the scene.
			QGraphicsScene* scene() { return scene_; }

			///\brief Get reference to the view.
			///\return a reference to the view.
			QGraphicsView* view() { return view_; }

			void setImage(QImage& image);

			///\brief Update the displayed scene.
			///\param frame Camera frame;
			void update();

		private:

	        QGraphicsScene* scene_;	/// < Scene that contains all graphical elements to be displayed in the ComicsView.
			QGraphicsView* view_;	/// < View displayed by the widget.
			QImage image_;			/// < Image in the scene.
	};
}

#endif  // GUI_DISPLAY_CAMERA_HPP
