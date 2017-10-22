#ifndef GUI_VIEW_BEHAVIORS_HPP
#define GUI_VIEW_BEHAVIORS_HPP

#include <robot_gui/gui_central_widget.hpp>

// Qt includes
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QObject>

#include <string>
#include <vector>
#include <stdint.h>

/*! 
 *  \brief     GuiViewBehaviors
 *  \details   This class is a GuiCentralWidget to display a graph 
 *             representing the behavior-based layer architecture 
 *             and the live Behaviors states updates.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_gui
{
    class GuiViewBehaviors : public GuiCentralWidget
    {
        Q_OBJECT

        public:

            ///\brief Default constructor
            GuiViewBehaviors();

            ///\brief Destructor
            ~GuiViewBehaviors();

            ///\brief Get reference to the scene.
            ///\return a reference to the scene.
            QGraphicsScene* scene() { return scene_; }

            ///\brief Get reference to the view.
            ///\return a reference to the view.
            QGraphicsView* view() { return view_; }

            void setNames(std::vector<std::string>& name);

            void setPriorities(std::vector<int>& priority);

            void setActivations(std::vector<uint8_t>& activation);

            ///\brief Update the displayed scene.
            ///\param frame Camera frame;
            void update();

        private:

            QGraphicsScene* scene_; /// < Scene that contains all graphical elements to be displayed in the ComicsView.
            QGraphicsView* view_;   /// < View displayed by the widget.

            std::vector<std::string> name_;
            std::vector<int> priority_;
            std::vector<uint8_t> activation_;
    };
}

#endif // GUI_VIEW_BEHAVIORS_HPP
