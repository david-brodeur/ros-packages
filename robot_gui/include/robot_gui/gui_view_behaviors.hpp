#ifndef GUI_VIEW_BEHAVIORS_HPP
#define GUI_VIEW_BEHAVIORS_HPP

#include <robot_gui/gui_central_widget.hpp>

// Qt includes
#include <QAction>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QObject>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

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

            std::vector<QPushButton*>& pushbuttons() { return pushbuttons_; }

            void setNames(std::vector<std::string>& name);

            void setPriorities(std::vector<int>& priority);

            void setActivations(std::vector<uint8_t>& activation);

            ///\brief Initialize the central widget.
            void init();

            ///\brief Reset the central widget.
            void reset();

            ///\brief Update the displayed scene.
            void update();

        private:

            QWidget* widget_;
            QHBoxLayout* hlayout_;

            QGroupBox* group_box_settings_;
            QVBoxLayout* layout_settings_;

            QGraphicsScene* scene_; /// < Scene that contains all graphical elements to be displayed in the ComicsView.
            QGraphicsView* view_;   /// < View displayed by the widget.

            std::vector<QPushButton*> pushbuttons_;

            std::vector<std::string> name_;
            std::vector<int> priority_;
            std::vector<uint8_t> activation_;
    };
}

#endif // GUI_VIEW_BEHAVIORS_HPP
