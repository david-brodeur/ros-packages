#ifndef GUI_CENTRAL_WIDGET_HPP
#define GUI_CENTRAL_WIDGET_HPP

// Qt includes
#include <QObject>
#include <QVBoxLayout>
#include <QWidget>

/*! 
 *  \brief     GuiCentralWidget
 *  \details   This class is the base class for any central widget
               in the GUI main window.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_gui {

    class GuiCentralWidget : public QWidget
    {
        Q_OBJECT

        public:

            ///\brief Default constructor
            GuiCentralWidget();

            ///\brief Destructor
            ~GuiCentralWidget();

            ///\brief Get reference to the layout.
            ///\return a reference to the layout.
            QVBoxLayout* layout() { return layout_; }

            ///\brief Initialize the central widget.
            virtual void init() = 0;

            ///\brief Reset the central widget.
            virtual void reset() = 0;

            ///\brief Update the widget.
            virtual void update() = 0;

        private:

            QVBoxLayout* layout_;
    };
}

#endif // GUI_CENTRAL_WIDGET_HPP
