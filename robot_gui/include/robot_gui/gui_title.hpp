#ifndef GUI_TITLE_HPP
#define GUI_TITLE_HPP

// Qt includes
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>

#include <QWidget>

/*! 
 *  \brief     GuiTitle
 *  \details   This class is a title dock widget.
 *             robotic speech interactions.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_gui
{
    class GuiTitle : public QDockWidget
    {
        Q_OBJECT

        public: 

            /// \brief Default constructor
            GuiTitle();

            /// \brief Destructor
            ~GuiTitle();

            ///\brief Set a picture as title. 
            void setPixmap(std::string file_path);

            ///\brief Set a text as title. 
            void setText(std::string title);

        private Q_SLOTS:

        private:

            QWidget* widget_;     /// < Widget that contains the layout.
            QHBoxLayout* layout_; /// < Layout to insert the label.
            QLabel* label_;       /// < Label that contains the image to display.
    };
}

#endif // GUI_TITLE_HPP
