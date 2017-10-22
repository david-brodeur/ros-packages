#ifndef GUI_DIAGRAM_TEXT_HPP
#define GUI_DIAGRAM_TEXT_HPP

// Qt includes
#include <QGraphicsTextItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QObject>

/*! 
 *  \brief     GuiDiagramText
 *  \details   This class is a text to be inserted in a graph.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_gui
{
    class GuiDiagramText : public QGraphicsTextItem
    {
        Q_OBJECT

        public:

            enum { Type = UserType + 3 };

            ///\brief Constructor
            GuiDiagramText(QGraphicsItem *parent = 0);

            ///\brief Destructor
            ~GuiDiagramText();

            ///\brief Get the QGraphicsItem type.
            ///\return the QGraphicsItem type.
            int type() const { return Type; }

        signals:

            void lostFocus(GuiDiagramText* item);
            void selectedChange(QGraphicsItem* item);

        protected:

            QVariant itemChange(GraphicsItemChange change, const QVariant& value);
            void focusOutEvent(QFocusEvent* event);
    };
}

#endif // GUI_DIAGRAM_TEXT_HPP
