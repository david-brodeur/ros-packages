#ifndef GUI_DIAGRAM_ARROW_HPP
#define GUI_DIAGRAM_ARROW_HPP

#include <robot_gui/gui_diagram_item.hpp>

// Qt includes
#include <QGraphicsLineItem>

/*! 
 *  \brief     GuiDiagramArrow
 *  \details   This class is an arrow that links two GuiDiagramItem
 *             in a graph.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

class QGraphicsPolygonItem;
class QGraphicsLineItem;
class QGraphicsScene;
class QRectF;
class QPainterPath;

namespace robot_gui
{
    class GuiDiagramArrow : public QGraphicsLineItem
    {
        public:

            enum { Type = UserType + 4 };

            ///\brief Constructor
            GuiDiagramArrow(GuiDiagramItem* item1, GuiDiagramItem* item2, QGraphicsItem* parent = NULL);

            ///\brief Destructor
            ~GuiDiagramArrow();

            ///\brief Get the QGraphicsItem type.
            ///\return the QGraphicsItem type.
            int type() const { return Type; }

            QRectF boundingRect() const;

            QPainterPath shape() const;

            void setColor(const QColor& color);

            ///\brief Get reference to the item where the arrow starts.
            ///\return a reference to the item.
            GuiDiagramItem* item1() { return item1_; }

            ///\brief Get reference to the item the arrow points at.
            ///\return a reference to the item.
            GuiDiagramItem* item2() { return item2_; }

            void updatePosition();

        protected:

            void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0);

        private:

            GuiDiagramItem* item1_;
            GuiDiagramItem* item2_;

            QColor color_;

            QPolygonF arrow_head_;
    };
}

#endif // GUI_DIAGRAM_ARROW_HPP
