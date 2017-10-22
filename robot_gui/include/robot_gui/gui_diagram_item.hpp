#ifndef GUI_DIAGRAM_ITEM_HPP
#define GUI_DIAGRAM_ITEM_HPP

// Qt includes
#include <QGraphicsPolygonItem>
#include <QGraphicsPixmapItem>
#include <QList>

/*! 
 *  \brief     GuiDiagramItem
 *  \details   This class is an item to be inserted in a graph.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

// Forward declarations
class QPixmap;
class QGraphicsScene;
class QTextEdit;
class QPainter;
class QStyleOptionGraphicsItem;
class QWidget;
class QPolygonF;

namespace robot_gui
{
    // Forward declarations
    class GuiDiagramArrow;

    class GuiDiagramItem : public QGraphicsPolygonItem
    {
        public:

            enum { Type = QGraphicsItem::UserType + 15 };
            enum DiagramItemType { Step, Conditional, StartEnd, Io,
                    Arbitration, Behavior };

            ///\brief Constructor
            GuiDiagramItem(DiagramItemType diagram_item_type, QGraphicsItem* parent = NULL);

            ///\brief Desstructor
            ~GuiDiagramItem();

            ///\brief Get the QGraphicsItem type.
            ///\return the QGraphicsItem type.
            int type() const { return Type; }

            ///\brief Get the type of a diagram item.
            ///\return the type of a diagram item.
            DiagramItemType diagramItemType() const { return diagram_item_type_; }

            ///\brief Add an arrow.
            ///\param arrow GuiDiagramArrow to be added.
            void addArrow(GuiDiagramArrow* arrow);

            ///\brief Remove an arrow.
            ///\param arrow GuiDiagramArrow to be removed.
            void removeArrow(GuiDiagramArrow* arrow);

            ///\brief Remove all arrows.
            void removeArrows();

            ///\brief Get the item's polygon shape.
            ///\return the item's polygon shape.
            QPolygonF polygon() const { return polygon_; }

            ///\brief Get the pixmap of the diagram item.
            ///\return the pixmap of the diagram item.
            QPixmap image() const;

        protected:

            QVariant itemChange(GraphicsItemChange change, const QVariant& value);

        private:

            DiagramItemType diagram_item_type_;

            QPolygonF polygon_;

            QList<GuiDiagramArrow*> arrows_;
    };
}

#endif // GUI_DIAGRAM_ITEM_HPP
