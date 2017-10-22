#include <robot_gui/gui_diagram_item.hpp>
#include <robot_gui/gui_diagram_arrow.hpp>

#include <QGraphicsScene>
#include <QPainter>

using namespace robot_gui;

GuiDiagramItem::GuiDiagramItem(DiagramItemType diagram_item_type, QGraphicsItem* parent) : QGraphicsPolygonItem(parent)
{
    diagram_item_type_ = diagram_item_type;

    QPainterPath path;

    switch (diagram_item_type_)
    {
        case StartEnd:

            path.moveTo(200, 50);
            path.arcTo(150, 0, 50, 50, 0, 90);
            path.arcTo(50, 0, 50, 50, 90, 90);
            path.arcTo(50, 50, 50, 50, 180, 90);
            path.arcTo(150, 50, 50, 50, 270, 90);
            path.lineTo(200, 25);
            polygon_ = path.toFillPolygon();
            break;

        case Conditional:

            polygon_ << QPointF(-100, 0) << QPointF(0, 100)
                    << QPointF(100, 0) << QPointF(0, -100)
                    << QPointF(-100, 0);
            break;

        case Step:

            polygon_ << QPointF(-100, -100) << QPointF(100, -100)
                    << QPointF(100, 100) << QPointF(-100, 100)
                    << QPointF(-100, -100);
            break;

        case Arbitration:

            polygon_ << QPointF(-50, -100) << QPointF(50, -100)
                    << QPointF(50, 100) << QPointF(-50, 100)
                    << QPointF(-50, -100);
            break;

        case Behavior:

            polygon_ << QPointF(-100, -50) << QPointF(100, -50)
                    << QPointF(100, 50) << QPointF(-100, 50)
                    << QPointF(-100, -50);
            break;

        default:

            polygon_ << QPointF(-120, -80) << QPointF(-70, 80)
                    << QPointF(120, 80) << QPointF(70, -80)
                    << QPointF(-120, -80);
            break;
    }

    setPolygon(polygon_);

    setFlag(QGraphicsItem::ItemIsMovable, true);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
}

GuiDiagramItem::~GuiDiagramItem()
{
}

void GuiDiagramItem::addArrow(GuiDiagramArrow* arrow)
{
    arrows_.append(arrow);
}


void GuiDiagramItem::removeArrow(GuiDiagramArrow* arrow)
{
    int index = arrows_.indexOf(arrow);

    if (index != -1)
    {
        arrows_.removeAt(index);
    }
}


void GuiDiagramItem::removeArrows()
{
    foreach (GuiDiagramArrow* arrow, arrows_)
    {
        arrow->item1()->removeArrow(arrow);
        arrow->item2()->removeArrow(arrow);
        scene()->removeItem(arrow);
        delete arrow;
    }
}

QPixmap GuiDiagramItem::image() const
{
    QPixmap pixmap(250, 250);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setPen(QPen(Qt::black, 8));
    painter.translate(125, 125);
    painter.drawPolyline(polygon_);

    return pixmap;
}

QVariant GuiDiagramItem::itemChange(GraphicsItemChange change, const QVariant& value)
{
    if (change == QGraphicsItem::ItemPositionChange)
    {
        foreach (GuiDiagramArrow* arrow, arrows_)
        {
            arrow->updatePosition();
        }
    }

    return value;
}
