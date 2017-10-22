#include <robot_gui/gui_diagram_arrow.hpp>

#include <math.h>

#include <QPen>
#include <QPainter>

using namespace robot_gui;

const qreal Pi = 3.141592;

GuiDiagramArrow::GuiDiagramArrow(GuiDiagramItem* item1, GuiDiagramItem* item2, QGraphicsItem* parent) : QGraphicsLineItem(parent)
{
    item1_ = item1;
    item2_ = item2;

    setFlag(QGraphicsItem::ItemIsSelectable, false);
    color_ = Qt::black;
    setPen(QPen(color_, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
}

GuiDiagramArrow::~GuiDiagramArrow()
{
}

QRectF GuiDiagramArrow::boundingRect() const
{
    qreal extra = (pen().width() + 20) / 2.0;

    QRectF rect = QRectF(line().p1(), 
        QSizeF(line().p2().x() - line().p1().x(), line().p2().y() - line().p1().y()));

    return rect.normalized().adjusted(-extra, -extra, extra, extra);
}

QPainterPath GuiDiagramArrow::shape() const
{
    QPainterPath path = QGraphicsLineItem::shape();
    path.addPolygon(arrow_head_);
    return path;
}

void GuiDiagramArrow::setColor(const QColor& color)
{
    QPen pen = this->pen();
    pen.setColor(color);
    setPen(pen);
}

void GuiDiagramArrow::updatePosition()
{
    QLineF line(mapFromItem(item1_, 0, 0), mapFromItem(item2_, 0, 0));
    setLine(line);
}

void GuiDiagramArrow::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    QPointF point1;
    QPointF point2;
    QPointF intersection;

    QLineF line;
    QLineF polyline;
    QPolygonF polygon;

    if (item1_->collidesWithItem(item2_))
    {
        return;
    }

    painter->setPen(pen());
    painter->setBrush(pen().color());

    qreal arrow_size = 20;

    line = QLineF(item1_->pos(), item2_->pos());
    polygon = item2_->polygon();

    point1 = polygon.first() + item2_->pos();

    for (int iPoint = 1; iPoint < polygon.count(); ++iPoint)
    {
        point2 = polygon.at(iPoint) + item2_->pos();
        polyline = QLineF(point1, point2);
        QLineF::IntersectType intersectType = polyline.intersect(line, &intersection);

        if (intersectType == QLineF::BoundedIntersection)
        {
            break;
        }

        point1 = point2;
    }

    setLine(QLineF(intersection, item1_->pos()));

    double angle = ::acos(this->line().dx() / this->line().length());

    if (this->line().dy() >= 0)
    {
        angle = (Pi * 2) - angle;
    }

    QPointF arrow_point1 = this->line().p1() + QPointF(sin(angle + Pi / 3) * arrow_size, cos(angle + Pi / 3) * arrow_size);
    QPointF arrow_point2 = this->line().p1() + QPointF(sin(angle + Pi - Pi / 3) * arrow_size, cos(angle + Pi - Pi / 3) * arrow_size);

    arrow_head_.clear();
    arrow_head_ << this->line().p1() << arrow_point1 << arrow_point2;

    painter->drawLine(this->line());
    painter->drawPolygon(arrow_head_);

    if (isSelected())
    {
        painter->setPen(QPen(color_, 1, Qt::DashLine));
        QLineF myLine = this->line();
        myLine.translate(0, 4.0);
        painter->drawLine(myLine);
        myLine.translate(0,-8.0);
        painter->drawLine(myLine);
    }
}
