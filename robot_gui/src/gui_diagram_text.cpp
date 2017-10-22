#include <robot_gui/gui_diagram_text.hpp>

using namespace robot_gui;

GuiDiagramText::GuiDiagramText(QGraphicsItem *parent) : QGraphicsTextItem(parent)
{
	setFlag(QGraphicsItem::ItemIsMovable, false);
	setFlag(QGraphicsItem::ItemIsSelectable, false);
}

GuiDiagramText::~GuiDiagramText()
{
}

QVariant GuiDiagramText::itemChange(GraphicsItemChange change, const QVariant &value)
{
	if (change == QGraphicsItem::ItemSelectedHasChanged)
    {
		emit selectedChange(this);
	}

	return value;
}

void GuiDiagramText::focusOutEvent(QFocusEvent* event)
{
	setTextInteractionFlags(Qt::NoTextInteraction);
	emit lostFocus(this);
	QGraphicsTextItem::focusOutEvent(event);
}
