#include <robot_gui/gui_view_behaviors.hpp>

#include <robot_gui/gui_diagram_arrow.hpp>
#include <robot_gui/gui_diagram_item.hpp>
#include <robot_gui/gui_diagram_text.hpp>

#include <QColor>
#include <QPixmap>
#include <QPointF>

using namespace robot_gui;

GuiViewBehaviors::GuiViewBehaviors() : GuiCentralWidget()
{
    view_ = new QGraphicsView();
    scene_ = new QGraphicsScene(this);
    view_->setScene(scene_);

    layout()->addWidget(view_);
/*
    name_.push_back("stop");
    name_.push_back("move");
    name_.push_back("avoid");
    name_.push_back("load");

    priority_.push_back(3);
    priority_.push_back(1);
    priority_.push_back(2);
    priority_.push_back(4);

    activation_.push_back(true);
    activation_.push_back(false);
    activation_.push_back(false);
    activation_.push_back(true);
*/
}

GuiViewBehaviors::~GuiViewBehaviors()
{
    // TODO remove arrows
    // TODO remove items in general
    delete scene_;
    delete view_;
}

void GuiViewBehaviors::setNames(std::vector<std::string>& name)
{
    name_ = name;
}

void GuiViewBehaviors::setPriorities(std::vector<int>& priority)
{
    priority_ = priority;
}

void GuiViewBehaviors::setActivations(std::vector<uint8_t>& activation)
{
    activation_ = activation;
}

void GuiViewBehaviors::update()
{
    QColor color = Qt::white;

    unsigned int x = 0;
    unsigned int y = 0;

    scene_->clear();

    GuiDiagramItem* item2 = new GuiDiagramItem(GuiDiagramItem::Arbitration);

    item2->setBrush(color);
    item2->setPos(QPointF(250, 225));
    item2->setPen(QPen(Qt::black, 8));
    scene_->addItem(item2);

    for (unsigned int iBehavior = 0; iBehavior < name_.size(); iBehavior++)
    {
        GuiDiagramItem* item = new GuiDiagramItem(GuiDiagramItem::Behavior);

        if (!activation_[iBehavior])
        {
            item->setPen(QPen(Qt::red, 10));
        }

        else
        {
            item->setPen(QPen(Qt::black, 8));
        }

        item->setBrush(color);
        item->setPos(QPointF(x, y));
        scene_->addItem(item);

        GuiDiagramArrow* arrow = new GuiDiagramArrow(item, item2);
        arrow->setColor(Qt::black);
        item->addArrow(arrow);
        item2->addArrow(arrow);
        arrow->setZValue(-1000.0);
        scene_->addItem(arrow);
        arrow->updatePosition();

        GuiDiagramText* text = new GuiDiagramText();
        text->setFont(QFont("Courier", 24, QFont::Bold));
        text->setDefaultTextColor(Qt::black);
        text->setTextInteractionFlags(Qt::NoTextInteraction);
        text->setZValue(1000.0);
        //connect(text, SIGNAL(lostFocus(DiagramTextItem*)), this, SLOT(editorLostFocus(DiagramTextItem*)));
        //connect(text, SIGNAL(selectedChange(QGraphicsItem*)), this, SIGNAL(itemSelected(QGraphicsItem*)));
        text->setPos(QPointF(x, y));
        text->setPlainText(QString::fromStdString(name_[iBehavior]));
        text->setTextWidth(item->polygon().boundingRect().width());
        scene_->addItem(text);

        y += 150;
    }

    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
}

