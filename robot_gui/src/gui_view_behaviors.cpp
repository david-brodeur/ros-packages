#include <robot_gui/gui_view_behaviors.hpp>

#include <robot_gui/gui_diagram_arrow.hpp>
#include <robot_gui/gui_diagram_item.hpp>
#include <robot_gui/gui_diagram_text.hpp>

#include <QColor>
#include <QPixmap>
#include <QPointF>
#include <QPushButton>

using namespace robot_gui;

GuiViewBehaviors::GuiViewBehaviors() : GuiCentralWidget()
{
}

GuiViewBehaviors::~GuiViewBehaviors()
{
    reset();
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

void GuiViewBehaviors::init()
{
    reset();

    widget_ = new QWidget();
    hlayout_ = new QHBoxLayout();

    view_ = new QGraphicsView();
    scene_ = new QGraphicsScene(this);
    view_->setScene(scene_);

    group_box_settings_ = new QGroupBox("Settings");
    layout_settings_ = new QVBoxLayout();

    for (unsigned int iBehavior = 0; iBehavior < name_.size(); iBehavior++)
    {
        QPushButton* pushbutton = new QPushButton(QString::fromStdString(name_[iBehavior]));
        //QAction* action;
        layout_settings_->addWidget(pushbutton);
    }

    layout_settings_->addStretch(1);

    group_box_settings_->setLayout(layout_settings_);

    hlayout_->addWidget(view_);
    hlayout_->addWidget(group_box_settings_);

    widget_->setLayout(hlayout_);

    layout()->addWidget(widget_);
}

void GuiViewBehaviors::reset()
{
}

// TODO replace hard-coded values
void GuiViewBehaviors::update()
{
    QColor color = Qt::white;

    float x = 0;
    float y = 0;

    scene_->clear();

    GuiDiagramItem* item2 = new GuiDiagramItem(GuiDiagramItem::Arbitration);

    if (name_.size() == 0)
        item2->setPos(QPointF(250.0, 0.0));
    else if (name_.size() == 1)
        item2->setPos(QPointF(250.0, 50.0));
    else
        item2->setPos(QPointF(250.0, ((name_.size() - 1) * 75.0)));

    item2->setBrush(color);
    item2->setPen(QPen(Qt::black, 8));
    scene_->addItem(item2);

    for (unsigned int iBehavior = 0; iBehavior < name_.size(); iBehavior++)
    {
        GuiDiagramItem* item = new GuiDiagramItem(GuiDiagramItem::Behavior);

        if (!activation_[iBehavior])
            item->setPen(QPen(Qt::red, 10));
        else
            item->setPen(QPen(Qt::black, 8));

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

        if (!activation_[iBehavior])
            text->setDefaultTextColor(Qt::red);
        else
            text->setDefaultTextColor(Qt::black);

        text->setFont(QFont("Courier", 12, QFont::Bold));
        text->setTextInteractionFlags(Qt::NoTextInteraction);
        text->setZValue(1000.0);
        text->setPos(QPointF(x-100.0, y-25));
        text->setPlainText(QString::fromStdString(name_[iBehavior]));
        text->setTextWidth(item->polygon().boundingRect().width());

        scene_->addItem(text);

        y += 150.0;
    }

    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
}

void GuiViewBehaviors::activate(bool checked)
{
}

