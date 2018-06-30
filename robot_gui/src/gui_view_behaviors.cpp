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
        pushbutton->setCheckable(true);
        pushbutton->setChecked(false);
        pushbuttons_.push_back(pushbutton);
        connect(pushbutton, SIGNAL(clicked(bool)), pushbutton, SLOT(setChecked(bool)));
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
    pushbuttons_.clear();
}

// TODO replace hard-coded values
void GuiViewBehaviors::update()
{
    QColor brush_color = Qt::white;
    QColor pen_color = Qt::black;
    unsigned int pen_width = 8;

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

    item2->setBrush(brush_color);
    item2->setPen(QPen(pen_color, pen_width));
    scene_->addItem(item2);

    for (unsigned int iBehavior = 0; iBehavior < name_.size(); iBehavior++)
    {
        GuiDiagramItem* item = new GuiDiagramItem(GuiDiagramItem::Behavior);

        if (!activation_[iBehavior])
        {
            pen_color = Qt::red;
            pen_width = 10;
        }

        else
        {
            pen_color = Qt::black;
            pen_width = 8;
        }

        item->setPen(QPen(pen_color, pen_width));
        item->setBrush(brush_color);
        item->setPos(QPointF(x, y));
        scene_->addItem(item);

        GuiDiagramArrow* arrow = new GuiDiagramArrow(item, item2);

        arrow->setColor(pen_color);
        item->addArrow(arrow);
        item2->addArrow(arrow);
        arrow->setZValue(-1000.0);
        scene_->addItem(arrow);
        arrow->updatePosition();

        GuiDiagramText* text = new GuiDiagramText();
        text->setDefaultTextColor(pen_color);
        text->setFont(QFont("Courier", 12, QFont::Bold));
        text->setTextInteractionFlags(Qt::NoTextInteraction);
        text->setZValue(1000.0);
        text->setPos(QPointF(x-100.0, y-25));
        text->setPlainText(QString::fromStdString(name_[iBehavior]));
        text->setTextWidth(item->polygon().boundingRect().width());

        scene_->addItem(text);

        y += 150.0;

        pushbuttons_[iBehavior]->setChecked(!activation_[iBehavior]);
    }

    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
}
