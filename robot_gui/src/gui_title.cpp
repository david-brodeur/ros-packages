#include <robot_gui/gui_title.hpp>

#include <QImage>
#include <QPixmap>

using namespace robot_gui;

GuiTitle::GuiTitle()
{
    widget_ = new QWidget();
    layout_ = new QHBoxLayout();
    label_ = new QLabel();

    layout_->addStretch();
    layout_->addWidget(label_);
    layout_->addStretch();

    widget_->setLayout(layout_);
    setWidget(widget_);

    setAllowedAreas(Qt::TopDockWidgetArea);
    setFeatures(QDockWidget::NoDockWidgetFeatures);
}

GuiTitle::~GuiTitle()
{
    delete label_;
    delete layout_;
    delete widget_;
}

void GuiTitle::setPixmap(std::string file_path)
{
    label_->setPixmap(QPixmap::fromImage(QImage(file_path.c_str())));
}

void GuiTitle::setText(std::string title)
{
    // NOT IMPLEMENTED
}
