#include <robot_gui/gui_central_widget.hpp>

using namespace robot_gui;

GuiCentralWidget::GuiCentralWidget()
{
    layout_ = new QVBoxLayout();
    setLayout(layout_);
}

GuiCentralWidget::~GuiCentralWidget()
{
    delete layout_;
}

void GuiCentralWidget::update()
{
}
