#include <robot_gui/gui_central_widget.hpp>

#include <iostream>

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
