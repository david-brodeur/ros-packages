#include <robot_gui/gui_mainwindow.hpp>

#include <QDebug>
#include <QMessageBox>
#include <QPlastiqueStyle>

#include <iostream>

using namespace robot_gui;

GuiMainWindow::GuiMainWindow(std::string gui_name)
{
    gui_name_ = gui_name;
}

GuiMainWindow::~GuiMainWindow()
{
    reset();
}

void GuiMainWindow::init(ParametersGuiMainWindow* parameters)
{
    reset();

    // Style
    style_ = new QPlastiqueStyle();

    // Title
    title_ = new GuiTitle();
    addDockWidget(Qt::TopDockWidgetArea, title_);

    // MenuBar
    menubar_ = menuBar();
    file_menu_ = menubar_->addMenu(QString::fromStdString(parameters->p_file_menu_name));
    edit_menu_ = menubar_->addMenu(QString::fromStdString(parameters->p_edit_menu_name));
    view_menu_ = menubar_->addMenu(QString::fromStdString(parameters->p_view_menu_name));
    param_menu_ = menubar_->addMenu(QString::fromStdString(parameters->p_param_menu_name));
    about_menu_ = menubar_->addMenu(QString::fromStdString(parameters->p_about_menu_name));

    // ToolBar
    view_toolbar_ = addToolBar(QString::fromStdString(parameters->p_view_menu_name));

    // Central widget
    tab_widget_ = new QTabWidget();
    tab_widget_->setTabPosition(QTabWidget::North);
    tab_widget_->setTabShape(QTabWidget::Rounded);

    connect(tab_widget_, SIGNAL(currentChanged(int)), this, SLOT(changeCurrent(int)));

    // Actions
    about_action_ = new QAction("About", this);
    exit_action_ = new QAction("Exit", this);

    about_action_->setCheckable(false);
    exit_action_->setCheckable(false);

    connect(about_action_, SIGNAL(triggered()), this, SLOT(about()));
    connect(exit_action_, SIGNAL(triggered()), this, SLOT(exit()));

    about_menu_->addAction(about_action_);
    file_menu_->addAction(exit_action_);

    view_actions_.clear();

    // Main window
    setWindowTitle(gui_name_.c_str());
    setCentralWidget(tab_widget_);

    // Timer
    update_timer_ = new QTimer(this);
    connect(update_timer_, SIGNAL(timeout()), this, SLOT(update()));
    update_timer_->start(parameters->p_update_time_interval);
}

void GuiMainWindow::reset()
{
    for (std::vector<QAction*>::iterator it = view_actions_.begin(); it != view_actions_.end(); ++it)
        delete *it;
}

void GuiMainWindow::setTitle(std::string title)
{
    title_->setText(title);
}

void GuiMainWindow::setLogo(std::string logo)
{
    title_->setPixmap(logo);
}

void GuiMainWindow::insertTabPage(int index, GuiCentralWidget* page, std::string label, QIcon* icon)
{
    QAction* action;
    std::vector<QAction*>::iterator it;

    tab_widget_->insertTab(index, page, QString::fromStdString(label));

    if (icon != NULL)
    {
        action = new QAction(*icon, QString::fromStdString(label), this);
    }

    else
    {
        action = new QAction(QString::fromStdString(label), this);
    }

    action->setCheckable(true);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(setView(bool)));

    view_menu_->addAction(action);
    view_toolbar_->addAction(action);

    it = view_actions_.begin();
    view_actions_.insert(it + index, action);

    if (view_actions_.size() == 1)
    {
        action->trigger();
    }
}

void GuiMainWindow::about()
{
    QMessageBox::about(this, "About", "ROS graphical tool to illustrate a behavior-based architecture.");
}

void GuiMainWindow::exit()
{
    QApplication::quit();
}

void GuiMainWindow::setView(bool checked)
{
    int index;

    QAction* action = qobject_cast<QAction*>(QObject::sender());

    for (std::vector<QAction*>::iterator it = view_actions_.begin(); it != view_actions_.end(); it++)
    {
        if (*it != action)
        {
            (*it)->setChecked(false);
        }

        else
        {
            (*it)->setChecked(true);
            index = std::distance(view_actions_.begin(), it);
            tab_widget_->setCurrentIndex(index);
        }
    }
}

void GuiMainWindow::update()
{    
    GuiCentralWidget* page = qobject_cast<GuiCentralWidget*>(tab_widget_->currentWidget());
    page->update();
}

void GuiMainWindow::changeCurrent(int index)
{
    if (view_actions_.size() >= 1)
    {
        view_actions_[index]->trigger();
    }
}

const char* GuiMainWindow::getErrorMessage(int return_value)
{
    switch (return_value)
    {
        case SUCCESS:
            return "success";
        case FAILURE:
            return "failure";
        default:
            return "error not handled";
    }
}

