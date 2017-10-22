#ifndef GUI_MAINWINDOW_HPP
#define GUI_MAINWINDOW_HPP

#include <robot_gui/gui_title.hpp>
#include <robot_gui/gui_central_widget.hpp>

#include <QAction>
#include <QApplication>
#include <QGraphicsView>
#include <QIcon>
#include <QMainWindow>
#include <QMenuBar>
#include <QObject>
#include <QStyle>
#include <QTabWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolBox>

#include <string>
#include <vector>

/*! 
 *  \brief     GuiMainWindow
 *  \details   This class is the main window for a GUI application.
 *  \author    David Brodeur <David.Brodeur@USherbrooke.ca>
 *  \version   0.0.1
 *  \date      2017
 *  \copyright GNU Public License.
 */

namespace robot_gui
{
    struct ParametersGuiMainWindow
    {
        std::string p_file_menu_name;
        std::string p_edit_menu_name;
        std::string p_view_menu_name;
        std::string p_param_menu_name;
        std::string p_about_menu_name;

        unsigned int p_update_time_interval;
    };

    class GuiMainWindow : public QMainWindow
    {
        Q_OBJECT

        public:

            enum Error { SUCCESS = 0, 
                         FAILURE };

            ///\brief Class constructor.
            ///\param parameters Parameters of the GUI main window.
            ///\param gui_name Name of the GUI application.
            GuiMainWindow(ParametersGuiMainWindow* parameters, std::string gui_name = "/gui/main_window");

            ///\brief Class destructor.
            ~GuiMainWindow();

            ///\brief Get the name of the GUI application.
            ///\return the name of the GUI application.
            std::string name() { return gui_name_; }

            ///\brief Reset the mainwindow
            void reset();

            ///\brief Add a title to the mainwindow.
            ///\param title Title of the application.
            void setTitle(std::string title);

            ///\brief Add a logo to the mainwindow.
            ///\param logo Path to the logo file.
            void setLogo(std::string logo);

            ///\brief Add a menu to the mainwindow.
            ///\param menu Name of the menu.
            void addMenu(std::string menu);

            ///\brief Add a action to a menu.
            void addMenuAction();

            ///\brief Add a toolbar button to the toolbar.
            void addToolBarButton();

            ///\brief Add a statusbar label to the statusbar.
            void addStatusBarLabel();

            ///\brief Add a tab page to the mainwindow's central widget.
            ///\param index Index of the page.
            ///\param page GuiCentralWidget of the page.
            ///\param label Label of the tab.
            ///\param icon Icon of the tab.
            ///\return index of the inserted tab.
            int insertTabPage(int index, GuiCentralWidget* page, std::string label, QIcon* icon = NULL);

            ///\brief Add a toolbar item.
            ///\param index Index of the item.
            ///\param item Widget of the item.
            ///\param label Label of the item.
            ///\param icon Icon of the item.
            ///\return index of the inserted item.
            int insertToolBoxItem(int index, QWidget* item,  std::string label, QIcon* icon = NULL);

            ///\brief Add a statusbar to the mainwindow.
//            void addDockWidget();

            ///\brief Get style
            ///\return style;
            QStyle* style() { return style_; }

            ///\brief Get Error message
            ///\return an error message
            static const char* getErrorMessage(int return_value);

        private Q_SLOTS:

            ///\brief Exit application.
            void exit();

            ///\brief Update the GUI elements.
            ///\param checked True if the action was checked.
            void setView(bool checked);

            ///\brief Update central widget.
            void update();

            ///\brief Change current tag widget.
            void changeCurrent(int index);

        private:

            std::string gui_name_;    /// < Name of the GUI application.

            QStyle* style_;

            QTabWidget* tab_widget_;
            QToolBox* toolbox_;

            GuiTitle* title_;

            QMenuBar* menubar_;
            QMenu* file_menu_;
            QMenu* edit_menu_;
            QMenu* view_menu_;
            QMenu* param_menu_;
            QMenu* about_menu_;

            QToolBar* view_toolbar_;

            QAction* exit_action_;
            std::vector<QAction*> view_actions_;

            QTimer* update_timer_;    /// < Timer to update the GUI elements in the main window.
    };
}

#endif // GUI_MAINWINDOW_HPP
