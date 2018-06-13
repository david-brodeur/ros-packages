#include <robot_gui/gui_view_perceptions.hpp>

#include <QPixmap>

using namespace robot_gui;

GuiViewPerceptions::GuiViewPerceptions() : GuiCentralWidget()
{
    view_ = new QGraphicsView();
    scene_ = new QGraphicsScene(this);
    view_->setScene(scene_);

    layout()->addWidget(view_);
}

GuiViewPerceptions::~GuiViewPerceptions()
{
    delete scene_;
    delete view_;
}

void GuiViewPerceptions::setImage(QImage& image)
{
    image_ = image;
}

void GuiViewPerceptions::init()
{
}

void GuiViewPerceptions::reset()
{
}

void GuiViewPerceptions::update()
{
    QPixmap pixmap = QPixmap::fromImage(image_);

    scene_->clear();
    scene_->setSceneRect(pixmap.rect());
    scene_->addPixmap(pixmap);

    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
}

