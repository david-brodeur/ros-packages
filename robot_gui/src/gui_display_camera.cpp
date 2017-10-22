#include <robot_gui/gui_display_camera.hpp>

#include <QPixmap>

using namespace robot_gui;

GuiDisplayCamera::GuiDisplayCamera() : GuiCentralWidget()
{
    view_ = new QGraphicsView();
    scene_ = new QGraphicsScene(this);
    view_->setScene(scene_);

    layout()->addWidget(view_);
}

GuiDisplayCamera::~GuiDisplayCamera()
{
    delete scene_;
    delete view_;
}

void GuiDisplayCamera::setImage(QImage& image)
{
    image_ = image;
}

void GuiDisplayCamera::update()
{
    QPixmap pixmap = QPixmap::fromImage(image_);

    scene_->clear();
    scene_->setSceneRect(pixmap.rect());
    scene_->addPixmap(pixmap);

    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
}

