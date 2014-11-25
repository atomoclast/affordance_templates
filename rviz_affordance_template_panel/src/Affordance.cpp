#include "Affordance.hpp"

#define PIXMAP_SIZE 100
#define CLASS_INDEX 0
#define TRAJECTORY_DATA 1
#define IMAGE 2
#define FILENAME 3

using namespace rviz_affordance_template_panel;

Affordance::Affordance(const string& class_type, const string& image_path, QMap<QString, QVariant> &trajectory_map, const string& filename) {
    QPixmap pixmap(QSize(PIXMAP_SIZE,PIXMAP_SIZE));

    // set pixmap to image if it exists
    pixmap.convertFromImage(QImage(image_path.c_str()));
    if (pixmap.isNull()) {
        // otherwise set it to a green box with the class name overlayed
        QPixmap colorPixmap(QSize(PIXMAP_SIZE,PIXMAP_SIZE));
        colorPixmap.fill(Qt::green);
        QPainter text(&colorPixmap);
        QRectF rect(0, 0, PIXMAP_SIZE, PIXMAP_SIZE);
        text.drawText(rect, Qt::AlignCenter, class_type.c_str());
        this->setPixmap(colorPixmap.scaled(PIXMAP_SIZE, PIXMAP_SIZE));
    } else {
        this->setPixmap(pixmap.scaled(PIXMAP_SIZE, PIXMAP_SIZE));
    }

    this->setFlag(QGraphicsItem::ItemIsSelectable);

    // store the class name associated with the template pixmap item
    // we'll use the class name to instantiate an object template using Pluginlib
    this->setData(CLASS_INDEX, QVariant(class_type.c_str()));
    this->setData(TRAJECTORY_DATA, QVariant(trajectory_map));
    this->setData(IMAGE, QVariant(image_path.c_str()));
    this->setData(FILENAME, QVariant(filename.c_str()));
    
    this->key_ = class_type;
    this->map_ = trajectory_map;
}