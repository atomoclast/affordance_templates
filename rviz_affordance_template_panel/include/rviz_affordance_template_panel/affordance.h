#ifndef AFFORDANCE_HPP
#define AFFORDANCE_HPP

// qt
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QRectF>
#include <QVariant>
#include <QMap>

#include <iostream>

using namespace std;

namespace rviz_affordance_template_panel
{
    class Affordance : public QGraphicsPixmapItem
    {
    public:
        Affordance(const string& class_type, const string& image_path, QMap<QString, QVariant> &trajectory_map, QStringList &display_objects, const string& filename);
        ~Affordance() {}
        string key() const { return key_; }
        QMap<QString, QVariant> map() const { return map_; }
        QStringList objs() const { return objs_; }

    private:
        string key_;
        QMap<QString, QVariant> map_;
        QStringList objs_;
    };
}

#endif