#pragma once

#include <QSlider>
#include <QMouseEvent>
#include <qslider.h>
#include <qwidget.h>

#define W_NO_PROPERTY_MACRO 1
#include "verdigris/src/wobjectdefs.h"


namespace  gui {

    class ImprovedSlider : public QSlider {
        // Q_OBJECT
    protected:
        virtual void mousePressEvent(QMouseEvent *event) override;

    public:
        explicit ImprovedSlider(QWidget *parent = 0);

        // ~ImprovedSlider();
    };

}