#include "improved_slider.h"

#include <QStyle>
#include <QStyleOptionSlider>

#include "lib/print.h"

using namespace gui;

ImprovedSlider::ImprovedSlider(QWidget *parent) :
    QSlider(parent)
{

}


void ImprovedSlider::mousePressEvent(QMouseEvent *event) {
  QStyleOptionSlider opt;
  initStyleOption(&opt);
  QRect sr = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

  if (event->button() == Qt::LeftButton &&
      !sr.contains(event->pos())) {
    int newVal;
    if (orientation() == Qt::Vertical) {
       double halfHandleHeight = (0.5 * sr.height()) + 0.5;
       int adaptedPosY = height() - event->y();
       if ( adaptedPosY < halfHandleHeight )
             adaptedPosY = halfHandleHeight;
       if ( adaptedPosY > height() - halfHandleHeight )
             adaptedPosY = height() - halfHandleHeight;
       double newHeight = (height() - halfHandleHeight) - halfHandleHeight;
       double normalizedPosition = (adaptedPosY - halfHandleHeight)  / newHeight ;

       newVal = minimum() + (maximum()-minimum()) * normalizedPosition;
    } else {
        double halfHandleWidth = (0.5 * sr.width()) + 0.5;
        int adaptedPosX = event->x();
        if ( adaptedPosX < halfHandleWidth )
              adaptedPosX = halfHandleWidth;
        if ( adaptedPosX > width() - halfHandleWidth )
              adaptedPosX = width() - halfHandleWidth;
        double newWidth = (width() - halfHandleWidth) - halfHandleWidth;
        double normalizedPosition = (adaptedPosX - halfHandleWidth)  / newWidth ;

        newVal = minimum() + ((maximum()-minimum()) * normalizedPosition);
    }

    int pos = newVal;
    if (invertedAppearance()) {
        pos = maximum() - newVal;
    }

    setSliderPosition(pos);
    print "slider click";
    sliderMoved(pos);
    QSlider::mousePressEvent(event);

    // event->accept();
  }
  else {
        QSlider::mousePressEvent(event);
  }
}

// ImprovedSlider::~ImprovedSlider() {

// }