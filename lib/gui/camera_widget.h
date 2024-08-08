#pragma once

#include <qobject.h>
#include <qthread.h>
#include <qvideoframe.h>
#include <qvideosink.h>
#include <qvideowidget.h>
#include <qwidget.h>

#include "robot/camera.h"
#include "lib/print.h"

namespace gui {
    struct CameraWidget : public QVideoWidget {
        
        explicit CameraWidget(QWidget *parent = 0) : QVideoWidget(parent) {}

        void on_new_frame(robot::CameraFeed *cam) {
            
            // time::monotime now = time::mono();
            // static time::monotime prev;
            // time::duration since_last = time::mono() - prev;
            // prev = now;
            // print "time since last", since_last.milliseconds();
            
            // CameraFeed *cam = (CameraFeed*) sender();
            // print "new frame";
            
            QVideoFrame *frame = cam->current_video_frame();
            videoSink()->videoFrameChanged(*frame);
            
            // videoSink()->setVideoFrame(*frame);
        }
    };
}