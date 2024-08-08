#pragma once

#include <qobject.h>
#include <opencv2/videoio.hpp>
#include <qvideoframe.h>
#include <qwindowdefs.h>

#define W_NO_PROPERTY_MACRO 1
#include "verdigris/src/wobjectdefs.h"

#include "lib/async.h"
#include "lib/base.h"
#include "lib/sync.h"
#include "lib/time.h"

namespace robot {
    using namespace lib;

    struct CameraFeed : QObject {

        W_OBJECT(CameraFeed);
    public:

        cv::VideoCapture    cap;
        std::string         name;
        int                 configured_fps = -1;
        bool                running = false;
        sync::WaitGroup     ready;
        async::Future<void> done_future;
        
        int width, height;
        QByteArray          buffer;

        sync::Mutex frame_mutex;
        cv::Mat frame_tmp;
        struct FrameData {
            cv::Mat frame_raw;
            cv::Mat frame_decoded;
            cv::Mat frame_rescaled;
            int     frame_index = -1;
        } data1, data2;
        
        QVideoFrame qframe1, qframe2;
        QVideoFrame *qframe_current = &qframe1,
                    *qframe_next    = &qframe2;

        FrameData *data_current   = &data1,
                *data_next      = &data2;
        bool has_next_frame = false;

        // public api
        void open(str name, str device_path, int width, int height, int fps, int focus, error &err);

        void start_async();
        void await_ready();
        void start(error &err);
        void stop();

        FrameData *current_frame();
        QVideoFrame *current_video_frame();
        bool has_next_video_frame = false;


        sync::Mutex preview_mutex;
        cv::Mat *current_preview = nil;

        // QImage current_image;
        void new_frame() const W_SIGNAL(new_frame);

        ~CameraFeed();

        // void paint(QLabel &label);
    };

    void rescale_frame(cv::Mat &img_in, cv::Mat &img_out);
}