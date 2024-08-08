#include "camera.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <qimage.h>
#include <qpixmap.h>
#include <qsize.h>
#include <qthread.h>
#include <qvideoframe.h>
#include <qvideoframeformat.h>
// #include <QThread>
#include "verdigris/src/wobjectdefs.h"
#include "verdigris/src/wobjectimpl.h"

#include "lib/async.h"
#include "lib/sync.h"
#include "lib/time.h"
#include "lib/print.h"

using namespace robot;

// W_REGISTER_ARGTYPE(QVideoFrame)
// W_REGISTER_ARGTYPE(CameraFeed)
W_OBJECT_IMPL(CameraFeed)

void CameraFeed::open(str name, str device_path, int width, int height, int fps, int focus, error &err) {
    this->name = name;
    this->configured_fps = 30; // fps;
    this->width = width;
    this->height = height;
    cap.setExceptionMode(true);

    print "opening";
    if (!cap.open(device_path, cv::CAP_V4L2)) {
        err("failed to open webcam");
        return;
    }
    print "done";

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);  // Width
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);  // Height
    cap.set(cv::CAP_PROP_FPS, fps);  // Frames per second
    print "done 2";

    if (focus >= 0) {
        cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
        cap.set(cv::CAP_PROP_FOCUS, focus);
    } else {
        //cap.set(cv::CAP_PROP_AUTOFOCUS, 1);
    }
    
    print "done 3";
    
    cap.set(cv::CAP_PROP_CONVERT_RGB, false); // don't decode
    print "done 4";

    // current_image = QImage(640, 480, QImage::Format_RGB888);
    qframe1 = QVideoFrame(QVideoFrameFormat(QSize(width, height), QVideoFrameFormat::Format_BGRA8888));
    qframe2 = QVideoFrame(QVideoFrameFormat(QSize(width, height), QVideoFrameFormat::Format_BGRA8888));
    buffer.resize(width*height*4);
}

void robot::rescale_frame(cv::Mat &img_in, cv::Mat &img_out) {
    int in_height = img_in.rows;
    int in_width = img_in.cols;
    
    int desired_width = in_height * 4 / 3; // 960; // 1280 * (4/3)
    int offset_x =  (in_width - desired_width) / 2; // 160;

    cv::Mat resized = img_in(cv::Rect(offset_x, 0, desired_width, in_height));

    time::monotime start = time::mono();
    // print "resize src type", resized.type();
    cv::resize(resized, img_out, cv::Size(640, 480), 0, 0, cv::INTER_AREA);;
    // print "resize took", (time::mono() - start).milliseconds();
}

void CameraFeed::start_async() {
    ready.add(1);
    done_future = async::go([&] { start(error::panic); });
    running = true;
}

void CameraFeed::start(error &err) {
    std::string win_name = name + " cam";
    running = true;
    bool is_ready = false;

    time::monotime start = time::mono();
    int frame_cnt = 0;
    float64 fps = -1;
    
    //moveToThread(QThread::currentThread());
    
    for (;;) {
        
        if (!cap.grab()) {
            err("cv grab failed");
        }
           
        if (!running) {
            print "closing cam", name;
            return;
        }
    
        // bool mapped_new_frame = false;
        {
            sync::Lock lock(frame_mutex);
            if (!cap.retrieve(data_next->frame_raw)) {
                err("cf retrieve failed");
            }

            // cv::imdecode(data_next->frame_raw, cv::IMREAD_COLOR, &frame_tmp);
            // rescale_frame(frame_tmp, data_next->frame_decoded);

            cv::imdecode(data_next->frame_raw, cv::IMREAD_COLOR, &data_next->frame_decoded);
            rescale_frame(data_next->frame_decoded, data_next->frame_rescaled);

            data_next->frame_index++;
            has_next_frame = true;
            
            
            if (buffer.size() != width * height * 4) {
                panic("size mismatch");
            }

            
            
            bool ok = qframe_next->map(QVideoFrame::WriteOnly);
            if (ok) {
                cv::Mat dst(height, width, CV_8UC4, qframe_next->bits(0));
                cv::cvtColor(data_next->frame_decoded, dst, cv::COLOR_BGR2BGRA);
                qframe_next->unmap();
                has_next_video_frame = true;
            }
            
            

            // bool ok = frame.map(QVideoFrame::WriteOnly);
            // if (ok) {
            //     print "map start", frame.width(), frame.height();
            //     if ((dst.dataend - dst.datastart) != frame.width()*frame.height()*4) {
            //         panic("size mismatch");
            //     }
                
            //     print "before", usize(dst.datastart), usize(dst.dataend);
                
                
            //     // 
            //     // first = false;
            //     time::sleep(time::millisecond*20);
                
            //     print "after ", usize(dst.datastart), usize(dst.dataend);
                
            //     print "map end";
            //     mapped_new_frame = true;
            // } else {
            //     print "map failed";
            // }
        }

        if (!is_ready && int(fps) >= configured_fps-5) {
            is_ready = true;

            // ensure data_current points to valid data
            has_next_frame = false;
            std::swap(data_current, data_next);
            
            ready.done();
        }

        frame_cnt++;
        
        time::monotime now = time::mono();
        time::duration elapsed = now - start;

        // static time::monotime last_time;
        // time::duration since_last = now - last_time;
        // last_time = now;
        // print "!!!time since last", since_last.milliseconds();
        // if (since_last.milliseconds() < 1000 && since_last.milliseconds() > 50) {
        //     panic("since last > 50");
        // }
        
        if (elapsed > time::second) {
            fps = float64(frame_cnt) / ( float64(elapsed.nsecs) / 1'000'000'000);
            print "fps", fps;
            frame_cnt = 0;
            start = now;
        }

        emit new_frame();
    }
}

void CameraFeed::await_ready() {
    ready.wait();
}

void CameraFeed::stop() {
    {
        sync::Lock lock(frame_mutex);
        if (!running) {
            return;
        }

        running = false;
    }

    done_future.await();
}


CameraFeed::FrameData *CameraFeed::current_frame() {
    sync::TryLock lock(frame_mutex);
    if (lock.locked && has_next_frame) {
        std::swap(data_current, data_next);
        has_next_frame = false;
    }
    return data_current;
}

QVideoFrame *CameraFeed::current_video_frame() {
    sync::TryLock lock(frame_mutex);
    
    if (lock.locked && has_next_video_frame) {
        std::swap(qframe_current, qframe_next);
        has_next_video_frame = false;
    }
    return qframe_current;
}

CameraFeed::~CameraFeed() {
    stop();
}

// void CameraFeed::paint(QLabel &label) {
//     QPixmap pixmap;
//     {
//         sync::Lock lock(frame_mutex);
//         QImage image(current_preview->data, current_preview->cols, current_preview->rows, QImage::Format_RGB888);
//         pixmap = label.pixmap();
//         pixmap.convertFromImage(image);
//     }
//     label.setPixmap(pixmap);
// }