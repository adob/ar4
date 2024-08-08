#include <memory>
#include <numeric>
#include <opencv2/opencv.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QKeyEvent>
#include <QVideoWidget>
#include <QCommandLineParser>
#include <qboxlayout.h>
#include <qcommandlineoption.h>
#include <qcontainerfwd.h>
#include <qevent.h>
#include <qformlayout.h>
#include <qjsonarray.h>
#include <qjsondocument.h>
#include <qjsonobject.h>
#include <qjsonvalue.h>
#include <qlabel.h>
#include <qnamespace.h>
#include <qobject.h>
#include <qpixmap.h>
#include <qsizepolicy.h>
#include <qwidget.h>
#include <QFormLayout>
#include <QLineEdit>

#include "robot/camera.h"
#include "robot/env.h"
#include "robot/export.h"
#include "tts/tts.h"

#include "lib/base.h"
#include "lib/fmt.h"
#include "lib/time.h"
#include "lib/debug.h"
#include "lib/sync.h"
#include "lib/async.h"
#include "lib/os.h"
#include "robot/kinematics.h"
#include "robot/robot.h"
#include "robot/user_input.h"
#include "robot/task.h"
#include "gui/camera_widget.h"

#include "lib/print.h"

using namespace lib;

struct MainWindow : QWidget {
    int last_key = 0;
    bool foot_pedal_down = false;
    sync::Cond foot_pedal_cond;
    
    bool is_waiting = false;

    sync::Mutex mtx;
    sync::Cond  has_key_cond;

    // CameraWidget side_view;
    // CameraWidget wrist_view;
    // QList<CameraWidget> cameras;

    robot::UserInput user_input;
    QBoxLayout *layout;

    MainWindow() {
        
        layout = new QBoxLayout(QBoxLayout::LeftToRight, this);
        // layout->addWidget(&side_view);
        // layout->addWidget(&wrist_view);
        

        setLayout(layout);
        // setBaseSize(1000, 1000);
        setFixedSize(2000,1000);
        // adjustSize();
    }
    
    void add_camera(robot::CameraFeed *camera_feed) {
        print "adding camera";
        gui::CameraWidget *camera_widget = new gui::CameraWidget(this);
        layout->addWidget(camera_widget);
        
        setLayout(layout);
        
        QObject::connect(camera_feed, &robot::CameraFeed::new_frame, [camera_widget, camera_feed] { 
            camera_widget->on_new_frame(camera_feed); 
        });
        
        this->update();
    }

    void keyPressEvent(QKeyEvent *ev) override {
        if (ev->isAutoRepeat()) {
            return;
        }
        user_input.handle_key_down(ev->key());
    }

    void keyReleaseEvent(QKeyEvent *ev) override {
        if (ev->isAutoRepeat()) {
            return;
        }
        user_input.handle_key_up(ev->key());
    }

} ;


static bool is_accepts(robot::UserInput &input, int *id) {
    for (;;) {
        int key = input.wait_key();
        print "got key", key;

        if (key == Qt::Key_Escape) { // ESC
            return false;
        }
        if (key == Qt::Key_Enter) { // ENTER
            return true;
        }

        if (key >= Qt::Key_0 && key <= Qt::Key_9) {
            *id = key - Qt::Key_0;
            return true;
        }
    }
}

struct Recording {
    String path;
    int    idx;
};

static Recording get_recording(str dataset_dir, int id, error &err) {
    Recording ep;
    ep.idx = 1;

  retry:
    ep.path = fmt::sprintf("%s/recording%d_%d.fb", dataset_dir, id, ep.idx);
    os::stat(ep.path, err.ignore(os::ErrNOENT));
    if (err.handle(os::ErrNOENT)) {
        return ep;
    }

    if (err) {
        return {};
    }

    ep.idx++;
    goto retry;
}

struct Recorder {
    async::Future<void> done;
    bool running = false;
    String dataset_dir;
    robot::TaskConfig task_config;    
    robot::Pose starting_pose;

    robot::RealEnv env;

    Recorder() {
        robot::JointsConfiguration start_pose = robot::inverse_kinematics(robot::ar4, {200, 0, 300, 180, 90, 180});
        this->starting_pose = {start_pose.j1, start_pose.j2, start_pose.j3, start_pose.j4b, start_pose.j5b, start_pose.j6b};
    }

    void loop(MainWindow &win, robot::RealEnv &env) {
        // if (env.cameras.length() > 0) {
        //     win.add_camera()
        //     QObject::connect(env.cameras[0].get(), &CameraFeed::new_frame, &win.wrist_view, &CameraWidget::on_new_frame);
        // }

        for (;;) {
            if (!running) {
                return;
            }

            
            print "new recording...";

            bool ok = env.run(win.user_input, error::panic);
            if (!ok) {
                continue;
            }

            env.move_to_start_position(error::panic);

            int id = 0;
            if (is_accepts(win.user_input, &id)) {
                Recording ep = get_recording(dataset_dir, id, error::panic);

                export_episode(ep.path, env.observations, error::panic);
                tts::say(fmt::sprintf("exported recording %d", ep.idx));
            } else {
                tts::say("rejected");
            }

        }

        print "SHUTTING DOWN";

        env.shutdown();
    }

    void start_async(MainWindow &win) {
        running = true;

        
        env.task_config = task_config;
        env.start_pose = this->starting_pose;
        print "start_pose", env.start_pose;

        print "starting env...";

        // QObject::connect(&env.cam_side, &CameraFeed::new_frame, &win.side_view,
        // &CameraWidget::on_new_frame); QObject::connect(&env.cam_wrist, &CameraFeed::new_frame,
        // &win.wrist_view, &CameraWidget::on_new_frame);
        env.start(error::panic);

        for (std::shared_ptr<robot::CameraFeed>& camera_widget : env.cameras) {
            win.add_camera(camera_widget.get());
        }
        
        done = async::go([&] { loop(win, env); });
    }

    void shutdown() {
        running = false;
        done.await();
    }
};



int main(int argc, char *argv[]) {
    debug::init();
    QApplication app(argc, argv);
    // panic("hello");

    QCommandLineParser parser;
    parser.addOption({"dataset_dir", "dataset dir", "dataset_name"});
    parser.addOption({"starting_pose", "starting pose", "starting_pose"});
    parser.addOption({"task_file", "task file", "task_file"});
    parser.process(app);
    
    // if (dataset_dir == "") {
    //     eprint "--dataset_dir option required";
    //     return 1;
    // }
    
    String task_file = parser.value("task_file").toStdString();
    robot::TaskConfig task_config = {};
    
    String dataset_dir = parser.value("dataset_dir").toStdString();
    if (dataset_dir != "") {
        task_config.dataset_dir = dataset_dir;
    }
    
    if (task_file != "") {
        String data = os::read_file(task_file, error::panic);
        QJsonParseError json_err;
        QJsonObject doc = QJsonDocument::fromJson(QByteArray((char *) data.buffer.data, len(data)), &json_err).object();
        
        if (json_err.error) {
            print "error parsing json";
            return 1;
        }
        
        task_config.dataset_dir = String(doc["dataset_dir"].toString().toStdString());
        print "task config dataset_dir", task_config.dataset_dir;
        
        if (doc.contains("starting_pose")) {
            QJsonArray starting_pose_arr = doc["starting_pose"].toArray();
            if (starting_pose_arr.size() != 6) {
                print "staringt_pose josn bad length";
                return 1;
            }

            robot::Pose starting_pose;
            for (int i = 0; i < 6; i++) {
                starting_pose[i] = starting_pose_arr[i].toDouble();
            }
            print "starting_pose", starting_pose;
            task_config.starting_pose = starting_pose;
        }
        
    
        if (doc.contains("cameras")) {
            QJsonArray cameras_arr = doc["cameras"].toArray();
            for (QJsonValue const& val : cameras_arr) {
                QJsonObject camera_obj = val.toObject();
                
                robot::CameraConfig camera_cfg;
                camera_cfg.device_path = String(camera_obj["device_path"].toString().toStdString());
                print "camera device path", camera_cfg.device_path;
                
                task_config.cameras.append(camera_cfg);
            }
            
        }

    }

    Recorder rec;
    rec.task_config = task_config;
    rec.starting_pose = task_config.starting_pose;

    QString starting_pose_flag = parser.value("starting_pose");
    if (starting_pose_flag != "") {
        QStringList parts = starting_pose_flag.split(",");
        if (parts.length() != 6) {
            eprint "--starting_pose bad format";
            return 1;
        }
        robot::Pose starting_pose;
        for (int i = 0; i < 6; i++) {
            starting_pose[i] = parts[i].toDouble();
        }
        
        print "starting pose", starting_pose;
        rec.starting_pose = starting_pose;
    }
    
    print "using dataset_dir %q" % task_config.dataset_dir;

    MainWindow wind;
    wind.show();

    
    rec.dataset_dir = task_config.dataset_dir;
    rec.start_async(wind);

    app.exec();
    print "SHUTTING DOWN";
    rec.shutdown();

    print "exit";
    return 0;
}