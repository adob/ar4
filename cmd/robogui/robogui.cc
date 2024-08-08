#include "robot/camera.h"

#include <qclipboard.h>
#include <qcontainerfwd.h>
#include <qcoreapplication.h>
#include <qdir.h>
#include <qguiapplication.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpMouseButton.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include "flatbuffer/trajectory_generated.h"

#include <flatbuffers/flatbuffer_builder.h>

#include <qapplication.h>
#include <qevent.h>
#include <qnamespace.h>
#include <qpushbutton.h>
#include <qthread.h>
#include <qtmetamacros.h>
#include <qvideoframe.h>
#include <qvideowidget.h>
#include <qwidget.h>
#include <QFileDialog>
#include "robot/arm.h"
#include "lib/time.h"
#include "lib/base.h"
#include "lib/async.h"
#include "robot/kinematics.h"
#include "robot/robot.h"
#include "robot/smoother.h"
#include "verdigris/src/wobjectdefs.h"
#include "verdigris/src/wobjectimpl.h"
#include "tts/tts.h"
#include "utils.h"
#include "lib/debug.h"

#include "ui_record.h"

#include "lib/print.h"



using namespace lib;

const str ar4_device_path = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_13756360-if00";

static constexpr float64 deg2rad(float64 degs) { return degs * (M_PI / 180.0); }

static constexpr float64 rad2deg(float64 rads) { return rads * (180.0 / M_PI); }

struct Env : QObject {
    W_OBJECT(Env);
  public:

    const time::duration control_freq = time::hz(50);
    async::Future<void> done;
    async::Future<void> done2;
    bool running = false;

    robot::AR4Arm arm;
    sync::Mutex mtx;
    robot::Pose current_pose;
    robot::Pose target_pose;
    
    robot::CameraFeed cam_wrist;

    void start() {
        running = true;
        arm.connect(ar4_device_path, error::panic);
        target_pose = robot::forward_kinematics(robot::ar4, arm.get_pose(error::panic));
        done = async::go([&] { loop(); });

        int width = 1920;
        int height = 1080;
        cam_wrist.open("wrist",
            "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_81B986FE-video-index0", width, height, 30,
            -1, error::panic);
        cam_wrist.start_async();
        
    }

    void loop() {
        for (time::LoopTimer timer(control_freq); ; timer.delay()) {
            if (!running) {
                print "loop done";
                return;
            }

            sync::Lock lock(mtx);
            current_pose = arm.get_pose(error::panic);
            lock.unlock();

            pose_updated(current_pose);
        }
    }

    void move_rel(robot::Pose const& rel_pose) {
        sync::Lock lock(mtx);
        // robot::Pose pose = current_pose;
        // for (int i = 0; i < 6; i++) {
        //     pose[i] += rel_pose[i];
        // }

        // arm.move_joints({
        //     .j1 = {.enable = true, .position_degrees = pose[0]},
        //     .j2 = {.enable = true, .position_degrees = pose[1]},
        //     .j3 = {.enable = true, .position_degrees = pose[2]},
        //     .j4 = {.enable = true, .position_degrees = pose[3]},
        //     .j5 = {.enable = true, .position_degrees = pose[4]},
        //     .j6 = {.enable = true, .position_degrees = pose[5]},
        // }, error::panic);

        robot::Pose cart_pose1 = target_pose;
        robot::Pose cart_pose2;
        for (int i = 0; i < 6; i++) {
            cart_pose2[i] = cart_pose1[i] + rel_pose[i];
        }

        print "move_rel %v: %v -> %v" % rel_pose, cart_pose1, cart_pose2;
        target_pose = cart_pose2;

        robot::JointsConfiguration new_pose = robot::inverse_kinematics(robot::ar4, cart_pose2);
        robot::move_to_pose(robot::ar4, arm.client, new_pose, false, false, error::panic);
    }

    void move_cart(robot::Pose const& pose, bool await, error &err) {
        sync::Lock lock(mtx);
        arm.move_cart(pose, await, err);
    }

    void move_abs(robot::Pose const& pose, bool await, error &err) {
        sync::Lock lock(mtx);
        
    }


    void shutdown() {
        if (!running) {
            return;
        }
        running = false;
        done.await();
        print "shutdown returned";
    }

    void replay_trajectory(std::vector<robot::Pose> const& trajectory, bool reverse) {
        sync::Lock lock(mtx);

        if (trajectory.empty()) {
            return;
        }

        robot::Pose start_pose;
        if (!reverse) {
            start_pose = trajectory[0];
        } else {
            start_pose = trajectory[trajectory.size() - 1];
        }
        
        arm.move_joints({
            .j1 = { .enable = true, .position_degrees = start_pose[0]},
            .j2 = { .enable = true, .position_degrees = start_pose[1]},
            .j3 = { .enable = true, .position_degrees = start_pose[2]},
            .j4 = { .enable = true, .position_degrees = start_pose[3]},
            .j5 = { .enable = true, .position_degrees = start_pose[4]},
            .j6 = { .enable = true, .position_degrees = start_pose[5]},
            .await = true,
        }, error::panic);

        robot::MotionSmoother smoother(control_freq);
        time::LoopTimer timer(control_freq);
        ssize i = 1;
        if (reverse) {
            i = trajectory.size() - 2;
        }

        for (;;) {
            if (!reverse && i >= trajectory.size()) {
                break;
            } else if (reverse && i <= -1) {
                break;
            }

            timer.delay();
            current_pose = arm.get_pose(error::panic);
            pose_updated(current_pose);
            
            robot::Pose const& target_pose = trajectory[i];
            smoother.move(arm, target_pose, current_pose, error::panic);

            if (!reverse) {
                i++;
            } else {
                i--;
            }
        }
    }

    void pose_updated(robot::Pose pose) W_SIGNAL(pose_updated, pose)
} ;

W_REGISTER_ARGTYPE(robot::Pose)
W_OBJECT_IMPL(Env)

struct RecorderWidget : QWidget, Ui_Recorder {
    Env env;
    robot::Pose current_pose;
    bool recording_trajectory = false;
    bool is_foot_pedal_down = false;
    std::vector<async::Future<void>> dones;

    std::vector<robot::Pose> trajectory;

    vpHomogeneousMatrix fMe;
    vpHomogeneousMatrix xMf;
    vpHomogeneousMatrix xMe;
    
    vpHomogeneousMatrix eMc;

    RecorderWidget() {
        setupUi(this);
        connect(&env, &Env::pose_updated, this, &RecorderWidget::update_pose);
        connect(replay_btn, &QPushButton::clicked, this, &RecorderWidget::replay_trajectory);
        connect(reverse_play_btn, &QPushButton::clicked, this, &RecorderWidget::replay_trajectory_reverse);
        connect(save_btn, &QPushButton::clicked, this, &RecorderWidget::export_trajectory);
        
        // connect(&env.cam_wrist, &CameraFeed::new_frame, cam_wrist, &CameraWidget::on_new_frame, Qt::QueuedConnection);
        QObject::connect(&env.cam_wrist, &robot::CameraFeed::new_frame, [&] { 
            cam_wrist->on_new_frame(&env.cam_wrist); 
        });
        
        connect(btn_trans_forward, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 1, 0, 0, 0); });
        connect(btn_trans_back, &QPushButton::clicked, this, [&] { move_rel2(0, 0, -1, 0, 0, 0); });
        connect(btn_trans_left, &QPushButton::clicked, this, [&] { move_rel2(0, 1, 0, 0, 0, 0); });
        connect(btn_trans_right, &QPushButton::clicked, this, [&] { move_rel2(0, -1, 0, 0, 0, 0); });
        connect(btn_trans_up, &QPushButton::clicked, this, [&] { move_rel2(-1, 0, 0, 0, 0, 0); });
        connect(btn_trans_down, &QPushButton::clicked, this, [&] { move_rel2(1, 0, 0, 0, 0, 0); });
        
        connect(btn_rot_pitch_up, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 0, -1, 0, 0); });
        connect(btn_rot_pitch_down, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 0, +1, 0, 0); });
        connect(btn_rot_roll_left, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 0, 0, -1, 0); });
        connect(btn_rot_roll_right, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 0, 0, 1, 0); });
        connect(btn_rot_yaw_left, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 0, 0, 0, -1); });
        connect(btn_rot_yaw_right, &QPushButton::clicked, this, [&] { move_rel2(0, 0, 0, 0, 0, 1); });
        
        connect(btn_use_ref_frame, &QPushButton::clicked, this, &RecorderWidget::use_ref_frame);
        connect(btn_xMe_copy,      &QPushButton::clicked, this, &RecorderWidget::xMe_copy);

        connect(btn_snapshot, &QPushButton::clicked, this, &RecorderWidget::snapshot);
    }

    void start_async() {
        vpPoseVector ePc;
        QString path = QDir::homePath() + "/visp/eMc.yaml";
        bool ok = vpPoseVector::loadYAML(path.toStdString(), ePc);
        print "read ePc", ePc[0], ePc[1], ePc[2];
        if (!ok) {
            panic("YAML load failed");
        }
        
        eMc = vpHomogeneousMatrix(ePc);
        
        env.start();
    }

    void update_pose(robot::Pose const& pose) {
        j1_display->display(pose[0]);
        j2_display->display(pose[1]);
        j3_display->display(pose[2]);
        j4_display->display(pose[3]);
        j5_display->display(pose[4]);
        j6_display->display(pose[5]);
        current_pose = pose;

        math::Matrix m = robot::forward_kinematics_mat(robot::ar4, pose);
        robot::Pose cart_pose = robot::mat2eul(m);
        x_display->display(cart_pose[0]);
        y_display->display(cart_pose[1]);
        z_display->display(cart_pose[2]);
        rz_display->display(cart_pose[3]);
        ry_display->display(cart_pose[4]);
        rx_display->display(cart_pose[5]);

        if (recording_trajectory) {
            trajectory.push_back(pose);
        }
        
        fMe = robot::to_vp(m);
        vpPoseVector pose_vp(fMe);
        
        QString fMe_s = QString("%1 %2 %3 %4 %5 %6")
            .arg(pose_vp[0], 0, 'g')
            .arg(pose_vp[1], 0, 'g')
            .arg(pose_vp[2], 0, 'g')
            .arg(pose_vp[3], 0, 'g')
            .arg(pose_vp[4], 0, 'g')
            .arg(pose_vp[5], 0, 'g');
            
        display_fMe->setText(fMe_s);
        
        xMe = xMf * fMe;
        vpPoseVector xMe_pose(xMe);
        QString xMe_s = QString("%1 %2 %3 %4 %5 %6")
            .arg(xMe_pose[0], 0, 'g')
            .arg(xMe_pose[1], 0, 'g')
            .arg(xMe_pose[2], 0, 'g')
            .arg(xMe_pose[3], 0, 'g')
            .arg(xMe_pose[4], 0, 'g')
            .arg(xMe_pose[5], 0, 'g');
        dispaly_xMe->setText(xMe_s);
        
    }

    void closeEvent(QCloseEvent *) override {
        env.shutdown();
        exit(0);
    }

    void keyPressEvent(QKeyEvent *ev) override {    
        if (ev->modifiers() != Qt::NoModifier) {
            return;
        }

        int key = ev->key();
        // print "got key", key;
        
        QKeyEvent *shortcut = new QKeyEvent(QKeyEvent::KeyPress, key, Qt::AltModifier);
        QCoreApplication::postEvent(this, shortcut);

        if (ev->key() == Qt::Key_B && !ev->isAutoRepeat()) {
            foot_pedal_down();
            return;
        }
        return;
        
        
        if (key == Qt::Key_Q) {
            move_rel({-0.1, 0, 0, 0, 0, 0});
            return;
        }
        if (key == Qt::Key_W) {
            move_rel({+0.1, 0, 0, 0, 0, 0});
            return;
        }

        if (key == Qt::Key_A) {
            move_rel({0, -0.1, 0, 0, 0, 0});
            return;
        }
        if (key == Qt::Key_S) {
            move_rel({0, +0.1, 0, 0, 0, 0,});
            return;
        }

        if (key == Qt::Key_Z) {
            move_rel({0, 0, -0.1, 0, 0, 0});
            return;
        }
        if (key == Qt::Key_X) {
            move_rel({0, 0, +0.1, 0, 0, 0});
            return;
        }

        if (key == Qt::Key_E) {
            move_rel({0, 0, 0, -0.1});
            return;
        }
        if (key == Qt::Key_R) {
            move_rel({0, 0, 0, +0.1});
            return;
        }

        if (key == Qt::Key_D) {
            move_rel({0, 0, 0, 0, -0.1});
            return;
        }
        if (key == Qt::Key_F) {
            move_rel({0, 0, 0, 0, +0.1});
            return;
        }

        if (key == Qt::Key_C) {
            move_rel({0, 0, 0, 0, 0, -0.1});
            return;
        }
        if (key == Qt::Key_V) {
            move_rel({0, 0, 0, 0, 0, +0.1});
            return;
        }

        if (ev->isAutoRepeat()) {
            return;
        }

    }

    void keyReleaseEvent(QKeyEvent *ev) override {
        print "key release", ev->key();
        if (ev->isAutoRepeat()) {
            return;
        }
        if (ev->key() == Qt::Key_B && is_foot_pedal_down) {
            foot_pedal_up();
        }
    }

    void foot_pedal_down() {
        print "foot pedal down";
        is_foot_pedal_down = true;

        if (tab_widget->currentWidget() == trajectory_tab) {
            record_trajectory();
            return;
        }
        
        if (tab_widget->currentWidget() == gello_tab) {
            // start_gello();
            return;
        }
        
        robot::Pose cart_pose = robot::forward_kinematics(robot::ar4, current_pose);
        tts::say("waypoint");
        print "joints %v; cart %v" % current_pose, cart_pose;
    }

    void foot_pedal_up() {
        is_foot_pedal_down = false;
        if (recording_trajectory) {
            tts::say("END");
            recording_trajectory = false;
        }
    }

    void move_rel(robot::Pose const& rel_pose) {
        env.move_rel(rel_pose);
    }

    void move_rel2(int x, int y, int z, int pitch, int roll, int yaw) {
        float64 trans_delta = trans_delta_input->value();
        float64 rot_delta = rot_delta_input->value();
        
        float64 dx = x*trans_delta;
        float64 dy = y*trans_delta;
        float64 dz = z*trans_delta;
        
        float64 dpitch = deg2rad(pitch * rot_delta);  // y axis
        float64 droll= deg2rad(roll * rot_delta);     // z axis
        float64 dyaw = deg2rad(yaw * rot_delta);      // x axis

        vpHomogeneousMatrix eMr;
        if (camera_radio->isChecked()) {
            eMr = eMc;
            std::swap(dx, dy);
            dx = -dx;

            dpitch = -dpitch;
            std::swap(dpitch, dyaw);
        }
        vpHomogeneousMatrix rMe = eMr.inverse();

        sync::Lock lock(env.mtx);

        vpHomogeneousMatrix rMrd(vpTranslationVector(dx, dy, dz), vpRotationMatrix(dyaw, dpitch, droll));
        print "rMrd<\n%s>" % rMrd;


        vpHomogeneousMatrix fMe = robot::to_vp(robot::forward_kinematics_mat(robot::ar4, env.arm.get_pose(error::panic)));

        math::Matrix4x4 fMed = robot::from_vp(fMe * eMr * rMrd * rMe);

        robot::JointsConfiguration req = robot::inverse_kinematics_mat(robot::ar4, fMed);

        robot::move_to_pose(robot::ar4, env.arm.client, req, false, false, error::panic);
    }
    
    void use_ref_frame() {
        xMf = fMe.inverse();
    }
    
    void xMe_copy() {
        vpPoseVector xMe_pose(xMe);
        
        QString xMe_s = QString("%1, %2, %3, %4, %5, %6")
            .arg(xMe_pose[0], 0, 'g', 17)
            .arg(xMe_pose[1], 0, 'g', 17)
            .arg(xMe_pose[2], 0, 'g', 17)
            .arg(xMe_pose[3], 0, 'g', 17)
            .arg(xMe_pose[4], 0, 'g', 17)
            .arg(xMe_pose[5], 0, 'g', 17);

        QGuiApplication::clipboard()->setText(xMe_s);
    }

    void record_trajectory() {
        tts::say("BEGIN");
        recording_trajectory = true;
        trajectory.clear();
    }

    void replay_trajectory() {
        if (trajectory.empty()) {
            return;
        }

        dones.emplace_back(sync::go([&] { env.replay_trajectory(trajectory, false); }));
        
    }

    void replay_trajectory_reverse() {
        if (trajectory.empty()) {
            return;
        }

        dones.emplace_back(sync::go([&] { env.replay_trajectory(trajectory, true); }));
        
    }

    void export_trajectory() {
        QString filename = QFileDialog::getSaveFileName(this);
        if (filename == "") {
            return;
        }

        flatbuffers::FlatBufferBuilder fbb;
        flatbuffer::TrajectoryT trajectory_t;

        for (ssize i = 0; i < trajectory.size(); i++) {
            robot::Pose p = trajectory[i];

            flatbuffer::Pose fp(std::array{p[0], p[1], p[2], p[3], p[4], p[5]} );
            trajectory_t.poses.push_back(fp);
        }

        fbb.Finish(flatbuffer::Trajectory::Pack(fbb, &trajectory_t));
        os::write_file(str(filename.toStdString()), str(fbb.GetBufferSpan()), error::panic);
        
    }
    
    void snapshot() {
        robot::CameraFeed::FrameData *frame = env.cam_wrist.current_frame();
        String image_path = get_next_file("image-%d.jpeg", error::panic);
        os::write_file(image_path, str(frame->frame_raw.datastart, frame->frame_raw.dataend-frame->frame_raw.datastart), error::panic);
        print "writen image to", image_path;
        
        robot::Pose current_pose = env.arm.get_pose(error::panic);
        math::Matrix4x4 current_pose_mat = robot::forward_kinematics_mat(robot::ar4, current_pose);
        vpHomogeneousMatrix current_pose_vp_mat = robot::to_vp(current_pose_mat);

        current_pose_vp_mat[0][3] /= 1000.0;
        current_pose_vp_mat[1][3] /= 1000.0;
        current_pose_vp_mat[2][3] /= 1000.0;

        vpPoseVector fPe(current_pose_vp_mat);
        
        String pose_path = get_next_file("pose_fPe_%d.yaml", error::panic);
        bool ok = vpPoseVector::saveYAML(pose_path.std_string(), fPe);
        if (!ok) {
            panic("problem writing pose");
        }
        print "written pose to", pose_path;
    }
} ;

int main(int argc, char *argv[]) {
    debug::init();
    print "thread", QThread::currentThreadId();
    QApplication app(argc, argv);

    RecorderWidget record_widget;
    record_widget.start_async();
    record_widget.show();

    return app.exec();
}