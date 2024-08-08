/********************************************************************************
** Form generated from reading UI file 'record.ui'
**
** Created by: Qt User Interface Compiler version 6.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RECORD_H
#define UI_RECORD_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "gui/camera_widget.h"
#include "gui/improved_slider.h"

QT_BEGIN_NAMESPACE

class Ui_Recorder
{
public:
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    gui::CameraWidget *cam_wrist;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QWidget *cam_side;
    QLabel *info_line;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *play_btn;
    gui::ImprovedSlider *progress_slider;
    QGridLayout *gridLayout;
    QLabel *label_7;
    QLabel *label_6;
    QLabel *label;
    QLCDNumber *rz_display;
    QLabel *label_11;
    QLabel *label_2;
    QLCDNumber *j1_display;
    QLCDNumber *x_display;
    QLabel *label_5;
    QLCDNumber *j5_display;
    QLCDNumber *ry_display;
    QLCDNumber *y_display;
    QLCDNumber *j6_display;
    QLCDNumber *z_display;
    QLCDNumber *j4_display;
    QLCDNumber *j2_display;
    QLabel *label_9;
    QLabel *label_8;
    QLCDNumber *rx_display;
    QLabel *label_10;
    QLCDNumber *j3_display;
    QLabel *label_12;
    QLabel *label_3;
    QLabel *label_4;
    QTabWidget *tab_widget;
    QWidget *waypoints_tab;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout_4;
    QRadioButton *camera_radio;
    QRadioButton *tool_radio;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_5;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_2;
    QPushButton *btn_trans_right;
    QPushButton *btn_trans_back;
    QLabel *label_14;
    QPushButton *btn_trans_left;
    QLabel *label_13;
    QPushButton *btn_trans_forward;
    QDoubleSpinBox *trans_delta_input;
    QPushButton *btn_trans_up;
    QPushButton *btn_trans_down;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_3;
    QPushButton *btn_rot_roll_left;
    QPushButton *btn_rot_roll_right;
    QPushButton *btn_rot_pitch_up;
    QPushButton *btn_rot_yaw_left;
    QPushButton *btn_rot_pitch_down;
    QPushButton *btn_rot_yaw_right;
    QDoubleSpinBox *rot_delta_input;
    QLabel *label_15;
    QLabel *label_16;
    QGridLayout *gridLayout_4;
    QLineEdit *display_fMe;
    QLabel *label_18;
    QLabel *label_17;
    QLineEdit *dispaly_xMe;
    QPushButton *btn_use_ref_frame;
    QToolButton *btn_xMe_copy;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *btn_snapshot;
    QSpacerItem *horizontalSpacer_3;
    QWidget *trajectory_tab;
    QHBoxLayout *horizontalLayout;
    QPushButton *replay_btn;
    QPushButton *reverse_play_btn;
    QPushButton *save_btn;
    QWidget *gello_tab;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *gello_save_btn;
    QPushButton *gello_reverse_btn;
    QPushButton *gello_replay_btn;

    void setupUi(QWidget *Recorder)
    {
        if (Recorder->objectName().isEmpty())
            Recorder->setObjectName("Recorder");
        Recorder->resize(1994, 1215);
        QSizePolicy sizePolicy(QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Recorder->sizePolicy().hasHeightForWidth());
        Recorder->setSizePolicy(sizePolicy);
        verticalLayout_4 = new QVBoxLayout(Recorder);
        verticalLayout_4->setObjectName("verticalLayout_4");
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        groupBox_2 = new QGroupBox(Recorder);
        groupBox_2->setObjectName("groupBox_2");
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName("verticalLayout_2");
        cam_wrist = new gui::CameraWidget(groupBox_2);
        cam_wrist->setObjectName("cam_wrist");
        QSizePolicy sizePolicy1(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(cam_wrist->sizePolicy().hasHeightForWidth());
        cam_wrist->setSizePolicy(sizePolicy1);
        cam_wrist->setMinimumSize(QSize(1280, 720));

        verticalLayout_2->addWidget(cam_wrist);


        horizontalLayout_2->addWidget(groupBox_2);

        groupBox = new QGroupBox(Recorder);
        groupBox->setObjectName("groupBox");
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName("verticalLayout");
        cam_side = new QWidget(groupBox);
        cam_side->setObjectName("cam_side");
        sizePolicy.setHeightForWidth(cam_side->sizePolicy().hasHeightForWidth());
        cam_side->setSizePolicy(sizePolicy);
        cam_side->setMinimumSize(QSize(640, 480));

        verticalLayout->addWidget(cam_side);


        horizontalLayout_2->addWidget(groupBox);


        verticalLayout_4->addLayout(horizontalLayout_2);

        info_line = new QLabel(Recorder);
        info_line->setObjectName("info_line");
        QSizePolicy sizePolicy2(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(info_line->sizePolicy().hasHeightForWidth());
        info_line->setSizePolicy(sizePolicy2);

        verticalLayout_4->addWidget(info_line);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        play_btn = new QPushButton(Recorder);
        play_btn->setObjectName("play_btn");

        horizontalLayout_3->addWidget(play_btn);

        progress_slider = new gui::ImprovedSlider(Recorder);
        progress_slider->setObjectName("progress_slider");
        progress_slider->setOrientation(Qt::Orientation::Horizontal);

        horizontalLayout_3->addWidget(progress_slider);


        verticalLayout_4->addLayout(horizontalLayout_3);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        label_7 = new QLabel(Recorder);
        label_7->setObjectName("label_7");
        QFont font;
        font.setPointSize(20);
        label_7->setFont(font);
        label_7->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_7, 1, 0, 1, 1);

        label_6 = new QLabel(Recorder);
        label_6->setObjectName("label_6");
        label_6->setFont(font);
        label_6->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_6, 0, 10, 1, 1);

        label = new QLabel(Recorder);
        label->setObjectName("label");
        label->setFont(font);
        label->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        label->setWordWrap(false);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        rz_display = new QLCDNumber(Recorder);
        rz_display->setObjectName("rz_display");
        rz_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(rz_display, 1, 7, 1, 1);

        label_11 = new QLabel(Recorder);
        label_11->setObjectName("label_11");
        label_11->setFont(font);
        label_11->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_11, 1, 8, 1, 1);

        label_2 = new QLabel(Recorder);
        label_2->setObjectName("label_2");
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_2, 0, 2, 1, 1);

        j1_display = new QLCDNumber(Recorder);
        j1_display->setObjectName("j1_display");
        j1_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(j1_display, 0, 1, 1, 1);

        x_display = new QLCDNumber(Recorder);
        x_display->setObjectName("x_display");
        x_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(x_display, 1, 1, 1, 1);

        label_5 = new QLabel(Recorder);
        label_5->setObjectName("label_5");
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_5, 0, 8, 1, 1);

        j5_display = new QLCDNumber(Recorder);
        j5_display->setObjectName("j5_display");
        j5_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(j5_display, 0, 9, 1, 1);

        ry_display = new QLCDNumber(Recorder);
        ry_display->setObjectName("ry_display");
        ry_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(ry_display, 1, 9, 1, 1);

        y_display = new QLCDNumber(Recorder);
        y_display->setObjectName("y_display");
        y_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(y_display, 1, 3, 1, 1);

        j6_display = new QLCDNumber(Recorder);
        j6_display->setObjectName("j6_display");
        j6_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(j6_display, 0, 11, 1, 1);

        z_display = new QLCDNumber(Recorder);
        z_display->setObjectName("z_display");
        z_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(z_display, 1, 5, 1, 1);

        j4_display = new QLCDNumber(Recorder);
        j4_display->setObjectName("j4_display");
        j4_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(j4_display, 0, 7, 1, 1);

        j2_display = new QLCDNumber(Recorder);
        j2_display->setObjectName("j2_display");
        j2_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(j2_display, 0, 3, 1, 1);

        label_9 = new QLabel(Recorder);
        label_9->setObjectName("label_9");
        label_9->setFont(font);
        label_9->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_9, 1, 4, 1, 1);

        label_8 = new QLabel(Recorder);
        label_8->setObjectName("label_8");
        label_8->setFont(font);
        label_8->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_8, 1, 2, 1, 1);

        rx_display = new QLCDNumber(Recorder);
        rx_display->setObjectName("rx_display");
        rx_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(rx_display, 1, 11, 1, 1);

        label_10 = new QLabel(Recorder);
        label_10->setObjectName("label_10");
        label_10->setFont(font);
        label_10->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_10, 1, 6, 1, 1);

        j3_display = new QLCDNumber(Recorder);
        j3_display->setObjectName("j3_display");
        j3_display->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout->addWidget(j3_display, 0, 5, 1, 1);

        label_12 = new QLabel(Recorder);
        label_12->setObjectName("label_12");
        label_12->setFont(font);
        label_12->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_12, 1, 10, 1, 1);

        label_3 = new QLabel(Recorder);
        label_3->setObjectName("label_3");
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_3, 0, 4, 1, 1);

        label_4 = new QLabel(Recorder);
        label_4->setObjectName("label_4");
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout->addWidget(label_4, 0, 6, 1, 1);


        verticalLayout_4->addLayout(gridLayout);

        tab_widget = new QTabWidget(Recorder);
        tab_widget->setObjectName("tab_widget");
        QSizePolicy sizePolicy3(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(tab_widget->sizePolicy().hasHeightForWidth());
        tab_widget->setSizePolicy(sizePolicy3);
        waypoints_tab = new QWidget();
        waypoints_tab->setObjectName("waypoints_tab");
        verticalLayout_3 = new QVBoxLayout(waypoints_tab);
        verticalLayout_3->setObjectName("verticalLayout_3");
        groupBox_3 = new QGroupBox(waypoints_tab);
        groupBox_3->setObjectName("groupBox_3");
        horizontalLayout_4 = new QHBoxLayout(groupBox_3);
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        camera_radio = new QRadioButton(groupBox_3);
        camera_radio->setObjectName("camera_radio");
        camera_radio->setChecked(false);

        horizontalLayout_4->addWidget(camera_radio);

        tool_radio = new QRadioButton(groupBox_3);
        tool_radio->setObjectName("tool_radio");
        tool_radio->setChecked(true);

        horizontalLayout_4->addWidget(tool_radio);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);


        verticalLayout_3->addWidget(groupBox_3);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        groupBox_4 = new QGroupBox(waypoints_tab);
        groupBox_4->setObjectName("groupBox_4");
        gridLayout_2 = new QGridLayout(groupBox_4);
        gridLayout_2->setObjectName("gridLayout_2");
        btn_trans_right = new QPushButton(groupBox_4);
        btn_trans_right->setObjectName("btn_trans_right");

        gridLayout_2->addWidget(btn_trans_right, 1, 3, 1, 1);

        btn_trans_back = new QPushButton(groupBox_4);
        btn_trans_back->setObjectName("btn_trans_back");

        gridLayout_2->addWidget(btn_trans_back, 1, 2, 1, 1);

        label_14 = new QLabel(groupBox_4);
        label_14->setObjectName("label_14");

        gridLayout_2->addWidget(label_14, 2, 3, 1, 1);

        btn_trans_left = new QPushButton(groupBox_4);
        btn_trans_left->setObjectName("btn_trans_left");

        gridLayout_2->addWidget(btn_trans_left, 1, 0, 1, 1);

        label_13 = new QLabel(groupBox_4);
        label_13->setObjectName("label_13");
        label_13->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout_2->addWidget(label_13, 2, 0, 1, 1);

        btn_trans_forward = new QPushButton(groupBox_4);
        btn_trans_forward->setObjectName("btn_trans_forward");

        gridLayout_2->addWidget(btn_trans_forward, 0, 2, 1, 1);

        trans_delta_input = new QDoubleSpinBox(groupBox_4);
        trans_delta_input->setObjectName("trans_delta_input");
        trans_delta_input->setValue(1.000000000000000);

        gridLayout_2->addWidget(trans_delta_input, 2, 2, 1, 1);

        btn_trans_up = new QPushButton(groupBox_4);
        btn_trans_up->setObjectName("btn_trans_up");

        gridLayout_2->addWidget(btn_trans_up, 0, 3, 1, 1);

        btn_trans_down = new QPushButton(groupBox_4);
        btn_trans_down->setObjectName("btn_trans_down");

        gridLayout_2->addWidget(btn_trans_down, 0, 0, 1, 1);


        horizontalLayout_5->addWidget(groupBox_4);

        groupBox_5 = new QGroupBox(waypoints_tab);
        groupBox_5->setObjectName("groupBox_5");
        gridLayout_3 = new QGridLayout(groupBox_5);
        gridLayout_3->setObjectName("gridLayout_3");
        btn_rot_roll_left = new QPushButton(groupBox_5);
        btn_rot_roll_left->setObjectName("btn_rot_roll_left");

        gridLayout_3->addWidget(btn_rot_roll_left, 0, 0, 1, 1);

        btn_rot_roll_right = new QPushButton(groupBox_5);
        btn_rot_roll_right->setObjectName("btn_rot_roll_right");

        gridLayout_3->addWidget(btn_rot_roll_right, 0, 2, 1, 1);

        btn_rot_pitch_up = new QPushButton(groupBox_5);
        btn_rot_pitch_up->setObjectName("btn_rot_pitch_up");

        gridLayout_3->addWidget(btn_rot_pitch_up, 0, 1, 1, 1);

        btn_rot_yaw_left = new QPushButton(groupBox_5);
        btn_rot_yaw_left->setObjectName("btn_rot_yaw_left");

        gridLayout_3->addWidget(btn_rot_yaw_left, 1, 0, 1, 1);

        btn_rot_pitch_down = new QPushButton(groupBox_5);
        btn_rot_pitch_down->setObjectName("btn_rot_pitch_down");

        gridLayout_3->addWidget(btn_rot_pitch_down, 1, 1, 1, 1);

        btn_rot_yaw_right = new QPushButton(groupBox_5);
        btn_rot_yaw_right->setObjectName("btn_rot_yaw_right");

        gridLayout_3->addWidget(btn_rot_yaw_right, 1, 2, 1, 1);

        rot_delta_input = new QDoubleSpinBox(groupBox_5);
        rot_delta_input->setObjectName("rot_delta_input");
        rot_delta_input->setValue(1.000000000000000);

        gridLayout_3->addWidget(rot_delta_input, 2, 1, 1, 1);

        label_15 = new QLabel(groupBox_5);
        label_15->setObjectName("label_15");

        gridLayout_3->addWidget(label_15, 2, 2, 1, 1);

        label_16 = new QLabel(groupBox_5);
        label_16->setObjectName("label_16");
        label_16->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        gridLayout_3->addWidget(label_16, 2, 0, 1, 1);


        horizontalLayout_5->addWidget(groupBox_5);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setObjectName("gridLayout_4");
        display_fMe = new QLineEdit(waypoints_tab);
        display_fMe->setObjectName("display_fMe");
        display_fMe->setMinimumSize(QSize(0, 0));

        gridLayout_4->addWidget(display_fMe, 0, 1, 1, 1);

        label_18 = new QLabel(waypoints_tab);
        label_18->setObjectName("label_18");

        gridLayout_4->addWidget(label_18, 1, 0, 1, 1);

        label_17 = new QLabel(waypoints_tab);
        label_17->setObjectName("label_17");

        gridLayout_4->addWidget(label_17, 0, 0, 1, 1);

        dispaly_xMe = new QLineEdit(waypoints_tab);
        dispaly_xMe->setObjectName("dispaly_xMe");

        gridLayout_4->addWidget(dispaly_xMe, 1, 1, 1, 1);

        btn_use_ref_frame = new QPushButton(waypoints_tab);
        btn_use_ref_frame->setObjectName("btn_use_ref_frame");

        gridLayout_4->addWidget(btn_use_ref_frame, 0, 2, 1, 1);

        btn_xMe_copy = new QToolButton(waypoints_tab);
        btn_xMe_copy->setObjectName("btn_xMe_copy");
        QIcon icon(QIcon::fromTheme(QString::fromUtf8("edit-copy")));
        btn_xMe_copy->setIcon(icon);

        gridLayout_4->addWidget(btn_xMe_copy, 1, 2, 1, 1);


        horizontalLayout_5->addLayout(gridLayout_4);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);


        verticalLayout_3->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        btn_snapshot = new QPushButton(waypoints_tab);
        btn_snapshot->setObjectName("btn_snapshot");

        horizontalLayout_6->addWidget(btn_snapshot);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);


        verticalLayout_3->addLayout(horizontalLayout_6);

        tab_widget->addTab(waypoints_tab, QString());
        trajectory_tab = new QWidget();
        trajectory_tab->setObjectName("trajectory_tab");
        horizontalLayout = new QHBoxLayout(trajectory_tab);
        horizontalLayout->setObjectName("horizontalLayout");
        replay_btn = new QPushButton(trajectory_tab);
        replay_btn->setObjectName("replay_btn");

        horizontalLayout->addWidget(replay_btn);

        reverse_play_btn = new QPushButton(trajectory_tab);
        reverse_play_btn->setObjectName("reverse_play_btn");

        horizontalLayout->addWidget(reverse_play_btn);

        save_btn = new QPushButton(trajectory_tab);
        save_btn->setObjectName("save_btn");

        horizontalLayout->addWidget(save_btn);

        tab_widget->addTab(trajectory_tab, QString());
        gello_tab = new QWidget();
        gello_tab->setObjectName("gello_tab");
        horizontalLayout_7 = new QHBoxLayout(gello_tab);
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        gello_save_btn = new QPushButton(gello_tab);
        gello_save_btn->setObjectName("gello_save_btn");

        horizontalLayout_7->addWidget(gello_save_btn);

        gello_reverse_btn = new QPushButton(gello_tab);
        gello_reverse_btn->setObjectName("gello_reverse_btn");

        horizontalLayout_7->addWidget(gello_reverse_btn);

        gello_replay_btn = new QPushButton(gello_tab);
        gello_replay_btn->setObjectName("gello_replay_btn");

        horizontalLayout_7->addWidget(gello_replay_btn);

        tab_widget->addTab(gello_tab, QString());

        verticalLayout_4->addWidget(tab_widget);


        retranslateUi(Recorder);

        tab_widget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Recorder);
    } // setupUi

    void retranslateUi(QWidget *Recorder)
    {
        Recorder->setWindowTitle(QCoreApplication::translate("Recorder", "Form", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("Recorder", "Wrist Cam", nullptr));
        groupBox->setTitle(QCoreApplication::translate("Recorder", "Side Cam", nullptr));
        info_line->setText(QString());
        play_btn->setText(QString());
        label_7->setText(QCoreApplication::translate("Recorder", "X", nullptr));
        label_6->setText(QCoreApplication::translate("Recorder", "J6", nullptr));
        label->setText(QCoreApplication::translate("Recorder", "J1", nullptr));
        label_11->setText(QCoreApplication::translate("Recorder", "rY", nullptr));
        label_2->setText(QCoreApplication::translate("Recorder", "J2", nullptr));
        label_5->setText(QCoreApplication::translate("Recorder", "J5", nullptr));
        label_9->setText(QCoreApplication::translate("Recorder", "Z", nullptr));
        label_8->setText(QCoreApplication::translate("Recorder", "Y", nullptr));
        label_10->setText(QCoreApplication::translate("Recorder", "rZ", nullptr));
        label_12->setText(QCoreApplication::translate("Recorder", "rX", nullptr));
        label_3->setText(QCoreApplication::translate("Recorder", "J3", nullptr));
        label_4->setText(QCoreApplication::translate("Recorder", "J4", nullptr));
        groupBox_3->setTitle(QCoreApplication::translate("Recorder", "Reference Frame", nullptr));
        camera_radio->setText(QCoreApplication::translate("Recorder", "Camera", nullptr));
        tool_radio->setText(QCoreApplication::translate("Recorder", "Tool", nullptr));
        groupBox_4->setTitle(QCoreApplication::translate("Recorder", "Translation", nullptr));
        btn_trans_right->setText(QCoreApplication::translate("Recorder", "Right (&D)", nullptr));
        btn_trans_back->setText(QCoreApplication::translate("Recorder", "Back (&S)", nullptr));
        label_14->setText(QCoreApplication::translate("Recorder", "mm", nullptr));
        btn_trans_left->setText(QCoreApplication::translate("Recorder", "Left (&A)", nullptr));
        label_13->setText(QCoreApplication::translate("Recorder", " \316\224", nullptr));
        btn_trans_forward->setText(QCoreApplication::translate("Recorder", "Forward (&W)", nullptr));
        btn_trans_up->setText(QCoreApplication::translate("Recorder", "Up (&E)", nullptr));
        btn_trans_down->setText(QCoreApplication::translate("Recorder", "Down (&Q)", nullptr));
        groupBox_5->setTitle(QCoreApplication::translate("Recorder", "Rotation", nullptr));
        btn_rot_roll_left->setText(QCoreApplication::translate("Recorder", "Roll \342\206\266 (&U)", nullptr));
        btn_rot_roll_right->setText(QCoreApplication::translate("Recorder", "Roll \342\206\267 (&O)", nullptr));
        btn_rot_pitch_up->setText(QCoreApplication::translate("Recorder", "Pitch \342\206\221 (&I)", nullptr));
        btn_rot_yaw_left->setText(QCoreApplication::translate("Recorder", "Yaw \342\206\220 (&J)", nullptr));
        btn_rot_pitch_down->setText(QCoreApplication::translate("Recorder", "Pitch \342\206\223 (&K)", nullptr));
        btn_rot_yaw_right->setText(QCoreApplication::translate("Recorder", "Yaw \342\206\222 (&L)", nullptr));
        label_15->setText(QCoreApplication::translate("Recorder", "\302\260", nullptr));
        label_16->setText(QCoreApplication::translate("Recorder", " \316\224", nullptr));
        label_18->setText(QCoreApplication::translate("Recorder", "custom \342\206\222 end effector", nullptr));
        label_17->setText(QCoreApplication::translate("Recorder", "fixed \342\206\222 end effector", nullptr));
        btn_use_ref_frame->setText(QCoreApplication::translate("Recorder", "use ref frame", nullptr));
        btn_xMe_copy->setText(QCoreApplication::translate("Recorder", "...", nullptr));
        btn_snapshot->setText(QCoreApplication::translate("Recorder", "Snapshot", nullptr));
        tab_widget->setTabText(tab_widget->indexOf(waypoints_tab), QCoreApplication::translate("Recorder", "Waypoints", nullptr));
        replay_btn->setText(QCoreApplication::translate("Recorder", "Replay Trajectory", nullptr));
        reverse_play_btn->setText(QCoreApplication::translate("Recorder", "Play Reverse Trajectory", nullptr));
        save_btn->setText(QCoreApplication::translate("Recorder", "Save", nullptr));
        tab_widget->setTabText(tab_widget->indexOf(trajectory_tab), QCoreApplication::translate("Recorder", "Trajectory", nullptr));
        gello_save_btn->setText(QCoreApplication::translate("Recorder", "Replay Trajectory", nullptr));
        gello_reverse_btn->setText(QCoreApplication::translate("Recorder", "Play Reverse Trajectory", nullptr));
        gello_replay_btn->setText(QCoreApplication::translate("Recorder", "Save", nullptr));
        tab_widget->setTabText(tab_widget->indexOf(gello_tab), QCoreApplication::translate("Recorder", "GELLO", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Recorder: public Ui_Recorder {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RECORD_H
