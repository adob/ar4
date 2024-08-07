#include <visp3/core/vpHomogeneousMatrix.h>

#include "kinematics.h"

#include "robot.h"
#include "lib/base.h"
#include <math.h>

#include "lib/exceptions.h"
#include "lib/io/io_stream.h"
#include "lib/math/matrix.h"
#include "lib/print.h"
#include "lib/str.h"
#include "lib/types.h"

using namespace lib;
using namespace robot;

// Z' Y'' X''' rotation system

// Z'  Yaw   - turning head left/right
// Y'' Pitch -

// static Matrix matrix_multiply(Matrix4x4 const& m1, Matrix4x4 const& m2) {
//     Matrix4x4 result = {};

//     for (int i = 0; i < 4; i++) {
//         for (int j = 0; j < 4; j++) {
//             for (int k = 0; k < 4; k++) {
//                 result.m[i][j] += m1.m[i][k] * m2.m[k][j];
//             }
//         }
//     }

//     return result;
// }

static constexpr float64 deg2rad(float64 degs) {
    return degs * (M_PI / 180.0);
}

static constexpr float64 rad2deg(float64 rads) {
    return rads * (180.0 / M_PI);
}

static math::Matrix4x4 dh_transform(float64 joint_angle, JointCfg const& cfg) {
    float64 theta = deg2rad(joint_angle + cfg.theta_offset_degrees);
    float64 alpha = cfg.alpha;
    float64 a = cfg.a;
    float64 d = cfg.d;

    return {{
        {cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),  a * cos(theta)},
        {sin(theta), cos(theta) * cos(alpha),  -cos(theta) * sin(alpha), a * sin(theta)},
        {0,          sin(alpha),               cos(alpha),               d},
        {0,          0,                        0,                        1},
    }};
}

// https://github.com/robotology-legacy/mex-wholebodymodel/blob/master/mex-wholebodymodel/matlab/utilities/+WBM/+utilities/rotm2eul.m
void robot::rotm2eul_zyx(math::Matrix3x3 const& m, float64 *z, float64 *y, float64 *x) {

    if (m[2, 0] < 1) {
        *z = atan2(m[1, 0], m[0, 0]);  // z; OK
        *y = asin(-m[2, 0]); // y; OK
        *x = atan2(m[2, 1], m[2, 2]); // x; OK
    } else {  // case 2: if r31 == -1
        // Gimbal lock: There is not a unique solution for
        // theta_x + theta_z = atan2(-r23, r22), by convention, set theta_x = 0.
        *z = -atan2(m[1, 2], m[1, 1]);
        *y = -M_PI/2;
        *x = 0;
    }
}

math::Matrix4x4 robot::eul2mat(Pose const& pose) {
    float64 x = pose[0];
    float64 y = pose[1];
    float64 z = pose[2];

    float64 rz = deg2rad(pose[3]);
    float64 ry = deg2rad(pose[4]);
    float64 rx = deg2rad(pose[5]);

    return {{
        {cos(rz)*cos(ry), cos(rz)*sin(ry)*sin(rx) - sin(rz)*cos(rx), cos(rz)*sin(ry)*cos(rx) + sin(rz)*sin(rx), x},
        {sin(rz)*cos(ry), sin(rz)*sin(ry)*sin(rx) + cos(rz)*cos(rx), sin(rz)*sin(ry)*cos(rx) - cos(rz)*sin(rx), y},
        {-sin(ry),        cos(ry)*sin(rx),                           cos(ry)*cos(rx),                           z},
        {0,               0,                                         0,                                         1},
    }};
}

robot::Pose robot::mat2eul(math::Matrix4x4 const& m) {
    double x = m[0, 3];
    double y = m[1, 3];
    double z = m[2, 3];

    float64 ry = atan2(-m[2, 0], sqrt(pow(m[2, 1], 2) + pow(m[2, 2], 2)));
    float64 rz = atan2(m[1, 0] / cos(ry), m[0, 0] / cos(ry));
    float64 rx = atan2(m[2, 1] / cos(ry), m[2, 2] / cos(ry));

    return {
        x,
        y,
        z,
        rad2deg(rz),
        rad2deg(ry),
        rad2deg(rx)
    };
}

vpHomogeneousMatrix robot::to_vp(math::Matrix4x4 const& m) {
    vpHomogeneousMatrix out;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i][j] = m[i, j];
        }
    }
    
    return out;
}

math::Matrix4x4 robot::from_vp(vpHomogeneousMatrix const& m) {
    math::Matrix4x4 out;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i, j] = m[i][j];
        }
    }

    return out;
}

math::Matrix4x4 robot::frame2mat(KDL::Frame const& f)
{
    math::Matrix4x4 out;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i, j] = f(i, j);
        }
    }

    return out;
}

KDL::Frame robot::mat2frame(math::Matrix4x4 const& m)
{
    // KDL::Frame out;

    // out.Make4x4((double*) m.data);

    return KDL::Frame(KDL::Rotation(m[0, 0], m[0, 1], m[0, 2], m[1, 0], m[1, 1], m[1, 2], m[2, 0],
                          m[2, 1], m[2, 2]),
        KDL::Vector(m[0, 3], m[1, 3], m[2, 3]));

    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         out(i, j) = m[i, j];
    //     }
    // }

    // return out;
}

Pose robot::forward_kinematics(RobotCfg const& cfg, Pose const& pose) {
    math::Matrix4x4 acc = forward_kinematics_mat(cfg, pose);

    double x = acc[0, 3];
    double y = acc[1, 3];
    double z = acc[2, 3];
    // rotation
    // print "x y z", x, y, z;

    float64 ry = atan2(-acc[2, 0], sqrt(pow(acc[2, 1], 2) + pow(acc[2, 2], 2)));
    float64 rz = atan2(acc[1, 0] / cos(ry), acc[0, 0] / cos(ry));
    float64 rx = atan2(acc[2, 1] / cos(ry), acc[2, 2] / cos(ry));

    // print "rz", rad2deg(rz);
    // print "ry", rad2deg(ry);
    // print "rx", rad2deg(rx);

    return {
        x,
        y,
        z,
        rad2deg(rz),
        rad2deg(ry),
        rad2deg(rx)
    };
}

math::Matrix4x4 robot::forward_kinematics_mat(RobotCfg const& cfg, Pose const& pose) {
    math::Matrix4x4 acc = dh_transform(pose[0], *cfg.joints[0]);

    for (int i = 1; i < 6; i++) {
        // print "dh %d\n%s" % (i+1), dh_transform(pose[i], *cfg.joints[i]);
        acc = acc * dh_transform(pose[i], *cfg.joints[i]);
        // print "acc\n", acc;
    }

    return acc;
}


static float64 get_j1_angle(math::Matrix4x4 const& m05) {
    float64 x = m05[0, 3];
    float64 y = m05[1, 3];
    // print "get_j1_angle x %d; y %d" % x, y;

    // if (y > 0) {
    //     return -rad2deg(atan(x / y));
    // }

    // if (y < 0) {
    //     if (x > 0) {
    //         return 180 + rad2deg(atan(x / y));
    //     }

    //     return 180 - rad2deg(atan(x / y));
    // }

    // // y == 0
    // if (x > 0) {
    //     return -90;
    // }

    // return 90;

    if (x > 0) {
        return rad2deg(atan(y / x));
    }

    if (x < 0) {
        if (y < 0) {
            return -180 + rad2deg(atan(y / x));
        }

        return 180 + rad2deg(atan(y / x));
    }

    // x == 0
    if (y < 0) {
        return -90;
    }

    return 90;
}

static float64 get_j2_angle(RobotCfg const& cfg, float64 r05x, float64 l1, float64 l2, float64 l3, float64 l4) {

    float64 T17 = l2;
    float64 T18 = l3;
    float64 theta_b = rad2deg(atan(l1 / l4));
    // print "theta_b", theta_b;
    float64 theta_c = rad2deg(acos((pow(cfg.j2.a, 2) + pow(T17, 2) - pow(T18, 2)) / (2*cfg.j2.a*T17)));
    // print "theta_c", theta_c;

    float64 T19 = l4;
    float64 T20 = theta_b;
    float64 T21 = theta_c;

    if (r05x > cfg.j1.a) {
        if(T19>0) {
            return T20-T21;
        } else {
            return T20-T21+180;
        }
    } else {
        return -(T20+T21);
    }
}

static float64 get_j3_angle(RobotCfg const& cfg, float64 l2, float64 l3) {

    // https://github.com/Jmeyer1292/opw_kinematics/blob/master/include/opw_kinematics/opw_kinematics_impl.h

    float64 theta_d = rad2deg(acos((pow(l3, 2) + pow(cfg.j2.a, 2) - pow(l2, 2)) / (2*l3*cfg.j2.a)));
    // print "theta_d", theta_d;
    float64 theta_e = rad2deg(atan(cfg.j3.a/cfg.j4.d));
    // print "theta_e", theta_e;
    float64 T22 = theta_d;
    float64 T23 = theta_e;

    return -(T22+T23)+90;
}

// static float64 get_j4_angle() {
//     float64 Q4 = j5_angle;

//     if (Q8 > 0) {
//         -DEGREES(ATAN2(Z33,-Z34));
//     }

//     return -DEGREES(ATAN2(-Z33,Z34));
// }

// static float64 get_j5_angle(Matrix3x3 m36) {
//     float64 r33 = m36[2, 2];

//     return rad2deg(atan2(sqrt(1 - pow(r33, 2)), r33));
// }

// static float64 get_j4_angle(float64 j5_angle, Matrix3x3 const& m36) {
//     float64 r13 = m36[0, 2];
//     float64 r23 = m36[1, 2];

//     if (j5_angle > 0) {
//         return -rad2deg(atan2(-r23, r13));
//     }

//     return -rad2deg(atan2(r23, -r13));
// }

// static float64 get_j6_angle(float64 j5_angle, Matrix3x3 const& m36) {
//     float64 r31 = m36[2, 0];
//     float64 r32 = m36[2, 1];

//     if (j5_angle > 0) {
//         return rad2deg(atan2(r32, -r31));
//     }

//     return rad2deg(atan2(-r32, r31));
// }

static bool is_near(float64 a, float64 b, float64 theta = 1e-6) {
    return std::abs(a - b) < theta;
}

// https://www.researchgate.net/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist
// https://opentextbooks.clemson.edu/wangrobotics/chapter/inverse-kinematics/
// https://www.geometrictools.com/Documentation/EulerAngles.pdf
void robot::ik_wrist(math::Matrix3x3 const& m36, JointsConfiguration *joints) {
    // print "m36\n%s" % m36;
    // float64 j4, j5, j6;

    float64 r33 = m36[2, 2];
    // float64 r23 = m36[1, 2];
    // float64 r13 = m36[0, 2];
    // float64 r32 = m36[2, 1];
    // float64 r31 = m36[2, 0];
    // float64 r12 = m36[2, 3];
    // float64 r11 = m36[0, 0];

    if(is_near(r33, 1)) {
        // not a unique solution
        // j4 + j6 == atan2(-r12, r11) == atan2(r21, r22)
        float64 r21 = m36[1, 0];
        float64 r22 = m36[1, 1];

        joints->j4 = 0;
        joints->j5 = 0;
        joints->j6 = rad2deg(atan2(r21, r22));

        return;
    }

    if (is_near(r33, -1)) {
        // not a unique solution
        // j6 - j4 == atan2(r21, r22) == atan(r12, -r11)

        // atan2(-r01, r00)
        float64 r21 = m36[1, 0];
        float64 r22 = m36[1, 1];
        joints->j4 = 0;
        joints->j5 = rad2deg(M_PI);
        joints->j6 = rad2deg(atan2(r21, r22));
        return;
    }


    float64 r13 = m36[0, 2];
    float64 r23 = m36[1, 2];
    float64 r31 = m36[2, 0];
    float64 r32 = m36[2, 1];

    // non-singular case (r33 != 0)
    joints->j4 = rad2deg(atan2(r23, r13));
    joints->j5 = rad2deg(acos(r33));
    joints->j6 = rad2deg(atan2(r32, -r31));

    joints->has_alt_wrist = true;
    joints->j4b = rad2deg(atan2(-r23, -r13));
    joints->j5b = -joints->j5;
    joints->j6b = rad2deg(atan2(-r32, r31));
    // print "wrist sol 1:", rad2deg(joints->j4), rad2deg(joints->j5), rad2deg(joints->j6);
    // print "wrist sol 2:", rad2deg(joints->j4b), rad2deg(joints->j5b), rad2deg(joints->j6b);
    // if (std::abs(j4b) < std::abs(j4)) {
    //     j4 = j4b;
    //     j5 = j5b;
    //     j6 = j6b;
    // }


}

math::Matrix3x3 robot::extract_rotm(math::Matrix4x4 const& m) {
    return { {
        { m[0, 0], m[0, 1], m[0, 2] },
        { m[1, 0], m[1, 1], m[1, 2] },
        { m[2, 0], m[2, 1], m[2, 2] },
    } };
}

JointsConfiguration robot::inverse_kinematics_mat(RobotCfg const& cfg, math::Matrix4x4 const& m) {
    return robot::inverse_kinematics(cfg, 
        m[0, 3], m[1, 3], m[2, 3],
        extract_rotm(m));
}

static bool use_alt_wrist(RobotCfg const& rcfg, JointsConfiguration const& jcfg, robot::Pose const& current_joints) {
    if (jcfg.j4b > rcfg.j4.max_angle || jcfg.j4b < rcfg.j4.min_angle) {
        return false;
    }

    if (jcfg.j5b > rcfg.j5.max_angle || jcfg.j5b < rcfg.j5.min_angle) {
        return false;
    }

    if (jcfg.j6b > rcfg.j6.max_angle || jcfg.j6b < rcfg.j6.min_angle) {
        return false;
    }

    if (jcfg.j4 > rcfg.j4.max_angle || jcfg.j4 < rcfg.j4.min_angle) {
        return true;
    }

    if (jcfg.j5 > rcfg.j5.max_angle || jcfg.j5 < rcfg.j5.min_angle) {
        return true;
    }

    if (jcfg.j6 > rcfg.j6.max_angle || jcfg.j6 < rcfg.j6.min_angle) {
        return true;
    }
    
    float64 j4_curr = current_joints[3];
    float64 dist_a = std::abs(j4_curr - jcfg.j4);
    float64 dist_b = std::abs(j4_curr - jcfg.j4b);
    
    return dist_b < dist_a;
}

robot::Pose robot::inverse_kinematics_mat(RobotCfg const& cfg, math::Matrix4x4 const& m, robot::Pose const& current_joints) {
    JointsConfiguration jcfg = inverse_kinematics_mat(cfg, m);
    
    if (use_alt_wrist(cfg, jcfg, current_joints)) {
        return {
            jcfg.j1,
            jcfg.j2,
            jcfg.j3,
            jcfg.j4b,
            jcfg.j5b,
            jcfg.j6b,
        };
    }

    return {
        jcfg.j1,
        jcfg.j2,
        jcfg.j3,
        jcfg.j4,
        jcfg.j5,
        jcfg.j6,
    };
}

JointsConfiguration robot::inverse_kinematics(RobotCfg const& cfg, Pose const& desired_pose) {
    float64 x = desired_pose[0];
    float64 y = desired_pose[1];
    float64 z = desired_pose[2];

    float64 rz = deg2rad(desired_pose[3]);
    float64 ry = deg2rad(desired_pose[4]);
    float64 rx = deg2rad(desired_pose[5]);

    math::Matrix3x3 rotm = {{
        {cos(rz)*cos(ry), cos(rz)*sin(ry)*sin(rx) - sin(rz)*cos(rx), cos(rz)*sin(ry)*cos(rx) + sin(rz)*sin(rx)},
        {sin(rz)*cos(ry), sin(rz)*sin(ry)*sin(rx) + cos(rz)*cos(rx), sin(rz)*sin(ry)*cos(rx) - cos(rz)*sin(rx)},
        {-sin(ry),        cos(ry)*sin(rx),                           cos(ry)*cos(rx)                          },
    }};

    return inverse_kinematics(cfg, x, y, z, rotm);
}

JointsConfiguration robot::inverse_kinematics(RobotCfg const& cfg, float64 x, float64 y, float64 z, math::Matrix3x3 const& rotm) {
    // https://www.hindawi.com/journals/mpe/2021/6647035/
    // https://github.com/Jmeyer1292/opw_kinematics/blob/master/include/opw_kinematics/opw_kinematics_impl.h

    // print "x", x;
    // print "y", y;
    // std::swap(x, y);
    // x = -x;

    //            |c_z*c_y    c_z*s_y*s_x - s_z*c_x    c_z*s_y*c_x + s_z*s_x|
    // R(Theta) = |s_z*c_y    s_z*s_y*s_x + c_z*c_x    s_z*s_y*c_x - c_z*s_x|
    //            |   -s_y                  c_y*s_x                  c_y*c_x|

    // rotation matrix translating 0 to j6
    math::Matrix4x4 m06 = {{
        {rotm[0, 0], rotm[0, 1], rotm[0, 2], x},
        {rotm[1, 0], rotm[1, 1], rotm[1, 2], y},
        {rotm[2, 0], rotm[2, 1], rotm[2, 2], z},
        {0, 0, 0, 1}
    }};
    // print "m06\n%s" % m06;

    // roation matrix translating 6 to j5
    math::Matrix4x4 j6_invert = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, -cfg.j6.d},
        {0, 0, 0, 1},
    }};
    // print "j6_invert\n%s" % j6_invert;

    // roation matrix translating 0 to j5
    math::Matrix4x4 m05 = m06 * j6_invert;
    // print "m05\n%s" % m05;

    // j1, j2, j3
    float64 j1_angle = get_j1_angle(m05);
    // print "j1 angle", j1_angle;

    float64 j1_rads = deg2rad(j1_angle);

    float64 m05x = m05[0, 3];
    float64 m05y = m05[1, 3];
    float64 m05z = m05[2, 3];

    float64 r05x = m05x*cos(-j1_rads) - m05y*sin(-j1_rads);
    // float64 r05y = m05y*cos(-j1_rads) + m05x*sin(-j1_rads);
    // print "r05 at j10 x y", r05x, r05y;

    float64 l1 = abs(r05x-cfg.j1.a);
    // print "l1", l1;
    float64 l2 = sqrt( pow(r05x-cfg.j1.a, 2) + pow(m05z-cfg.j1.d, 2));
    // print "l2", l2;
    float64 l3 = sqrt(pow(cfg.j4.d, 2) + pow(cfg.j3.a, 2));
    // print "l3", l3;
    float64 l4 = m05z - cfg.j1.d;
    // print "l4", l4;

    float64 j2_angle = get_j2_angle(cfg, r05x, l1, l2, l3, l4);
    // print "j2_angle", j2_angle;

    float64 j3_angle = get_j3_angle(cfg, l2, l3);
    // print "j3_angle", j3_angle;

    // j4, j5, j6
    math::Matrix4x4 j1 = dh_transform(j1_angle, cfg.j1);
    math::Matrix4x4 j2 = dh_transform(j2_angle, cfg.j2);
    math::Matrix4x4 j3 = dh_transform(j3_angle, cfg.j3);

    // rotation matrix from 0 to j3
    math::Matrix4x4 m03 = j1 * j2 * j3;
    // print "m03\n%s" % m03;
    math::Matrix3x3 m03_transposed = m03.resized<3>().transposed();
    // print "m03_transposed\n%s" % m03_transposed;

    // rotation from j3 to j6
    // spherical wrist orientation
    math::Matrix3x3 m36 = m03_transposed * m06.resized<3>();
    // print "m36\n%s" % m36;

    // float64 j5_angle = get_j5_angle(m36);
    // print "j5_angle", j5_angle;

    // float64 j4_angle = get_j4_angle(j5_angle, m36);
    // print "j4_angle", j4_angle;

    // float64 j6_angle = get_j6_angle(j5_angle, m36);
    // print "j6_angle", j6_angle;

    // ik_test(m36);

    JointsConfiguration joints = {
        .j1 = j1_angle,
        .j2 = j2_angle,
        .j3 = j3_angle,
    };
    ik_wrist(m36, &joints);

    return joints;
}

Pose robot::reverse_kinematics_wrist(RobotCfg const& cfg, float64 x, float64 y, float64 z) {

    float64 rz = 0;
    float64 ry = 0;
    float64 rx = 0;

    // rotation matrix translating 0 to j6
    math::Matrix4x4 m05 = {{
        {cos(rz)*cos(ry), cos(rz)*sin(ry)*sin(rx) - sin(rz)*cos(rx), cos(rz)*sin(ry)*cos(rx) + sin(rz)*sin(rx), x},
        {sin(rz)*cos(ry), sin(rz)*sin(ry)*sin(rx) + cos(rz)*cos(rx), sin(rz)*sin(ry)*cos(rx) - cos(rz)*sin(rx), y},
        {-sin(ry),        cos(ry)*sin(rx),                           cos(ry)*cos(rx),                           z},
        {0, 0, 0, 1}
    }};

    // j1, j2, j3
    float64 j1_angle = get_j1_angle(m05);
    float64 j1_rads = deg2rad(j1_angle);

    float64 m05x = m05[0, 3];
    float64 m05y = m05[1, 3];
    float64 m05z = m05[2, 3];

    float64 r05x = m05x*cos(-j1_rads) - m05y*sin(-j1_rads);

    float64 l1 = abs(r05x-cfg.j1.a);
    float64 l2 = sqrt( pow(r05x-cfg.j1.a, 2) + pow(m05z-cfg.j1.d, 2));
    float64 l3 = sqrt(pow(cfg.j4.d, 2) + pow(cfg.j3.a, 2));
    float64 l4 = m05z - cfg.j1.d;

    float64 j2_angle = get_j2_angle(cfg, r05x, l1, l2, l3, l4);
    float64 j3_angle = get_j3_angle(cfg, l2, l3);

    return {j1_angle, j2_angle, j3_angle, 0, 0, 0};
}
