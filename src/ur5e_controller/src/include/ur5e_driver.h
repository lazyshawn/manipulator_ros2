/*************************************************************************
 * @file: ur5e_driver.h
 * Check doc/ur_kinematics/ur.md for more details.
*************************************************************************/
#ifndef UR5E_DRIVER_H
#define UR5E_DRIVER_H
#include "ur_driver/ur_driver.h"

/*************************************************************************
 * ======================================================================
 * ===                   运动学规划 | Kinematics                      ===
 * ======================================================================
*************************************************************************/
#include <vector>
#include <eigen3/Eigen/Dense>

// Modified DH parameters for UR5
#define DH_D1 89.459
#define DH_A2 425.00
#define DH_A3 392.25
#define DH_D4 109.15
#define DH_D5 94.65
#define DH_D6 82.3 

#ifndef rad2deg
#define rad2deg 180.0 / M_PI
#endif
#ifndef deg2rad
#define deg2rad M_PI / 180.0
#endif

typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Matrix<double,4,4> Mat4d;
typedef Eigen::Matrix<double,6,1> Vec6d;
typedef Eigen::Matrix<double,6,6> Mat6d;

typedef std::array<double,6> Arr6d;  // 六维数组
typedef std::array<double,3> Arr3d;
typedef std::array<double,6> THETA;  // 关节角_6
typedef std::array<double,6> TWIST;  // 位姿

// 路径
#define PATH_GOING 0
#define PATH_DONE  1
#define PATH_CLEAR 2
#define PATH_INIT 255
// 伺服模式
#define ANGLE_SERVO 0
#define VELOCITY_SERVO 1
#define SLIDE_SERVO 2
#define PIVOT_SERVO 3
struct PATH {
  PATH () {
    servoMode = ANGLE_SERVO;
    status = PATH_INIT;
    delT = 0.008;
    velocity = {0,0,0,0,0,0};
    freq = 1/delT;
    interpMode = 2;
    fingerPos = -1;
  }

  double beginTime;  // 开始时间
  double freq;       // 插补频率: 总时间的倒数
  unsigned char servoMode;   // 角度伺服标志
  unsigned char status;      // 轨迹完成标志
  unsigned char interpMode;    // 插补模式
  Arr6d orig;        // 插值起点(角度伺服) [mm]
  Arr6d goal;        // 插值终点(角度伺服) [mm]
  Arr6d velocity;    // 速度命令(位姿伺服) [mm/s, rad/s]
  float delT;       // 伺服周期时间
  float fingerPos;  // 手指位置
};

/*************************************************************************
 * @func : calc_joint
 * @brief: 更新各关节角的正余弦值;
 * @param: 关节角向量;
 * @return: 错误(-1) | 正常执行(0);
*************************************************************************/
int calcJnt(THETA q);

/*************************************************************************
 * @brief: 平面运动规划
*************************************************************************/
bool plane_kinematics(std::array<double,3>& state);
THETA plane_invese_kinematics(std::array<double,3>& state);
THETA plane_jacobian(Vec3d twist);
THETA plane_jacobian(Vec3d twist, float time);

/*************************************************************************
 * @func : ur_kinematics
 * @brief: M-DH求解机械臂运动学
 * @param: 旋转矩阵(ori_hnd);
 * @return: 末端位姿向量(6x1)
*************************************************************************/
int ur_kinematics(Mat4d& tranMat);

/*************************************************************************
 * @brief: 计算从关节空间到笛卡尔空间的雅克比矩阵，及其逆矩阵、转置矩阵
 * @param: 雅克比矩阵结构体指针(jcbn);
 * @return: 雅克比矩阵
*************************************************************************/
Mat6d ur_jacobian(void);

Mat3d RPY2RotMat(double alpha, double beta, double gamma);
Vec3d RotMat2AxisAngle(Mat3d rotMat);
Vec3d RotMat2EulerAngle(Mat3d rotMat);
THETA ur_InverseKinematics(Mat4d tranMat, THETA curTheta);
bool swap_joint(double& joint);


/*************************************************************************
 * ======================================================================
 * ===             轨迹插补 | Trajectory Interpolation                ===
 * ======================================================================
*************************************************************************/
// 插值模式
#define INTERP_SIN 0
#define INTERP_5JI 1
#define INTERP_3JI 2
#define INTERP_1JI 3
#define INTERP_STEP 4
#define JNT_PATH_SIN_FF 100
#define JNT_PATH_5JI_FF 200
#define JNT_PATH_3JI_FF 300
#define TRAJECTORY_LENGTH 20

#define SERVO_TIME (0.008)

bool joint_interpolation(double offsetTime, double freq, int interpMode, 
    Arr6d orig, Arr6d displacement, Arr6d& refVal);
bool velocity_interpolation(double offsetTime, double freq, THETA curTheta, 
    THETA& refTheta, Arr6d velocity);

#endif

