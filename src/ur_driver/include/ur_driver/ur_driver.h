/* *****************************************************************
 * UR驱动
 * @ur_driver.cpp
 * *****************************************************************/

#ifndef UR_DRIVER_H_
#define UR_DRIVER_H_

#include "ur_communication.h"
#include "ur_communication_RT.h"
#include <condition_variable>
#include <math.h>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>
#include <chrono>

class UrDriver {
private:
  double maximum_time_step_;
  double minimum_payload_;
  double maximum_payload_;
  const int MULT_JOINTSTATE_ = 1000000;
  const int MULT_TIME_ = 1000000;
  std::vector<std::string> joint_names_;
  std::string ip_addr_;                   // 机械臂IP地址
  const unsigned int REVERSE_PORT_;       // 通信端口
  int incoming_sockfd_;
  int new_sockfd_;                        // 发送控制指令的套接字
  bool reverse_connected_;
  double servoj_time_;                    // 伺服周期
  bool executing_traj_;                   // 执行轨迹规划的标志
  double firmware_version_;               // 机械臂硬件版本号

public:
  UrRealtimeCommunication *rt_interface_; // 
  UrCommunication *sec_interface_;        // 

  // 构造函数
  UrDriver();
  UrDriver(std::condition_variable &rt_msg_cond,
           std::condition_variable &msg_cond, std::string host,
           // 参数默认值
           unsigned int reverse_port = 50007, double servoj_time = 0.016,
           unsigned int safety_count_max = 12, double max_time_step = 0.08,
           double min_payload = 0., double max_payload = 1.);
  void bringup(std::condition_variable &rt_msg_cond,
               std::condition_variable &msg_cond, std::string host,
               unsigned int safety_count_max = 12);
  // 启动UR
  bool start();
  // 暂停UR
  void halt();

  // 速度设置
  void setSpeed(double q0, double q1, double q2, double q3, double q4,
                double q5, double acc = 100.);

  // 角度控制
  bool doTraj(std::vector<double> inp_timestamps,
              std::vector<std::vector<double>> inp_positions,
              std::vector<std::vector<double>> inp_velocities);
  // 角度伺服
  void servoj(std::vector<double> positions, int keepalive = 1);

  // 停止角度控制
  void stopTraj();

  bool uploadProg();
  bool openServo();
  void closeServo(std::vector<double> positions);

  // 关节角的三次轨迹插补
  std::vector<double> interp_cubic(double t, double T,
                                   std::vector<double> p0_pos,
                                   std::vector<double> p1_pos,
                                   std::vector<double> p0_vel,
                                   std::vector<double> p1_vel);

  std::vector<std::string> getJointNames();
  void setJointNames(std::vector<std::string> jn);
  void setToolVoltage(unsigned int v);
  void setFlag(unsigned int n, bool b);
  void setDigitalOut(unsigned int n, bool b);
  void setAnalogOut(unsigned int n, double f);
  bool setPayload(double m);

  void setMinPayload(double m);
  void setMaxPayload(double m);
  void setServojTime(double t);
};

#endif /* UR_DRIVER_H_ */

