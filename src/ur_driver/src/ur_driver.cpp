#include "ur_driver/ur_driver.h"

/* 构造函数: 申明机UR类 */
// rt_msg_cond, msg_cond: 条件变量; host: 主机IP; reverse_port: 通信端口;
// servoj_time: 角度伺服指令的完成时间; safety_count_max: ; max_time_step: ;
// min_payload: ; max_payload: .
UrDriver::UrDriver():REVERSE_PORT_(50007) {}
UrDriver::UrDriver(std::condition_variable &rt_msg_cond,
                   std::condition_variable &msg_cond, std::string host,
                   unsigned int reverse_port, double servoj_time,
                   unsigned int safety_count_max, double max_time_step,
                   double min_payload, double max_payload)
    // 成员变量初始化
    : maximum_time_step_(max_time_step), minimum_payload_(min_payload),
      maximum_payload_(max_payload), REVERSE_PORT_(reverse_port),
      servoj_time_(servoj_time) {
  firmware_version_ = 0;
  reverse_connected_ = false;
  executing_traj_ = false;
  ip_addr_ = host;
  bringup(rt_msg_cond, msg_cond, ip_addr_, safety_count_max);
}

void UrDriver::bringup(std::condition_variable &rt_msg_cond,
             std::condition_variable &msg_cond, std::string host,
             unsigned int safety_count_max) {
  // char buffer[256];
  struct sockaddr_in serv_addr;
  int flag = 1;
  rt_interface_ =
      new UrRealtimeCommunication(rt_msg_cond, host, safety_count_max);
  new_sockfd_ = -1;
  sec_interface_ = new UrCommunication(msg_cond, host);
  /* 创建套接字 */
  // IPv4互联网协议族; 数据传输方式; 通信协议.
  incoming_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (incoming_sockfd_ < 0) {
    printf("ERROR opening socket for reverse communication\n");
  }

  // 将内存块serv_addr清零, 初始化地址信息
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  // htons: 将一个16位数从主机字节顺序转换为网络字节顺序。
  serv_addr.sin_port = htons(REVERSE_PORT_);

  /* 设置套接口(不懂) */
  setsockopt(incoming_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *)&flag,
             sizeof(int));
  // 在处于ESTABLISHED状态下的socket调用close(socket)后继续重用该socket;
  setsockopt(incoming_sockfd_, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));

  /* 分配IP地址和端口号 */
  if (bind(incoming_sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) <
      0) {
    printf("ERROR on binding socket for reverse communication\n");
  }

  /* 将套接字转换为可接收连接状态(监听套接字) */
  // 队列长度为5
  listen(incoming_sockfd_, 5);
}

/* 关节角的三次轨迹插补 */
std::vector<double> UrDriver::interp_cubic(double t, double T,
                                           std::vector<double> p0_pos,
                                           std::vector<double> p1_pos,
                                           std::vector<double> p0_vel,
                                           std::vector<double> p1_vel) {
  /* Returns positions of the joints at time 't' */
  std::vector<double> positions;
  for (unsigned int i = 0; i < p0_pos.size(); i++) {
    double a = p0_pos[i];
    double b = p0_vel[i];
    double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i] - T * p1_vel[i]) / pow(T, 2);
    double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i] + T * p1_vel[i]) / pow(T, 3);
    positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
  }
  return positions;
}

bool UrDriver::doTraj(std::vector<double> inp_timestamps,
                      std::vector<std::vector<double>> inp_positions,
                      std::vector<std::vector<double>> inp_velocities) {
  std::chrono::high_resolution_clock::time_point t0, t;
  std::vector<double> positions;
  unsigned int j = 0;

  if (!UrDriver::uploadProg()) {
    return false;
  }
  executing_traj_ = true;
  t = t0 = std::chrono::high_resolution_clock::now();

  // inp_timestamps[n-1] >= t-t0; 确保还在给定的时间段内。
  while ((inp_timestamps[inp_timestamps.size() - 1] >=
          std::chrono::duration_cast<std::chrono::duration<double>>(t - t0)
              .count()) && executing_traj_) {
    // 找到下一时间戳
    while (inp_timestamps[j] <=
           std::chrono::duration_cast<std::chrono::duration<double>>(t - t0)
           .count() && j < inp_timestamps.size() - 1) {
      j++;
    } // while(inp_timestamps[j])
    // 关节角的三次多项式轨迹插补
    positions = UrDriver::interp_cubic(
        std::chrono::duration_cast<std::chrono::duration<double>>(t - t0)
        .count() - inp_timestamps[j - 1],
        inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
        inp_positions[j], inp_velocities[j - 1], inp_velocities[j]);
    UrDriver::servoj(positions);

    // Oversample with 4 * sample_time(s)
    std::this_thread::sleep_for(
        std::chrono::milliseconds((int)((servoj_time_ * 1000) / 4.)));
    t = std::chrono::high_resolution_clock::now();
  } // while(inp_timestamps[size()-1])

  executing_traj_ = false;
  // Signal robot to stop driverProg()
  UrDriver::closeServo(positions);
  return true;
}

void UrDriver::servoj(std::vector<double> positions, int keepalive) {
  if (!reverse_connected_) {
    printf("UrDriver::servoj called without a reverse connection present\n");
    return;
  }
  int tmp;
  unsigned char buf[28];
  // unsigned int bytes_written;
  for (int i = 0; i < 6; i++) {
    tmp = htonl((int)(positions[i] * MULT_JOINTSTATE_));
    buf[i * 4] = tmp & 0xff;
    buf[i * 4 + 1] = (tmp >> 8) & 0xff;
    buf[i * 4 + 2] = (tmp >> 16) & 0xff;
    buf[i * 4 + 3] = (tmp >> 24) & 0xff;
  }
  tmp = htonl((int)keepalive);
  buf[6 * 4] = tmp & 0xff;
  buf[6 * 4 + 1] = (tmp >> 8) & 0xff;
  buf[6 * 4 + 2] = (tmp >> 16) & 0xff;
  buf[6 * 4 + 3] = (tmp >> 24) & 0xff;
  write(new_sockfd_, buf, 28);
}

void UrDriver::stopTraj() {
  executing_traj_ = false;
  rt_interface_->addCommandToQueue("stopj(10)\n");
}

bool UrDriver::uploadProg() {
  std::string cmd_str;
  char buf[128];
  cmd_str = "def driverProg():\n";

  snprintf(buf, sizeof(buf), "\tMULT_jointstate = %i\n", MULT_JOINTSTATE_);
  cmd_str += buf;

  cmd_str += "\tSERVO_IDLE = 0\n";
  cmd_str += "\tSERVO_RUNNING = 1\n";
  cmd_str += "\tcmd_servo_state = SERVO_IDLE\n";
  cmd_str += "\tcmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  cmd_str += "\tdef set_servo_setpoint(q):\n";
  cmd_str += "\t\tenter_critical\n";
  cmd_str += "\t\tcmd_servo_state = SERVO_RUNNING\n";
  cmd_str += "\t\tcmd_servo_q = q\n";
  cmd_str += "\t\texit_critical\n";
  cmd_str += "\tend\n";
  cmd_str += "\tthread servoThread():\n";
  cmd_str += "\t\tstate = SERVO_IDLE\n";
  cmd_str += "\t\twhile True:\n";
  cmd_str += "\t\t\tenter_critical\n";
  cmd_str += "\t\t\tq = cmd_servo_q\n";
  cmd_str += "\t\t\tdo_brake = False\n";
  cmd_str += "\t\t\tif (state == SERVO_RUNNING) and ";
  cmd_str += "(cmd_servo_state == SERVO_IDLE):\n";
  cmd_str += "\t\t\t\tdo_brake = True\n";
  cmd_str += "\t\t\tend\n";
  cmd_str += "\t\t\tstate = cmd_servo_state\n";
  cmd_str += "\t\t\tcmd_servo_state = SERVO_IDLE\n";
  cmd_str += "\t\t\texit_critical\n";
  cmd_str += "\t\t\tif do_brake:\n";
  cmd_str += "\t\t\t\tstopj(1.0)\n";
  cmd_str += "\t\t\t\tsync()\n";
  cmd_str += "\t\t\telif state == SERVO_RUNNING:\n";

  if (sec_interface_->robot_state_->getVersion() >= 3.1)
    snprintf(buf, sizeof(buf), "\t\t\t\tservoj(q, t=%.4f, lookahead_time=0.03)\n",
            servoj_time_);
  else
    snprintf(buf, sizeof(buf), "\t\t\t\tservoj(q, t=%.4f)\n", servoj_time_);
  cmd_str += buf;

  cmd_str += "\t\t\telse:\n";
  cmd_str += "\t\t\t\tsync()\n";
  cmd_str += "\t\t\tend\n";
  cmd_str += "\t\tend\n";
  cmd_str += "\tend\n";

  snprintf(buf, sizeof(buf), "\tsocket_open(\"%s\", %i)\n", ip_addr_.c_str(),
           REVERSE_PORT_);
  cmd_str += buf;

  cmd_str += "\tthread_servo = run servoThread()\n";
  cmd_str += "\tkeepalive = 1\n";
  cmd_str += "\twhile keepalive > 0:\n";
  cmd_str += "\t\tparams_mult = socket_read_binary_integer(6+1)\n";
  cmd_str += "\t\tif params_mult[0] > 0:\n";
  cmd_str += "\t\t\tq = [params_mult[1] / MULT_jointstate, ";
  cmd_str += "params_mult[2] / MULT_jointstate, ";
  cmd_str += "params_mult[3] / MULT_jointstate, ";
  cmd_str += "params_mult[4] / MULT_jointstate, ";
  cmd_str += "params_mult[5] / MULT_jointstate, ";
  cmd_str += "params_mult[6] / MULT_jointstate]\n";
  cmd_str += "\t\t\tkeepalive = params_mult[7]\n";
  cmd_str += "\t\t\tset_servo_setpoint(q)\n";
  cmd_str += "\t\tend\n";
  cmd_str += "\tend\n";
  cmd_str += "\tsleep(.1)\n";
  cmd_str += "\tsocket_close()\n";
  cmd_str += "\tkill thread_servo\n";
  cmd_str += "end\n";

  rt_interface_->addCommandToQueue(cmd_str);
  return UrDriver::openServo();
}

/* 开始通信 */
bool UrDriver::openServo() {
  struct sockaddr_in cli_addr;
  socklen_t clilen = sizeof(cli_addr);
  /* 受理连接请求 */
  new_sockfd_ = accept(incoming_sockfd_, (struct sockaddr *)&cli_addr, &clilen);
  if (new_sockfd_ < 0) {
    printf("ERROR on accepting reverse communication\n");
    return false;
  }
  return reverse_connected_ = true;
}

/* 中断通信 */
void UrDriver::closeServo(std::vector<double> positions) {
  if (positions.size() != 6)
    UrDriver::servoj(rt_interface_->robot_state_->getQActual(), 0);
  else
    UrDriver::servoj(positions, 0);

  reverse_connected_ = false;
  close(new_sockfd_);
}

bool UrDriver::start() {
  // 获得硬件版本信息
  if (!sec_interface_->start()) return false;
  firmware_version_ = sec_interface_->robot_state_->getVersion();
  // 设置硬件版本信息
  rt_interface_->robot_state_->setVersion(firmware_version_);
  // 建立通信
  if (!rt_interface_->start()) return false;
  ip_addr_ = rt_interface_->getLocalIp();

  std::cout << "Listening on" + ip_addr_ + ":" + std::to_string(REVERSE_PORT_) + "\n";
  return true;
}

void UrDriver::halt() {
  if (executing_traj_) {
    UrDriver::stopTraj();
  }
  sec_interface_->halt();
  rt_interface_->halt();
  close(incoming_sockfd_);
}

void UrDriver::setSpeed(double q0, double q1, double q2, double q3, double q4,
                        double q5, double acc) {
  rt_interface_->setSpeed(q0, q1, q2, q3, q4, q5, acc);
}

std::vector<std::string> UrDriver::getJointNames() { return joint_names_; }

void UrDriver::setJointNames(std::vector<std::string> jn) { joint_names_ = jn; }

void UrDriver::setToolVoltage(unsigned int v) {
  char buf[256];
  snprintf(buf, sizeof(buf), "sec setOut():\n\tset_tool_voltage(%d)\nend\n", v);
  rt_interface_->addCommandToQueue(buf);
  // print_debug(buf); Changed by Jiang
}
void UrDriver::setFlag(unsigned int n, bool b) {
  char buf[256];
  snprintf(buf, sizeof(buf), "sec setOut():\n\tset_flag(%d, %s)\nend\n", n,
          b ? "True" : "False");
  rt_interface_->addCommandToQueue(buf);
  printf("%s", buf);
}
void UrDriver::setDigitalOut(unsigned int n, bool b) {
  char buf[256];
  if (firmware_version_ < 2) {
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_digital_out(%d, %s)\nend\n", n,
            b ? "True" : "False");
  } else if (n > 15) {
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n", n - 16,
            b ? "True" : "False");
  } else if (n > 7) {
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_configurable_digital_out(%d, %s)\nend\n",
            n - 8, b ? "True" : "False");

  } else {
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n", n,
            b ? "True" : "False");
  }
  rt_interface_->addCommandToQueue(buf);
  // print_debug(buf); Changed by Jiang
}
void UrDriver::setAnalogOut(unsigned int n, double f) {
  char buf[256];
  if (firmware_version_ < 2) {
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_analog_out(%d, %1.4f)\nend\n", n, f);
  } else {
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_standard_analog_out(%d, %1.4f)\nend\n", n, f);
  }

  rt_interface_->addCommandToQueue(buf);
  // print_debug(buf);           Changed by Jiang
}

bool UrDriver::setPayload(double m) {
  if ((m < maximum_payload_) && (m > minimum_payload_)) {
    char buf[256];
    snprintf(buf, sizeof(buf), "sec setOut():\n\tset_payload(%1.3f)\nend\n", m);
    rt_interface_->addCommandToQueue(buf);
    printf("%s", buf);
    return true;
  } else
    return false;
}

void UrDriver::setMinPayload(double m) {
  minimum_payload_ = m>0 ? m : 0;
}

void UrDriver::setMaxPayload(double m) { maximum_payload_ = m; }

void UrDriver::setServojTime(double t) {
  servoj_time_ = t>0.008 ? t : 0.008;
}

