#ifndef UR_COMMUNICATION_H_
#define UR_COMMUNICATION_H_

#include "robot_state.h"
#include <chrono>              // C++11 时间库
#include <condition_variable>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>

class UrCommunication {
private:
  int pri_sockfd_, sec_sockfd_;
  struct sockaddr_in pri_serv_addr_, sec_serv_addr_;
  struct hostent *server_;
  bool keepalive_;
  std::thread comThread_;
  int flag_;
  void run();

public:
  bool connected_;
  RobotState *robot_state_;

  UrCommunication(std::condition_variable &msg_cond, std::string host);
  bool start();
  void halt();
};

#endif /* UR_COMMUNICATION_H_ */

