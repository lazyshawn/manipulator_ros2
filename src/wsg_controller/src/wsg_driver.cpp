#include "wsg_controller/wsg_driver.h"

/*************************************************************************
 * @class: WSGGripper
 *************************************************************************/
WSGGripper::WSGGripper() {}

WSGGripper::WSGGripper(std::string targetIP_, unsigned int portIndex_) {
  portIndex = portIndex_;
  targetIP = targetIP_;
  clientFd = this->active(targetIP_, portIndex);
}

WSGGripper::~WSGGripper() {
  disable();
  close(clientFd);
  std::cout << "Termination" << std::endl;
}

int WSGGripper::active(std::string targetIP_, unsigned int portIndex_){
  portIndex = portIndex_;
  targetIP = targetIP_;
  // cmd置零
  memset(cmd, 0, sizeof(cmd));
  clientFd = socket(AF_INET, SOCK_STREAM, 0);
  /* 设置 sockaddr_in 结构体 */
  bzero(&targetAddr, sizeof(targetAddr));
  targetAddr.sin_family = AF_INET;
  targetAddr.sin_port = htons(portIndex);
  targetAddr.sin_addr.s_addr = inet_addr(targetIP.c_str());
  /* 连接夹爪 */
  msgFd = connect(clientFd, (struct sockaddr *)&targetAddr, sizeof(targetAddr));
  if (msgFd + 1 == 0) {
    std::cout << "Connect WSGGripper failed!" << std::endl;
  }
  home(1);
  return clientFd;
}

void WSGGripper::send_cmd(std::string str) {
  if (send(clientFd, str.c_str(), strlen(str.c_str()), 0) == -1) {
    std::cout << "Client send failed" << std::endl;
    return;
  }
  memset(msgRecv, 0, sizeof(msgRecv));
  int recvFlag = recv(clientFd, msgRecv, sizeof(msgRecv), 0);
  if (recvFlag < 0) {printf("ERROR: WSGGripper::send_cmd\n");}
}

void WSGGripper::home(bool direction) {
  sprintf(cmd, "home(%d)\n", direction);
  send_cmd(cmd);
}

float WSGGripper::read_pos() {
  sprintf(cmd, "POS?\n");
  send_cmd(cmd);
  // pos = std::atof(&msgRecv[12])/1000;
  // pos = std::atof(&msgRecv[12]);
  printf("%s\n",msgRecv);
  // printf("%f\n", pos);
  return pos;
}

void WSGGripper::move(float position, float speed) {
  if (position > 110) {
    position = 110;
    printf("Warning: WSGGripper\n");
  } else if (position < 0) {
    position = 0;
  }
  if (speed > 420) {
    speed = 420;
  } else if (position < 0) {
    speed = 0;
  }
  sprintf(cmd, "move(%f,%f)\n", position, speed);
  send_cmd(cmd);
}

void WSGGripper::move(float position) {
  move(position, 40);
}

void WSGGripper::grip() {
  sprintf(cmd, "GRIP()\n");
  send_cmd(cmd);
}
void WSGGripper::grip(float force) {
  if (force > 80) {
    force = 80;
  } else if (force < 0) {
    force = 0;
  }
  sprintf(cmd, "GRIP(%f)\n", force);
  send_cmd(cmd);
}
void WSGGripper::grip(float force, float position) {
  if (force > 80) {
    force = 80;
  } else if (force < 0) {
    force = 0;
  }
  if (position > 110) {
    position = 110;
  } else if (position < 0) {
    position = 0;
  }
  sprintf(cmd, "GRIP(%f,%f)\n", force, position);
  send_cmd(cmd);
}
void WSGGripper::grip(float force, float position, float speed) {
  if (force > 80) {
    force = 80;
  } else if (force < 0) {
    force = 0;
  }
  if (position > 110) {
    position = 110;
  } else if (position < 0) {
    position = 0;
  }
  if (speed > 420) {
    speed = 420;
  } else if (position < 0) {
    speed = 0;
  }
  sprintf(cmd, "GRIP(%f,%f,%f)\n", force, position, speed);
  send_cmd(cmd);
}

void WSGGripper::release() {
  sprintf(cmd, "RELEASE()\n");
  send_cmd(cmd);
}
void WSGGripper::release(float re_position) {
  sprintf(cmd, "RELEASE(%f)\n", re_position);
  send_cmd(cmd);
}
void WSGGripper::release(float re_position, float speed) {
  sprintf(cmd, "RELEASE(%f,%f)\n", re_position, speed);
  send_cmd(cmd);
}

void WSGGripper::disable() {
  sprintf(cmd, "BYE()\n");
  send_cmd(cmd);
}

void WSGGripper::read_recv() { printf("%s\n", msgRecv); }

void WSGGripper::read_config() {
  send_cmd("GRIPCFG[0][0]?\n");
  printf("%s\n", msgRecv);
}

