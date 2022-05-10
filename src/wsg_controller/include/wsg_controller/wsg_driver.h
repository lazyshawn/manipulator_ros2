#ifndef WSG_DRIVER_H
#define WSG_DRIVER_H

/* Socket通信头文件 */
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <iostream>
#include <arpa/inet.h>

/*************************************************************************
 * @class: WSGGripper
*************************************************************************/
class WSGGripper {
private:
  int clientFd, msgFd;
  sockaddr_in targetAddr;
  float cfg[3], pos;
  std::string targetIP;
  unsigned int portIndex;
  char msgRecv[1024], cmd[32];

public:
  WSGGripper();
  WSGGripper(std::string targetIP_, unsigned int portIndex_);
  ~WSGGripper();
  int active(std::string targetIP_, unsigned int portIndex_);
  void send_cmd(std::string str);
  // direction: positive_110(1), negetive_0(0)
  void home(bool direction);
  float read_pos();
  void disable();
  void read_recv();
  void read_config();
  void move(float position, float speed);
  void move(float position);
  void force(float f);
  void grip();
  void grip(float force);
  void grip(float force, float position);
  void grip(float force, float position, float speed);
  void release();
  void release(float re_position);
  void release(float re_position, float speed);
};

#endif

