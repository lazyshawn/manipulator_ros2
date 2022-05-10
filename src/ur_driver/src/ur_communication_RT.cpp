#include "ur_driver/ur_communication_RT.h"

UrRealtimeCommunication::UrRealtimeCommunication(
    std::condition_variable &msg_cond, std::string host,
    unsigned int safety_count_max) {
  robot_state_ = new RobotStateRT(msg_cond);
  bzero((char *)&serv_addr_, sizeof(serv_addr_));
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_ < 0) {
    printf("ERROR openting socket\n");
  }
  server_ = gethostbyname(host.c_str());
  if (server_ == NULL) {
    printf("ERROR, no such host\n");
  }
  serv_addr_.sin_family = AF_INET;
  bcopy((char *)server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr,
        server_->h_length);
  serv_addr_.sin_port = htons(30003);
  flag_ = 1;
  setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *)&flag_, sizeof(int));
  setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *)&flag_, sizeof(int));
  setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *)&flag_, sizeof(int));
  fcntl(sockfd_, F_SETFL, O_NONBLOCK);
  connected_ = false;
  keepalive_ = false;
  safety_count_ = safety_count_max + 1;
  safety_count_max_ = safety_count_max;
}

bool UrRealtimeCommunication::start() {
  fd_set writefds;
  struct timeval timeout;

  keepalive_ = true;
  printf("Realtime port: Connecting...\n");
  connect(sockfd_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
  FD_ZERO(&writefds);
  FD_SET(sockfd_, &writefds);
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  select(sockfd_ + 1, NULL, &writefds, NULL, &timeout);
  unsigned int flag_len;
  getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
  if (flag_ < 0) {
    printf("Error connecting to RT port 30003\n");
    return false;
  }
  sockaddr_in name;
  socklen_t namelen = sizeof(name);
  int err = getsockname(sockfd_, (sockaddr *)&name, &namelen);
  if (err < 0) {
    printf("Could not get local IP\n");
    close(sockfd_);
    return false;
  }
  char str[18];
  inet_ntop(AF_INET, &name.sin_addr, str, 18);
  local_ip_ = str;
  comThread_ = std::thread(&UrRealtimeCommunication::run, this);
  return true;
}

void UrRealtimeCommunication::halt() {
  keepalive_ = false;
  comThread_.join();
}

void UrRealtimeCommunication::addCommandToQueue(std::string inp) {
  if (inp.back() != '\n') {
    inp.append("\n");
  }
  if (connected_)
    // int bytes_written = write(sockfd_, inp.c_str(), inp.length());
    write(sockfd_, inp.c_str(), inp.length());
  else
    printf("Could not send command. The robot is not connected! Command is "
           "discarded\n");
}

void UrRealtimeCommunication::setSpeed(double q0, double q1, double q2,
                                       double q3, double q4, double q5,
                                       double acc) {
  char cmd[1024];
  sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.02)\n",
          q0, q1, q2, q3, q4, q5, acc);
  addCommandToQueue((std::string)(cmd));
  if ((q0 != 0.) | (q1 != 0.) | (q2 != 0.) | (q3 != 0.) | (q4 != 0.) | (q5 != 0.)) {
    // If a joint speed is set, make sure we stop it again after some time if
    // the user doesn't
    safety_count_ = 0;
  }
}

void UrRealtimeCommunication::run() {
  uint8_t buf[2048];
  int bytes_read;
  bzero(buf, 2048);
  struct timeval timeout;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sockfd_, &readfds);
  printf("Realtime port: Got connection\n");
  connected_ = true;
  while (keepalive_) {
    while (connected_ && keepalive_) {
      timeout.tv_sec = 0;       // do this each loop as selects modifies timeout
      timeout.tv_usec = 500000; // timeout of 0.5 sec
      select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);
      bytes_read = read(sockfd_, buf, 2048);
      if (bytes_read > 0) {
        setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *)&flag_, sizeof(int));
        robot_state_->unpack(buf);
        if (safety_count_ == safety_count_max_) {
          setSpeed(0., 0., 0., 0., 0., 0.);
        }
        safety_count_ += 1;
      } else {
        connected_ = false;
        close(sockfd_);
      }
    }
    if (keepalive_) {
      // reconnect
      printf("Realtime port: No connection. Is controller crashed? Will try to "
             "reconnect in 10 seconds...\n");
      sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd_ < 0) {
        printf("ERROR opening socket\n");
      }
      flag_ = 1;
      setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *)&flag_,
                 sizeof(int));
      setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *)&flag_,
                 sizeof(int));

      setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *)&flag_,
                 sizeof(int));
      fcntl(sockfd_, F_SETFL, O_NONBLOCK);
      while (keepalive_ && !connected_) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        fd_set writefds;

        connect(sockfd_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
        FD_ZERO(&writefds);
        FD_SET(sockfd_, &writefds);
        select(sockfd_ + 1, NULL, &writefds, NULL, NULL);
        unsigned int flag_len;
        getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
        if (flag_ < 0) {
          printf(
              "Error re-connecting to RT port 30003. Is controller started? "
              "Will try to reconnect in 10 seconds...\n");
        } else {
          connected_ = true;
          printf("Realtime port: Reconnected\n");
        }
      } // while(keepalive_ && !connected_)
    } // if(keepalive_)
  }
  setSpeed(0., 0., 0., 0., 0., 0.);
  close(sockfd_);
}

void UrRealtimeCommunication::setSafetyCountMax(uint inp) {
  safety_count_max_ = inp;
}

std::string UrRealtimeCommunication::getLocalIp() { return local_ip_; }

