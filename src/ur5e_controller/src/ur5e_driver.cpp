#include "include/ur5e_driver.h"

/*************************************************************************
 * ======================================================================
 * ===                   运动学规划 | Kinematics                      ===
 * ======================================================================
*************************************************************************/
// 各关节角的正余弦值
double c1, s1, c2, s2, c3, s3, c4, s4, c5, s5, c6, s6, c23, s23, c234, s234,
       c34, s34;
// 关节角度
double q2, q3, q4;

/*************************************************************************
 * @func : calcJnt
 * @brief: 更新各关节角的正余弦值;
 * @param: 关节角向量;
*************************************************************************/
int calcJnt(THETA q) {
  if (q.size() != 6) return -1;
  c1 = cos(q[0]);  s1 = sin(q[0]);
  c2 = cos(q[1]);  s2 = sin(q[1]);
  c3 = cos(q[2]);  s3 = sin(q[2]);
  c4 = cos(q[3]);  s4 = sin(q[3]);
  c5 = cos(q[4]);  s5 = sin(q[4]);
  c6 = cos(q[5]);  s6 = sin(q[5]);
  c23 = cos(q[1]+q[2]); s23 = sin(q[1]+q[2]);
  c234 = cos(q[1]+q[2]+q[3]); s234 = sin(q[1]+q[2]+q[3]);
  c34 = cos(q[2]+q[3]); s34 = sin(q[2]+q[3]);
  q2 = q[1]; q3 = q[2]; q4 = q[3];
  return 0;
}

// 平面内运动的正运动学
// state[2] 为指尖连线与x轴正向的夹角(逆时针为正)
bool plane_kinematics(std::array<double,3>& state) {
  state[0] = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 - DH_D6*c234;
  state[1] = DH_D1 - DH_A2*s2 - DH_A3*s23 - DH_D5*c234 + DH_D6*s234;
  state[2] = -q2 -q3 -q4 -M_PI/2;
  return true;
}

// 平面内运动的逆解(降低为三自由度)
THETA plane_invese_kinematics(std::array<double,3>& state) {
  double x = state[0], z = state[1], q = -state[2] -M_PI/2;
  double c234 = cos(q), s234 = sin(q), c3, s3;
  double A = x + DH_D5*s234 + DH_D6*c234;
  double B = -z - DH_D5*c234 + DH_D6*s234 + DH_D1;

  // 限制工作空间后，多解问题已忽略
  THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  theta[2] = acos((A*A+B*B-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  c3 = cos(theta[2]); s3 = sin(theta[2]);
  double delta = DH_A2*DH_A2 + 2*DH_A2*DH_A3*c3 + DH_A3*DH_A3;
  double A2 = (DH_A2+DH_A3*c3)*A + DH_A3*s3*B;
  double B2 = -DH_A3*s3*A        + (DH_A2+DH_A3*c3)*B;
  theta[1] = atan2(B2/delta, A2/delta);
  theta[3] = q - theta[1] - theta[2];
  // Wrap qJoint[3] to (-pi,pi]
  swap_joint(theta[3]);
  return theta;
}

// 平面内的 Jacobian 矩阵
// 速度旋量对应的是单位时间内的旋量，后续记得转换成伺服周期内的速度
THETA plane_jacobian(Vec3d twist) {
  Mat3d jcb;
  Vec3d q234;
  THETA dq;
  jcb(0,0) = jcb(0,1) = jcb(0,2) = -1;
  jcb(1,2) = -82.3             ; jcb(2,2) = 94.65;              // jcb4
  jcb(1,1) = jcb(1,2)+392.25*c4; jcb(2,1) = jcb(2,2)-392.25*s4; // jcb3
  jcb(1,0) = jcb(1,1)+425*c34  ; jcb(2,0) = jcb(2,1)-425*s34;   // jcb2
  q234 = jcb.inverse()*twist;
  dq = {0,q234(0),q234(1),q234(2),0,0};
  return dq;
}


/*************************************************************************
 * @func : ur_kinematics
 * @brief: M-DH求解机械臂运动学
 * @param: MATRIX_D rx_h - 旋转矩阵;
 * @return: 末端位姿向量(6x1)
*************************************************************************/
int ur_kinematics(Mat4d& tranMat) {
  double hori, vert;

  hori = DH_D4 + DH_D6*c5;
  vert = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 + DH_D6*c234*s5;
  // 位置向量
  tranMat(0,3) = -s1*hori + c1*vert;
  tranMat(1,3) =  c1*hori + s1*vert;
  tranMat(2,3) = DH_D1 - DH_A2*s2 - DH_A3*s23 - DH_D5*c234 - DH_D6*s234*s5;
  // 旋转矩阵
  tranMat(0,0) = c6*(s1*s5 + c1*c234*c5) - s234*c1*s6;
  tranMat(0,1) = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
  tranMat(0,2) = c234*c1*s5 - c5*s1;
  tranMat(1,0) = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
  tranMat(1,1) = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
  tranMat(1,2) = c1*c5 + c234*s1*s5;
  tranMat(2,0) = -c234*s6 - s234*c5*c6;
  tranMat(2,1) = s234*c5*s6 - c234*c6;
  tranMat(2,2) = -s234*s5;
  // 齐次项
  tranMat(3,0) = tranMat(3,1) = tranMat(3,2) = 0;
  tranMat(3,3) = 1;

  return 1;
}

/*************************************************************************
 * @func : ur_jacobian
 * @brief: 计算从关节空间到笛卡尔空间的雅克比矩阵
 * @return: 雅克比矩阵
*************************************************************************/
Mat6d ur_jacobian() {
  Mat6d jcb;

  double foo11, foo12, foo2, foo3, foo4;
  foo11 = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 + DH_D6*c234*s5;
  foo12 = DH_D4 + DH_D6*c5;
  foo2 = DH_A2*s2 + DH_A3*s23 + DH_D5*c234 + DH_D6*s234*s5; 
  foo3 = DH_A3*s23 + DH_D5*c234 + DH_D6*s234*s5;
  foo4 = DH_D5*c234 + DH_D6*s234*s5;

  /* ****** 平动分量 ****** */
  // dr/dq5
  jcb(0,4) = DH_D6*(s1*s5+c1*c234*c5);
  jcb(1,4) = DH_D6*(-c1*s5+s1*c234*c5);
  jcb(2,4) = -DH_D6*s234*c5;
  // dr/dq4
  jcb(0,3) = -foo4*c1;
  jcb(1,3) = -foo4*s1;
  jcb(2,3) = DH_D5*s234 - DH_D6*c234*s5; 
  // dr/dq3
  jcb(0,2) = -foo3*c1;
  jcb(1,2) = -foo3*s1;
  jcb(2,2) = -DH_A3*c23 + jcb(2,3); 
  // dr/dq2
  jcb(0,1) = -c1*foo2;
  jcb(1,1) = -s1*foo2;
  jcb(2,1) = -DH_A2*c2 + jcb(2,2); 
  // dr/dq1
  jcb(0,0) = -s1*foo11 - c1*foo12;
  jcb(1,0) = -s1*foo12 + c1*foo11;
  /* ****** 转动分量 ****** */
  jcb(5,0) = 1;
  jcb(3,1) = jcb(3,2) = jcb(3,3) = -s1;
  jcb(4,1) = jcb(4,2) = jcb(4,3) = c1;
  jcb(3,4) = -c1*s234;
  jcb(4,4) = -s1*s234;
  jcb(5,4) = -c234;               
  jcb(3,5) = -s1*c5 + c1*c234*s5; 
  jcb(4,5) = c1*c5 + s1*c234*s5;  
  jcb(5,5) = -s234*s5;            

  return jcb;
}

Mat3d RPY2RotMat(double alpha, double beta, double gamma) {
  Mat3d rotMat;

  // X
  rotMat(0,0) = cos(alpha)*cos(beta);
  rotMat(1,0) = sin(alpha)*cos(beta);
  rotMat(2,0) = -sin(beta);
  // Y
  rotMat(0,1) = cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma);
  rotMat(1,1) = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
  rotMat(2,1) = cos(beta)*sin(gamma);
  // Z
  rotMat(0,2) = cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
  rotMat(1,2) = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
  rotMat(2,2) = cos(beta)*cos(gamma);

  return rotMat;
}

Vec3d RotMat2EulerAngle(Mat3d rotMat) {
  double alpha, beta, gamma, r11, r21, r31, r32, r33;
  r31 = rotMat(2,0);
  r11 = rotMat(0,0);
  r21 = rotMat(1,0);
  r32 = rotMat(2,1);
  r33 = rotMat(2,2);

  beta = atan2(-r31, sqrt(r11*r11+r21*r21));
  alpha = atan2(r21/cos(beta), r11/cos(beta));
  gamma = atan2(r32/cos(beta), r33/cos(beta));

  Vec3d EulerAngle(alpha, beta, gamma);
  return EulerAngle;
}


THETA ur_InverseKinematics(Mat4d tranMat, THETA curTheta) {
  std::array<double,6> qJoint;
  double c1, s1,  c3, s3,  s5, c6, s6;
  double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, temp1, temp2;
  nx = tranMat(0,0);  ox = tranMat(0,1);  ax = tranMat(0,2);  px = tranMat(0,3);
  ny = tranMat(1,0);  oy = tranMat(1,1);  ay = tranMat(1,2);  py = tranMat(1,3);
  nz = tranMat(2,0);  oz = tranMat(2,1);  az = tranMat(2,2);  pz = tranMat(2,3);

  // 关节角1 (t15_24)
  double A1 = py - ay*DH_D6, B1 = -px + ax*DH_D6;
  temp1 = atan2(DH_D4, sqrt(A1*A1+B1*B1-DH_D4*DH_D4)) - atan2(A1,B1);
  temp2 = atan2(DH_D4,-sqrt(A1*A1+B1*B1-DH_D4*DH_D4)) - atan2(A1,B1);
  swap_joint(temp1); swap_joint(temp2);
  qJoint[0] = fabs(temp1-curTheta[0])<fabs(temp2-curTheta[0]) ? temp1 : temp2;
  s1 = sin(qJoint[0]); c1 = cos(qJoint[0]);
  // 关节角5 (t15_22)
  temp1 =  acos(-ax*s1+ay*c1);
  temp2 = -acos(-ax*s1+ay*c1);
  qJoint[4] = fabs(temp1-curTheta[4])<fabs(temp2-curTheta[4]) ? temp1 : temp2;
  s5 = sin(qJoint[4]);
  // 关节角6 (t15_21)
  double A6 = ny*c1-nx*s1, B6 = -oy*c1+ox*s1;
  qJoint[5] = atan2(-s5,0) - atan2(A6,B6);
  // qJoint[5] = atan2(-B6/s5,-A6/s5);
  s6 = sin(qJoint[5]); c6 = cos(qJoint[5]);
  // 关节角3 (t14_14, t14_34)
  double A3 = px*c1 +py*s1 -DH_D5*(nx*c1*s6+ox*c1*c6+ny*s1*s6+oy*s1*c6) -DH_D6*(ay*s1+ax*c1);
  double B3 = pz -DH_D1 -az*DH_D6 -DH_D5*(oz*c6+nz*s6);
  temp1 =  acos((A3*A3+B3*B3-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  temp2 = -acos((A3*A3+B3*B3-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  qJoint[2] = fabs(temp1-curTheta[2])<fabs(temp2-curTheta[2]) ? temp1 : temp2;
  s3 = sin(qJoint[2]); c3 = cos(qJoint[2]);
  // 关节角2 (t14_14, t14_34)
  double A2 = -(DH_A3*DH_A3 +2*DH_A2*DH_A3*c3 +DH_A2*DH_A2);
  double B2 = -(DH_A3*c3+DH_A2)*A3 +DH_A3*s3*B3;
  double C2 = (DH_A3*c3+DH_A2)*B3 +DH_A3*s3*A3;
  qJoint[1] = atan2(C2/A2, B2/A2);
  // 关节角4 (t15_13, t15_33)
  double A4 = -(oz*c6 + nz*s6);
  double B4 = -c6*(ox*c1+oy*s1) -s6*(nx*c1+ny*s1);
  qJoint[3] = atan2(B4, A4) - qJoint[1] - qJoint[2];
  // Wrap qJoint[3] to (-pi,pi]
  swap_joint(qJoint[3]);
  return qJoint;
}

// Wrap joint angle to (-pi,pi]
bool swap_joint(double& joint) {
  while(true) {
    if (joint > M_PI) {
      joint -= 2*M_PI;
    } else if (joint <= -M_PI) {
      joint += 2*M_PI;
    } else {
      return 1;
    }
  }
  return 0;
}


/*************************************************************************
 * ======================================================================
 * ===             轨迹插补 | Trajectory Interpolation                ===
 * ======================================================================
*************************************************************************/
/*************************************************************************
 * @func  : velocity_interpolation
 * @brief : 利用 Jacobian 矩阵进行速度伺服控制 
 * @param : offsetTime_运动时间; freq_插补频率; curTheta_当前关节角; 
 *          refTheta_下一时刻关节角; velocity_速度命令
 * @return: 修改_下一时刻的关节角，返回_轨迹完成标志位
*************************************************************************/
bool velocity_interpolation(double offsetTime, double freq, THETA curTheta, 
    THETA& refTheta, Arr6d velocity) {
  Vec6d velo, dq;
  double time = freq * (offsetTime+SERVO_TIME);

  for (int i=0; i<6; ++i) velo[i] = velocity[i];
  // 用参考角度计算各关节角的正余弦值
  calcJnt(refTheta);
  // 计算 Jacobian 矩阵
  Mat6d ijcb = ur_jacobian().inverse();
  dq = ijcb*velo;
  // 当前运动时间占总时间的比例大于1，即超过预定时间，插值点不变，返回已完成
  if (time >= 1) {
    // 路径停止前，给一定时间让机械臂停止
    for (int i=0; i<6; ++i) refTheta[i] = refTheta[i];
    if (time >= 1.1) {
      return true;
    }
  }
  for (int i=0; i<6; ++i) refTheta[i] += dq(i);
  return false;
} // bool velocity_interpolation()


/*************************************************************************
 * @func  : joint_interpolation
 * @brief : 计算关节空间的位置插补
 * @param : offsetTime_运动时间; freq_插补频率; interpMode_插补模式; 
 *          orig_插值起点; goal_插值终点; &refVal_当前时刻插值点位置
 * @return: 修改_下一时刻的关节角，返回_轨迹完成标志位
*************************************************************************/
bool joint_interpolation(double offsetTime, double freq, int interpMode, 
    Arr6d orig, Arr6d goal, Arr6d& refVal) {
  // 当前运动时间占总时间的比例 | 已完成的路径占全路径的比例
  double time = freq * (offsetTime+SERVO_TIME);

  // 超过预定时间，插值点设为终点值，返回已完成
  if (time >= 1) {
    refVal = goal;
    return true;
  }
  // 按插补模式在关节空间进行轨迹插补
  switch (interpMode) {
  // sin 関数による軌道補間 | sin函数的轨迹插补
  case INTERP_SIN:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] + (goal[i]-orig[i])*time -
        (goal[i]-orig[i])*sin(2.0*M_PI*time)/(2.0*M_PI);
    }
    break;
  // 1 次関数による軌道補間 | 一次函数的轨迹插补
  case INTERP_1JI:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] + (goal[i] - orig[i]) * time;
    }
    break;
  // 3 次関数による軌道補間 | 3次函数的轨迹插补
  case INTERP_3JI:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] + (goal[i] - orig[i]) * time*time*(3.0-2*time);
    }
    break;
  // 5 次関数による軌道補間 | 5次函数的轨迹插补
  case INTERP_5JI:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] +
        (goal[i] - orig[i]) * time*time*time*(10.0+time*(-15.0+6.0*time));
    }
    break;
  // ステップ関数による軌道補間 | 阶跃函数的轨迹插补
  case INTERP_STEP: for (int i=0; i<6; ++i) {refVal[i] = goal[i];} break;
  default: printf("\n==>> Error! Unknow interpolate mode!\n"); break;
  } // switch(interpMode)
  return false;
} // bool joint_interpolation()


