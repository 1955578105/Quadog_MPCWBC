#ifndef __QUAD_H
#define __QUAD_H

#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include <eigen3/Eigen/Eigen>
#include <qpOASES.hpp>
#include "platform_ui_adapter.h"
#include "simulate.h"
#include "QuadProg++.hh"
#include "glfw_adapter.h"
#include <thread>
#include <memory>
#include <list>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>

using namespace std;
#include "Self_mujoco_lib.h"
using RMT = Eigen::Matrix3f;
namespace prog = quadprogpp;
#define h 20

#define MPC_T 0.01f
namespace Quad
{

  inline constexpr float hx = 0.1934f; //
  inline constexpr float hy = 0.0465f;
  inline constexpr float l1 = 0.0955f;
  inline constexpr float l2 = 0.213f;
  inline constexpr float l3 = 0.213f;

  namespace PDcontrol
  {
    void PDcontrol(const mjModel *model, mjData *data,
                   const Eigen::VectorXf &q_des,
                   const Eigen::VectorXf &qdot_des,
                   const Eigen::VectorXf &tau_ff);
  };
  namespace SystemControl
  {
    void System_Init(mjModel *model, mjData *data, float dt);
    void Control_Step(const mjModel *model, mjData *data, float dt);

    extern bool first_mpc;
  };
  namespace FSM
  {
    enum FSMstate
    {
      squat = 0,
      stand = 1,
      move = 2
    };

    enum Busystate
    {
      BUSY = 0,
      FREE = 1
    };
    extern Eigen::Matrix3i StateTransformMatrix;
    extern FSMstate currentState;
    extern Busystate busystate;
    void Init_FSM();
    int StateTran(const FSMstate &Goalstate);
    extern FSMstate lastState;
  };
  namespace Gait
  {

    extern float Time;
    extern float GaitPeriod, stperiod, swperiod; // 步态周期
    extern char currentGait;
    extern unsigned int n; //
    extern float tng;
    extern float _tng;
    //  中间变量   支撑布尔值  支撑相位进度   摆动相位进度
    extern Eigen::Vector4f tFai, sFai;
    extern Eigen::Vector4f tst, tsw;
    void UpdateGait();
    void Gait_Init();
    extern Eigen::Vector3f FrPstend, FlPstend, RrPstend, RlPstend;
    Eigen::Matrix3f TF_Z(float dFaiz);
    extern Eigen::Vector3f SymPb1, SymPb2, SymPb3, SymPb4; // 对称足底位置的XY坐标向量
    extern Eigen::Vector4f FaiZtouch;                      // 触地时间质心位置 和 偏航角
    // 足底触地时间质心位置   足底触地时世界坐标系下的对称点坐标
    extern vector<Eigen::Vector3f> Pcomtouch, Psymtouch, P1, P2, P3, P4, Pswend;
    extern vector<Eigen::Vector3f> SwingTrajectory; // 轨迹就是 位置 速度
    extern vector<Eigen::Vector3f> FootdesirePos, FootdesireVelocity, Pstend, Pstsw;
    void ChangeGait(int gait);
  };

  namespace KF
  {
    extern RMT B2I, I2I0, I02W, B2W;
    extern vector<float> Quat;
    extern float Faiz, Faix, Faiy;
    void B2WUpdate(const mjModel *model, mjData *data, const std::string &sensor_name);
    extern Eigen::VectorXf jointpos, jointvel, jointForce;
    void joint_sensor_data_update(const mjModel *model, mjData *data);
    extern Eigen::Vector3f Flipb, Fripb, Rlipb, Rripb;     // 足底位置 在本体系中
    extern Eigen::Vector3f FliPbv, FriPbv, RliPbv, RriPbv; //  足底速度 在本体系中
    extern Eigen::Vector3f qvfr, qvfl, qvrr, qvrl;         // 关节速度向量
    extern Eigen::Matrix3f jocofr, jocofl, jocorl, jocorr; // 腿部的雅可比矩阵
    void FootUpdate();
    void kf_Init(float t);
    void Kfsolver();
    extern Eigen::MatrixXf A, B, U, Z, H; // 状态矩阵 输入矩阵  输入向量 观测向量 观测矩阵
    extern Eigen::MatrixXf _X, X, _P, P;  // 先验状态状态向量， 后验状态， 先验状态估计误差协方差矩阵， 后验状态估计误差协方差矩阵
    extern Eigen::MatrixXf Q, R, K;       // 过程噪声协方差矩阵   测量噪声协方差矩阵  卡尔曼增益
    extern Eigen::Vector3f Wb;            // 角速度在本体系中的表示
    extern Eigen::Matrix3f WbS;           // 反对称矩阵
    extern Eigen::Vector3f pcom, vcom;
    extern vector<Eigen::Vector3f> iPb;
  };

  namespace KeyboardIns
  {
    // 键盘发出的命令是相对 机器狗自身的
    extern float dVxb, dVyb, dWzb, dWzO;
    //  变成向量形式         在世界中的期望
    extern Eigen::Vector3f dVb, dVO, dWb, dWO;
    extern float dFaiz, dHb, dFaix, dFaiy; //  期望偏航角  期望机身高度
    extern Eigen::Matrix3f TFZ;            // 关于Z的旋转矩阵
    extern Eigen::Vector3f dPO;            // 在世界坐标系中的期望质心位置
    extern Eigen::MatrixXf W;              // 用于坡度估计的矩阵   N_ 是归一化的
    extern Eigen::VectorXf Z, A, _A, N, N_;
    extern Eigen::Vector3f Tao, dFai;       // dFai  是期望角度向量
    extern vector<Eigen::VectorXf> desirex; // 期望状态
    void Update_ins();
    extern Eigen::MatrixXf D; // 期望轨迹序列
  };

  namespace ConvexMPC
  {

    extern Eigen::MatrixXf Continue_A, Continue_B, A, B;
    void UpdateState();
    extern Eigen::Matrix3f BInertia, PInertia; // 本体系的惯性矩阵 和定向本体系的惯性矩阵
    void MPC_init();
    Eigen::Matrix3f QUa2Mat(float w, float x, float y, float z);
    Eigen::Matrix3f getRotationMatrix(double psi, double theta, double phi);
    extern Eigen::MatrixXf Q, R, Aqp, Bqp, D;
    extern Eigen::Vector4f MPCsFai;
  };

};

class DataVisualizer
{
private:
  int sock;
  struct sockaddr_in serv_addr;

public:
  DataVisualizer(const char *ip = "127.0.0.1", int port = 9876)
  {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(ip);
  }

  void sendData(const std::string &json_str)
  {
    sendto(sock, json_str.c_str(), json_str.length(), 0,
           (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  }
};

// 全局静态实例
static inline DataVisualizer visualizer;

#endif