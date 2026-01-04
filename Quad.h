#ifndef __QUAD_H
#define __QUAD_H

#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include <eigen3/Eigen/Eigen>
#include <qpOASES.hpp>
#include "platform_ui_adapter.h"
#include "simulate.h"
#include "glfw_adapter.h"
#include <thread>
using namespace Eigen;
using namespace std;
#include "Self_mujoco_lib.h"
using RMT = Matrix3f;
#define h 10
namespace Quad
{

  inline constexpr float hx = 0.1934f; // 使用 constexpr 表示编译时常量
  inline constexpr float hy = 0.0465f;
  inline constexpr float l1 = 0.0955f;
  inline constexpr float l2 = 0.213f;
  inline constexpr float l3 = 0.213f;

  namespace PDcontrol
  {

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
    extern Matrix3i StateTransformMatrix;
    extern FSMstate currentState;
    extern Busystate busystate;
    void Init_FSM();
    int StateTran(const FSMstate &Goalstate);
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
    extern Vector4f tFai, sFai;
    extern Vector4f tst, tsw;
    void UpdateGait();
    void Gait_Init();
    extern Vector3f FrPstend, FlPstend, RrPstend, RlPstend;
    Matrix3f TF_Z(float dFaiz);

    extern Vector3f SymPb1, SymPb2, SymPb3, SymPb4; // 对称足底位置的XY坐标向量
    extern Vector4f FaiZtouch;                      // 触地时间质心位置 和 偏航角
    // 足底触地时间质心位置   足底触地时世界坐标系下的对称点坐标
    extern vector<Vector3f> Pcomtouch, Psymtouch, P1, P2, P3, P4, Pswend;
    extern vector<Vector3f> SwingTrajectory; // 轨迹就是 位置 速度
  };

  namespace KF
  {
    extern RMT B2I, I2I0, I02W, B2W;
    extern vector<float> Quat;
    extern float Faiz, Faix, Faiy;
    void B2WUpdate(const mjModel *model, const mjData *data, const std::string &sensor_name = "imu_quat");
    extern VectorXf jointpos, jointvel, jointForce;
    void joint_sensor_data_update(const mjModel *model, const mjData *data);
    extern Vector3f Flipb, Fripb, Rlipb, Rripb;     // 足底位置 在本体系中
    extern Vector3f FliPbv, FriPbv, RliPbv, RriPbv; //  足底速度 在本体系中
    extern Vector3f qvfr, qvfl, qvrr, qvrl;         // 关节速度向量
    extern Matrix3f jocofr, jocofl, jocorl, jocorr; // 腿部的雅可比矩阵
    void FootUpdate();
    void kf_Init(float t);
    void Kfsolver();
    extern MatrixXf A, B, U, Z, H; // 状态矩阵 输入矩阵  输入向量 观测向量 观测矩阵
    extern MatrixXf _X, X, _P, P;  // 先验状态状态向量， 后验状态， 先验状态估计误差协方差矩阵， 后验状态估计误差协方差矩阵
    extern MatrixXf Q, R, K;       // 过程噪声协方差矩阵   测量噪声协方差矩阵  卡尔曼增益
    extern Vector3f Wb;            // 角速度在本体系中的表示
    extern Matrix3f WbS;           // 反对称矩阵
    extern Vector3f pcom, vcom;
  };

  namespace KeyboardIns
  {
    // 键盘发出的命令是相对 机器狗自身的
    extern float dVxb, dVyb, dWzb, dWzO;
    //  变成向量形式         在世界中的期望
    extern MatrixXf dVb, dVO, dWb, dWO;
    extern float dFaiz, dHb, dFaix, dFaiy; //  期望偏航角  期望机身高度
    extern Matrix3f TFZ;                   // 关于Z的旋转矩阵
    extern Vector3f dPO;                   // 在世界坐标系中的期望质心位置
    extern MatrixXf W;                     // 用于坡度估计的矩阵   N_ 是归一化的
    extern VectorXf Z, A, _A, N, N_;
    extern Vector3f Tao, dFai;       // dFai  是期望角度向量
    extern vector<VectorXf> desirex; // 期望状态
    void Update_ins(mujoco::Simulate *sim);
    extern MatrixXf D; // 期望轨迹序列
  };

  namespace ConvexMPC
  {

    extern MatrixXf Continue_A, Continue_B, A, B;
    void UpdateState();
    extern Matrix3f BInertia, PInertia; // 本体系的惯性矩阵 和定向本体系的惯性矩阵
    void MPC_init();
    Matrix3f QUa2Mat(float w, float x, float y, float z);
    Matrix3f getRotationMatrix(double psi, double theta, double phi);
    extern MatrixXf Q, R, Aqp, Bqp, D;
  };

};

#endif