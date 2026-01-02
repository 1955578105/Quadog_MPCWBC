#include "Quad.h"
#define h 10
namespace Quad
{

  namespace PDcontrol
  {
    void PDcontrol(const mjModel *model, const mjData *data, vector<float> torques)
    {
      int i = 0;
      for (auto torque : torques)
        data->ctrl[i++] = torque;
    }
  };
  //   有限状态机
  //   1.状态：  下蹲 站立 行走（包括trot walk等）
  //   2.用矩阵表示 状态转换关系
  // 当前\目标 squat stand move
  //   squat    0     1     0
  //   stand    1     0      1
  //   move     0     1      0

  //   3.  转换状态要满足两个条件  ：（1） 满足状态转换关系   （2）当前状态不处于忙碌状态

  //   4. 忙碌状态判断  ： 下蹲 和站立可以用角度判断  行走要等脚都触碰地
  namespace FSM
  {
    Matrix3i StateTransformMatrix;
    FSMstate currentState;
    Busystate busystate;
    void Init_FSM()
    {
      StateTransformMatrix << 0, 1, 0, 1, 0, 1, 0, 1, 0;
      currentState = squat;
      busystate = BUSY;
    }

    // 按照不同的状态  判断是否 处于忙碌状态
    Busystate checkBusy()
    {
      if (currentState == squat)
      {
        // 检查关节角度
        return BUSY;
      }
      else if (currentState == stand)
      {
        // 检查关节角度
        return BUSY;
      }
      else if (currentState == move)
      {
        // 检查关节角度
        return BUSY;
      }
      return BUSY;
    }

    // 状态转换  成功返回1 失败返回0
    int StateTran(const FSMstate &Goalstate)
    {
      if (StateTransformMatrix(currentState, Goalstate) == 0)
        return 0;
      if (checkBusy() == BUSY)
        return 0;
      currentState = Goalstate;
      return 1;
    }

  };

  // 步态调度器   进行步态调度  落足点规划 和 足底轨迹规划

  // 目前加入两种步态
  // trot 步态和walk 步态
  //         fr  fl    rr rl
  // trot步态 Gd= [0.5 0.5 0.5 0.5]   Go = [0.5 0 0 0.5]
  // walk 步态 Gd= [0.75 0.75 0.75 0.75]  Go=[0.25 0.75 0.5 0]
  // */

  namespace Gait
  {
    Matrix<float, 2, 4> Gd;
    Matrix<float, 2, 4> Go;
    Vector4f tFai, sFai;
    Vector4f tst, tsw;
    Vector3f FrPstend, FlPstend, RrPstend, RlPstend;
    float stperiod, swperiod; // 支撑和摆动周期  不同的步态不同
    void Gait_Init()
    {
      Gd << 0.5, 0.5, 0.5, 0.5,
          0.75, 0.75, 0.75, 0.75;
      Go << 0.5, 0, 0, 0.5,
          0.25, 0.75, 0.5, 0;
      FrPstend = KF::X.block(6, 0, 3, 1);
      FlPstend = KF::X.block(9, 0, 3, 1);
      RrPstend = KF::X.block(12, 0, 3, 1);
      RlPstend = KF::X.block(15, 0, 3, 1);
    }
    char currentGait = 0; // 默认选择一个步态
    // 进入当前步态的持续时间
    float Time = 0;
    // 步态周期
    float GaitPeriod = 0.5;

    // 当前在第几个周期
    unsigned int n;
    // 在当前周期的 时间
    float tng;
    //  归一化的 当前步态进度
    float _tng;

    // 开一个线程 实现步态计时  也是在Move状态开始
    void TimeUpdate()
    {
      // 判断状态后 更新
      Time++;
    }

    /// @brief update Gait 在进入move的状态时 才运行

    void UpdateGait()
    {
      n = Time / GaitPeriod + 1;
      tng = Time - (n - 1) * GaitPeriod;
      _tng = tng / GaitPeriod; // 当前进度

      for (int i = 0; i < 4; i++)
      {
        // (2.34)
        if (_tng >= Go(currentGait, i))
        {
          tFai[i] = _tng - Go(currentGait, i);
        }
        else
        {
          tFai[i] = _tng - Go(currentGait, i) + 1;
        }
        //(2.35) 更新支撑腿的判断
        if (tFai[i] <= Gd(currentGait, i))
        {
          sFai[i] = 1; // 当前腿为支撑腿
        }
        else
        {
          sFai[i] = 0; // 为摆动腿
        }
        // (2.36) 支撑相位进度
        if (sFai[i] == 1)
        {
          tst[i] = tFai[i] / Gd(currentGait, i);
        }
        else
        {
          tst[i] = 0;
        }
        // (2.37) 摆动相位进度
        if (sFai[i] == 0)
        {
          tsw[i] = (tFai[i] - Gd(currentGait, i)) / (1 - Gd(currentGait, i));
        }
        // else
        // {
        //   tsw[i] = 0;
        // }
      }
    }
    // 每个腿的最后一次支撑足坐标更新 用与 坡度估计
    void Pstend_Update()
    {
      if (sFai[0] == 1)
      {
        FrPstend = KF::X.block(6, 0, 3, 1);
      }
      if (sFai[1] == 1)
      {
        FlPstend = KF::X.block(9, 0, 3, 1);
      }
      if (sFai[2] == 1)
      {
        RrPstend = KF::X.block(12, 0, 3, 1);
      }
      if (sFai[3] == 1)
      {
        RlPstend = KF::X.block(15, 0, 3, 1);
      }
    }

    Matrix3f TF_Z(float dFaiz)
    {
      Matrix3f TFZ;
      TFZ << cos(dFaiz), -sin(dFaiz), 0,
          sin(dFaiz), cos(dFaiz), 0,
          0, 0, 1;
      return TFZ;
    }

    Vector3f SymPb1, SymPb2, SymPb3, SymPb4;
    vector<Vector3f> SymPb;
    Vector4f FaiZtouch;
    vector<Vector3f> Pcomtouch, Psymtouch, Pswend, P1, P2, P3, P4;
    float kp = 0.15;
    ; // 对称足底位置的XY坐标向量
    /// @brief 落足点规划 和 足底轨迹规划
    void FootTraj_Planning()
    {
      // -0.02 是为了让足底位置稍微靠后一些  增加稳定性
      SymPb1 << hx - 0.02, -hy - l1, 0;
      SymPb2 << hx - 0.02, hy + l1, 0;
      SymPb3 << -hx - 0.02, -hy - l1, 0;
      SymPb4 << -hx - 0.02, hy + l1, 0;
      SymPb.push_back(SymPb1);
      SymPb.push_back(SymPb2);
      SymPb.push_back(SymPb3);
      SymPb.push_back(SymPb4);

      for (int i = 0; i < 3; i++)
      {
        // 无论支撑腿还是摆动腿 都要更新  质心触地位置 和 偏航角
        Pcomtouch[i] = KF::pcom + KF::B2W * KeyboardIns::dVb * ((1 - tsw[i]) * swperiod);
        FaiZtouch[i] = KF::Faiz + KeyboardIns::dWzb * ((1 - tsw[i]) * swperiod);

        if (sFai[i] == 1) // 支撑腿一直更新目标落足点 直到支撑的最后一刻落足点确定，不再更新
        {

          // 世界坐标系下的每个足的对称点坐标
          /// @note  足底轨迹规划
          Psymtouch[i] = Pcomtouch[i] + TF_Z(FaiZtouch[i]) * SymPb[i];
          P1[i] = (KeyboardIns::dVO * stperiod) / 2;
          P2[i] = TF_Z(FaiZtouch[i]) * ((TF_Z(KeyboardIns::dWzO * stperiod / 2) * SymPb[i]) - SymPb[i]);
          P3[i] = kp * (KF::vcom - KeyboardIns::dVO);
          P4[i] = (KF::pcom[2] / 9.81) * (KF::vcom.cross(KF::B2W * KF::Wb));
          // 最终的落足点坐标
          Pswend[i] = Psymtouch[i] + P1[i] + P2[i] + P3[i] + P4[i];
          Pswend[i][2] = KeyboardIns::A[0] + KeyboardIns::A[1] * (Pswend[i][0]) + KeyboardIns::A[2] * (Pswend[i][1]);
        }
        else // 摆动腿 根据 终点和起点进行轨迹规划
        {
        }
      }
    }
  };

  // 基于卡尔曼滤波的 状态估计器

  // 1. 先获得 基于imu的 旋转矩阵
  // B2I, I2I0, I02W, B2W  4个矩阵
  // B2W =  I02W *I2I0 * B2I;  发现  I02W  和 B2I 都是单位矩阵
  // 则  B2W  =  I2I0    I2I0  就是 IMU直接读取的四元数转换的旋转矩阵

  namespace KF
  {
    RMT B2I, I2I0, I02W, B2W;
    VectorXf jointpos(12), jointvel(12), jointForce(12);
    Vector3f Flipb, Fripb, Rlipb, Rripb;
    Vector3f FliPbv, FriPbv, RliPbv, RriPbv;
    Vector3f qvfr, qvfl, qvrr, qvrl;
    Matrix3f jocofr, jocofl, jocorl, jocorr;
    vector<float> Quat;
    float Faiz, Faix, Faiy;
    Vector3f Wb;  // 角速度在本体系中的表示
    Matrix3f WbS; // 反对称矩阵
    Matrix3f skewSymmetric(const Vector3f &v)
    {
      Matrix3f m;
      m << 0, -v.z(), v.y(),
          v.z(), 0, -v.x(),
          -v.y(), v.x(), 0;
      return m;
    }
    // 更新 B2W 旋转矩阵 读取角速度
    void B2WUpdate(const mjModel *model, const mjData *data, const std::string &sensor_name)
    {
      Quat = mujo::get_sensor_data(model, data, sensor_name); // 获得角度
      Quaternionf q(Quat[0], Quat[1], Quat[2], Quat[3]);
      Faiz = q.toRotationMatrix().eulerAngles(2, 1, 0)[0]; // 获得当前偏航角
      vector<float> temp = mujo::get_sensor_data(model, data, "imu_gyro");
      Wb << temp[0], temp[1], temp[2]; // 获得角速度数据 由于Imu坐标系和本体系方向相同  所以不用变换
      WbS = skewSymmetric(Wb);         // 获得反对称矩阵
      float w = Quat[0];
      float x = Quat[1];
      float y = Quat[2];
      float z = Quat[3];
      B2W << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y,
          2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x,
          2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y;
    }

    // 获取 关节传感器数据
    void joint_sensor_data_update(const mjModel *model, const mjData *data)
    {
      jointpos[0] = mujo::get_sensor_data(model, data, "FR_hip_pos")[0];
      jointpos[1] = mujo::get_sensor_data(model, data, "FR_thigh_pos")[0];
      jointpos[2] = mujo::get_sensor_data(model, data, "FR_calf_pos")[0];

      jointpos[3] = mujo::get_sensor_data(model, data, "FL_hip_pos")[0];
      jointpos[4] = mujo::get_sensor_data(model, data, "FL_thigh_pos")[0];
      jointpos[5] = mujo::get_sensor_data(model, data, "FL_calf_pos")[0];

      jointpos[6] = mujo::get_sensor_data(model, data, "RR_hip_pos")[0];
      jointpos[7] = mujo::get_sensor_data(model, data, "RR_thigh_pos")[0];
      jointpos[8] = mujo::get_sensor_data(model, data, "RR_calf_pos")[0];

      jointpos[9] = mujo::get_sensor_data(model, data, "RL_hip_pos")[0];
      jointpos[10] = mujo::get_sensor_data(model, data, "RL_thigh_pos")[0];
      jointpos[11] = mujo::get_sensor_data(model, data, "RL_calf_pos")[0];

      jointvel[0] = mujo::get_sensor_data(model, data, "FR_hip_vel")[0];
      jointvel[1] = mujo::get_sensor_data(model, data, "FR_thigh_vel")[0];
      jointvel[2] = mujo::get_sensor_data(model, data, "FR_calf_vel")[0];
      qvfr << jointvel[0], jointvel[1], jointvel[2];
      jointvel[3] = mujo::get_sensor_data(model, data, "FL_hip_vel")[0];
      jointvel[4] = mujo::get_sensor_data(model, data, "FL_thigh_vel")[0];
      jointvel[5] = mujo::get_sensor_data(model, data, "FL_calf_vel")[0];
      qvfl << jointvel[3], jointvel[4], jointvel[5];
      jointvel[6] = mujo::get_sensor_data(model, data, "RR_hip_vel")[0];
      jointvel[7] = mujo::get_sensor_data(model, data, "RR_thigh_vel")[0];
      jointvel[8] = mujo::get_sensor_data(model, data, "RR_calf_vel")[0];
      qvrr << jointvel[6], jointvel[7], jointvel[8];
      jointvel[9] = mujo::get_sensor_data(model, data, "RL_hip_vel")[0];
      jointvel[10] = mujo::get_sensor_data(model, data, "RL_thigh_vel")[0];
      jointvel[11] = mujo::get_sensor_data(model, data, "RL_calf_vel")[0];
      qvrl << jointvel[9], jointvel[10], jointvel[11];
      jointForce[0] = mujo::get_sensor_data(model, data, "FR_hip_torque")[0];
      jointForce[1] = mujo::get_sensor_data(model, data, "FR_thigh_torque")[0];
      jointForce[2] = mujo::get_sensor_data(model, data, "FR_calf_torque")[0];
      jointForce[3] = mujo::get_sensor_data(model, data, "FL_hip_torque")[0];
      jointForce[4] = mujo::get_sensor_data(model, data, "FL_thigh_torque")[0];
      jointForce[5] = mujo::get_sensor_data(model, data, "FL_calf_torque")[0];
      jointForce[6] = mujo::get_sensor_data(model, data, "RR_hip_torque")[0];
      jointForce[7] = mujo::get_sensor_data(model, data, "RR_thigh_torque")[0];
      jointForce[8] = mujo::get_sensor_data(model, data, "RR_calf_torque")[0];
      jointForce[9] = mujo::get_sensor_data(model, data, "RL_hip_torque")[0];
      jointForce[10] = mujo::get_sensor_data(model, data, "RL_thigh_torque")[0];
      jointForce[11] = mujo::get_sensor_data(model, data, "RL_calf_torque")[0];
    }

    // 更新足端位置 和速度  雅可比
    void FootUpdate()
    {
      Flipb = {-l2 * sin(jointpos[4]) - l3 * sin(jointpos[4] + jointpos[5]) + hx,
               l1 * cos(jointpos[3]) + l3 * sin(jointpos[3]) * cos(jointpos[4] + jointpos[5]) + l2 * cos(jointpos[4]) * sin(jointpos[3]) + hy,
               l1 * sin(jointpos[3]) - l3 * cos(jointpos[3]) * cos(jointpos[4] + jointpos[5]) - l2 * cos(jointpos[3]) * cos(jointpos[4])};

      Fripb = {-l2 * sin(jointpos[1]) - l3 * sin(jointpos[1] + jointpos[2]) + hx,
               -l1 * cos(jointpos[0]) + l3 * sin(jointpos[0]) * cos(jointpos[1] + jointpos[2]) + l2 * cos(jointpos[1]) * sin(jointpos[0]) - hy,
               -l1 * sin(jointpos[0]) - l3 * cos(jointpos[0]) * cos(jointpos[1] + jointpos[2]) - l2 * cos(jointpos[0]) * cos(jointpos[1])};

      Rlipb = {-l2 * sin(jointpos[10]) - l3 * sin(jointpos[10] + jointpos[11]) - hx,
               l1 * cos(jointpos[9]) + l3 * sin(jointpos[6]) * cos(jointpos[10] + jointpos[11]) + l2 * cos(jointpos[10]) * sin(jointpos[9]) + hy,
               l1 * sin(jointpos[9]) - l3 * cos(jointpos[6]) * cos(jointpos[10] + jointpos[11]) - l2 * cos(jointpos[9]) * cos(jointpos[10])};

      Rripb = {-l2 * sin(jointpos[7]) - l3 * sin(jointpos[7] + jointpos[8]) - hx,
               -l1 * cos(jointpos[6]) + l3 * sin(jointpos[6]) * cos(jointpos[7] + jointpos[8]) + l2 * cos(jointpos[7]) * sin(jointpos[6]) - hy,
               -l1 * sin(jointpos[6]) - l3 * cos(jointpos[6]) * cos(jointpos[7] + jointpos[8]) - l2 * cos(jointpos[6]) * cos(jointpos[7])};

      jocofr << 0, -l2 * cos(jointpos[1]) - l3 * cos(jointpos[1] + jointpos[2]), -l3 * cos(jointpos[1] + jointpos[2]),
          l1 * sin(jointpos[0]) + l3 * cos(jointpos[0]) * cos(jointpos[1] + jointpos[2]) + l2 * cos(jointpos[1]) * cos(jointpos[0]), -l3 * sin(jointpos[0]) * sin(jointpos[1] + jointpos[2]) - l2 * sin(jointpos[1]) * sin(jointpos[0]), -l3 * sin(jointpos[0]) * sin(jointpos[1] + jointpos[2]),
          -l1 * cos(jointpos[0]) + l3 * sin(jointpos[0]) * cos(jointpos[1] + jointpos[2]) + l2 * sin(jointpos[0]) * cos(jointpos[1]), l3 * cos(jointpos[0]) * sin(jointpos[1] + jointpos[2]) + l2 * cos(jointpos[0]) * sin(jointpos[1]), l3 * cos(jointpos[0]) * sin(jointpos[1] + jointpos[2]);
      jocofl << 0, -l2 * cos(jointpos[4]) - l3 * cos(jointpos[4] + jointpos[5]), -l3 * cos(jointpos[4] + jointpos[5]),
          -l1 * sin(jointpos[3]) + l3 * cos(jointpos[3]) * cos(jointpos[4] + jointpos[5]) + l2 * cos(jointpos[4]) * cos(jointpos[3]), -l3 * sin(jointpos[3]) * sin(jointpos[4] + jointpos[5]) - l2 * sin(jointpos[4]) * sin(jointpos[3]), -l3 * sin(jointpos[3]) * sin(jointpos[4] + jointpos[5]),
          l1 * cos(jointpos[3]) + l3 * sin(jointpos[3]) * cos(jointpos[4] + jointpos[5]) + l2 * sin(jointpos[3]) * cos(jointpos[4]), l3 * cos(jointpos[3]) * sin(jointpos[4] + jointpos[5]) + l2 * cos(jointpos[3]) * sin(jointpos[4]), l3 * cos(jointpos[3]) * sin(jointpos[4] + jointpos[5]);
      jocorr << 0, -l2 * cos(jointpos[7]) - l3 * cos(jointpos[7] + jointpos[8]), -l3 * cos(jointpos[7] + jointpos[8]),
          l1 * sin(jointpos[6]) + l3 * cos(jointpos[6]) * cos(jointpos[7] + jointpos[8]) + l2 * cos(jointpos[7]) * cos(jointpos[6]), -l3 * sin(jointpos[6]) * sin(jointpos[7] + jointpos[8]) - l2 * sin(jointpos[7]) * sin(jointpos[6]), -l3 * sin(jointpos[6]) * sin(jointpos[7] + jointpos[8]),
          -l1 * cos(jointpos[6]) + l3 * sin(jointpos[6]) * cos(jointpos[7] + jointpos[8]) + l2 * sin(jointpos[6]) * cos(jointpos[7]), l3 * cos(jointpos[6]) * sin(jointpos[7] + jointpos[8]) + l2 * cos(jointpos[6]) * sin(jointpos[7]), l3 * cos(jointpos[6]) * sin(jointpos[7] + jointpos[8]);
      jocorl << 0, -l2 * cos(jointpos[10]) - l3 * cos(jointpos[10] + jointpos[11]), -l3 * cos(jointpos[10] + jointpos[11]),
          -l1 * sin(jointpos[9]) + l3 * cos(jointpos[9]) * cos(jointpos[10] + jointpos[11]) + l2 * cos(jointpos[10]) * cos(jointpos[9]), -l3 * sin(jointpos[9]) * sin(jointpos[10] + jointpos[11]) - l2 * sin(jointpos[10]) * sin(jointpos[9]), -l3 * sin(jointpos[9]) * sin(jointpos[10] + jointpos[11]),
          l1 * cos(jointpos[9]) + l3 * sin(jointpos[9]) * cos(jointpos[10] + jointpos[11]) + l2 * sin(jointpos[9]) * cos(jointpos[10]), l3 * cos(jointpos[9]) * sin(jointpos[10] + jointpos[11]) + l2 * cos(jointpos[9]) * sin(jointpos[10]), l3 * cos(jointpos[9]) * sin(jointpos[10] + jointpos[11]);
      FliPbv = jocofl * qvfl;
      FriPbv = jocofr * qvfr;
      RliPbv = jocorl * qvrl;
      RriPbv = jocorr * qvrr;
    }
    MatrixXf A(18, 18);
    MatrixXf B(18, 3);
    MatrixXf U(3, 1);
    MatrixXf H(28, 18); // 把状态向量  映射到 观测向量
    MatrixXf Q(18, 18);
    MatrixXf R(28, 28);
    MatrixXf _X(18, 1);
    MatrixXf X(18, 1);
    MatrixXf _P(18, 18);
    MatrixXf P(18, 18);
    MatrixXf Z(28, 1);
    MatrixXf K(18, 28);

    Matrix3f iden3 = Matrix3f::Identity();
    Matrix3f _3x3 = Matrix3f::Zero();
    MatrixXf _3x12(3, 12);
    MatrixXf _12x3(12, 3);

    MatrixXf iden12(12, 12);
    MatrixXf iden18(18, 18);
    MatrixXf _12x12(12, 12);
    Matrix<float, 1, 1> Onemat;
    Vector3f pcom, vcom;
    void kf_Init(float t) // t 是控制步长
    {

      iden12 = MatrixXf ::Identity(12, 12);
      // 初始化常量
      A << iden3, t * iden3, _3x12,
          _3x3, iden3, _3x12,
          _12x3, _12x3, iden12;
      B << _3x3, t * iden3, _12x3;
      U << 0, 0, -9.81;

      // 观测矩阵
      H << 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
          1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
          1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
      X << 0, 0, 0.44, 0, 0, 0, 0.1934, -0.142, 0, 0.1934, 0.142, 0, -0.1934, -0.142, 0, -0.1934, 0.142, 0; // 初始状态 应接近 真实状态
      P = MatrixXf::Ones(18, 18);
      iden18 = MatrixXf ::Identity(18, 18);
      Onemat << 1;
      Q.block(0, 0, 6, 6) = MatrixXf::Identity(6, 6);
    }
    // 求解 卡尔曼
    void Kfsolver()
    {
      // Q R Z 手动更新
      for (int i = 0; i < 4; i++)
      {
        if (Gait::sFai[i] == 0) // 是否为摆动腿 将摆动腿的状态噪声和测量噪声变大 降低他的贡献
        {
          Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = iden3;
          R.block(0 + 3 * i, 0 + 3 * i, 3, 3) = iden3;
          R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = iden3;
          R.block(24 + i, 24 + i, 1, 1) = Onemat;
        }
        else
        {
          Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = iden3;
          R.block(0 + 3 * i, 0 + 3 * i, 3, 3) = iden3;
          R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = iden3;
          R.block(24 + i, 24 + i, 1, 1) = Onemat;
        }
      }
      Z << -B2W * Fripb, -B2W * Flipb, -B2W * Rripb, -B2W * Rlipb,
          -B2W * (WbS * Fripb + FriPbv), -B2W * (WbS * Flipb + FliPbv), -B2W * (WbS * Rripb + RriPbv), -B2W * (WbS * Rlipb + RliPbv),
          0, 0, 0, 0;

      _X = A * X + B * U;
      _P = A * P * (A.transpose()) + Q;
      K = (_P * (H.transpose())) * ((H * _P * H.transpose() + R).inverse());
      X = _X + K * (Z - H * _X);
      P = (iden18 - K * H) * _P;
      pcom = X.block(0, 0, 3, 1);
      vcom = X.block(3, 0, 3, 1);
    }

  };

  // 用键盘命令代替手柄   先进行简单模拟 后边完善
  // 然后完成坡度估计
  namespace KeyboardIns
  {

    /// @brief 期望速度和角速度
    float dVxb, dVyb, dWzb, dWzO;
    float dFaiz, dHb, dFaix, dFaiy;
    MatrixXf dVb(3, 1);
    MatrixXf dVO(3, 1);
    MatrixXf dWb(3, 1);
    MatrixXf dWO(3, 1);
    Matrix3f TFZ;
    Vector3f dPO;
    MatrixXf W(4, 3);
    VectorXf Z(4), A(3), _A(3), N(3), N_(3);
    Vector3f Tao, dFai;
    vector<VectorXf> desirex;
    void Update_ins(mujoco::Simulate *sim)
    {
      static auto KeyPressedTime = std::chrono::steady_clock::now();
      while (!sim->exitrequest.load())
      {

        auto enterTime = std::chrono::steady_clock::now();

        // 检测按键状态
        if (sim->platform_ui->Is1KeyPressed())
        {

          if (std::chrono::duration_cast<chrono::milliseconds>(enterTime - KeyPressedTime) >= chrono::milliseconds(100))
          {
            std::cout << "1 key is pressed" << std::endl;
            dVyb += 1;
            KeyPressedTime = std::chrono::steady_clock::now();
          }
        }
        if (sim->platform_ui->Is2KeyPressed())
        {
          if (std::chrono::duration_cast<chrono::milliseconds>(enterTime - KeyPressedTime) >= chrono::milliseconds(100))
          {
            std::cout << "2 key is pressed" << std::endl;
            dVxb -= 1;

            KeyPressedTime = std::chrono::steady_clock::now();
          }
        }
        if (sim->platform_ui->Is3KeyPressed())
        {
          if (std::chrono::duration_cast<chrono::milliseconds>(enterTime - KeyPressedTime) >= chrono::milliseconds(100))
          {
            std::cout << "3 key is pressed" << std::endl;
            dVyb -= 1;
            KeyPressedTime = std::chrono::steady_clock::now();
          }
        }
        if (sim->platform_ui->Is4KeyPressed())
        {
          if (std::chrono::duration_cast<chrono::milliseconds>(enterTime - KeyPressedTime) >= chrono::milliseconds(100))
          {
            std::cout << "4  key is pressed" << std::endl;
            dWzb += 1;

            KeyPressedTime = std::chrono::steady_clock::now();
          }
        }
        if (sim->platform_ui->Is5KeyPressed())
        {
          if (std::chrono::duration_cast<chrono::milliseconds>(enterTime - KeyPressedTime) >= chrono::milliseconds(100))
          {
            dVxb += 1;
            std::cout << "5 key is pressed" << std::endl;

            KeyPressedTime = std::chrono::steady_clock::now();
          }
        }
        if (sim->platform_ui->Is6KeyPressed())
        {
          if (std::chrono::duration_cast<chrono::milliseconds>(enterTime - KeyPressedTime) >= chrono::milliseconds(100))
          {
            std::cout << "6 key is pressed" << std::endl;
            dWzb -= 1;

            KeyPressedTime = std::chrono::steady_clock::now();
          }
        }
        enterTime += std::chrono::milliseconds(2); // 1khz

        std::this_thread::sleep_until(enterTime);
      }
      // 根据键盘更新指令
    }

    void Keyboard_init()
    {

      dFaiz = KF::Faiz; // 用当前偏航角初始 目标偏航角
      dWzO = dWzb;
    }

    void Desire_ins_update(float MPCtime) // 这个仅能根据当前状态 估计下一次状态  ， 但是需要估计未来多个时间段的状态
    {
      for (int i = 0; i < h; ++i)
      {
        // 更新期望偏航角
        if (i == 0) // 每次第一次 将一些参数设置为当前值
        {
          dFaiz = KF::Faiz;
          TFZ << cos(dFaiz), -sin(dFaiz), 0,
              sin(dFaiz), cos(dFaiz), 0,
              0, 0, 1;
          // 更新期望角速度
          dWzO = dWzb;
          // 期望角速度向量
          dWO << 0, 0, dWzO;
          dVb << dVxb, dVyb, 0;
          dVO = TFZ * dVb;

          // 坡度估计
          Z << Gait::FrPstend[2], Gait::FlPstend[2], Gait::RrPstend[2], Gait::RlPstend[2];
          W << 1, Gait::FrPstend[0], Gait::FrPstend[1],
              1, Gait::FlPstend[0], Gait::FlPstend[1],
              1, Gait::RrPstend[0], Gait::RrPstend[1],
              1, Gait::RlPstend[0], Gait::RlPstend[1];
          _A = A;
          A = (W.transpose() * ((W * W.transpose()).inverse())) * Z;
          A = 0.2 * A + 0.8 * _A; // 低通滤波
          N << -A[1], -A[2], 1;
          // 归一化的法向量
          N_ = N * (1.0f / pow((pow(N[0], 2) + pow(N[1], 2) + 1), 0.5));
          dPO = KF::X.block(0, 0, 3, 1);
          Tao = TFZ.inverse() * N_;
          dFaix = asin(Tao[1]);
          dFaiy = atan(Tao[0] / Tao[2]);
          dFai << dFaix, dFaiy, dFaiz;
          desirex[i] << dFai, dPO, dWO, dVO, -9.81; // 此为当前状态
        }
        dFaiz = dFaiz + dWzO * MPCtime;
        // Z轴旋转矩阵
        TFZ << cos(dFaiz), -sin(dFaiz), 0,
            sin(dFaiz), cos(dFaiz), 0,
            0, 0, 1;
        // 更新期望位置
        dVO = TFZ * dVb;
        dPO = dPO + dVO * MPCtime;
        dPO[2] = dHb; // 期望机身高度

        // 计算期望滚摆角
        Tao = TFZ.inverse() * N_;
        dFaix = asin(Tao[1]);
        dFaiy = atan(Tao[0] / Tao[2]);
        dFai << dFaix, dFaiy, dFaiz;
        // 更新期望状态向量
        desirex[i + 1] << dFai, dPO, dWO, dVO, -9.81;
      }
    }
  };

  namespace ConvexMPC
  {

    MatrixXf Temp_A(13, 13), A(13, 13), Continue_B(13, 13), B(13, 13);
    Matrix3f BInertia, PInertia;
    float MPC_T = 0.01;
    // float h; // mpc预测步长
    MatrixXf Q(13 * h, 13 * h), R(12 * h, 12 * h), Aqp(13 * h, 13), Bqp(13 * h, 12 * h), D(13 * h, 1), H(12 * h, 12 * h);
    VectorXf vec(13);
    MatrixXf temp(13, 13);
    int nv = 12 * h;
    int nc = 20 * h;
    int nWSR = 100; // 最大工作集切换次数（Working Set Recalculations）
    qpOASES::QProblem solver(nv, nc);
    Matrix3f QUa2Mat(float w, float x, float y, float z)
    {
      Matrix3f tran;
      tran << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y,
          2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x,
          2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y;
      return tran;
    }
    void MPC_init()
    {
      // 初始化 A的固定值
      A.setZero();
      A.setIdentity(13, 13);
      A.block(3, 9, 3, 3) = Matrix3f::Identity() * MPC_T;
      A(11, 12) = 1 * MPC_T;

      // 初始化  本体系的惯性矩阵
      Matrix3f R = QUa2Mat(-0.000543471, 0.713435, -0.00173769, 0.700719);
      Eigen::Vector3f diaginertia(0.107027, 0.0980771, 0.0244531);
      Eigen::Matrix3f I_principal = diaginertia.asDiagonal();
      BInertia = R * I_principal * (R.transpose());

      // 初始化 B 的固定值
      B.setZero();
      B.block(9, 0, 3, 3) = Matrix3f::Identity() * MPC_T / 6.921;
      B.block(9, 3, 3, 3) = Matrix3f::Identity() * MPC_T / 6.921;
      B.block(9, 6, 3, 3) = Matrix3f::Identity() * MPC_T / 6.921;
      B.block(9, 9, 3, 3) = Matrix3f::Identity() * MPC_T / 6.921;
      // for(int i=0;i<h;i++)
      vec << 25, 25, 10, 1, 1, 100, 0, 0, 0.3, 0.2, 0.2, 20, 0;
      temp = vec.asDiagonal();
      Q.setZero();
      for (int i = 0; i < h; ++i)
      {
        Q.block(i * 13, i * 13, 13, 13) = Q;
      }
      R.setZero();
      R = MatrixXf::Identity(12 * h, 12 * h) * 0.00005;
      Aqp.setZero();
      Bqp.setZero();
      //   h = 50; // 预测步长    预测时域不能超过半个步态周期
      //  调用 qpoases 求解

      qpOASES::Options option;
      option.setToMPC();
      option.printLevel = qpOASES::PL_NONE;
      solver.setOptions(option);
    }

    Matrix3f getRotationMatrix(double psi, double theta, double phi)
    {
      // 预先计算三角函数值
      double c_psi = std::cos(psi);
      double s_psi = std::sin(psi);
      double c_theta = std::cos(theta);
      double s_theta = std::sin(theta);
      double c_phi = std::cos(phi);
      double s_phi = std::sin(phi);

      // 初始化 3x3 矩阵
      Matrix3f R;

      // 根据图片中给出的最终矩阵公式进行赋值
      // 第一行
      R(0, 0) = c_theta * c_psi;
      R(0, 1) = c_psi * s_phi * s_theta - c_phi * s_psi;
      R(0, 2) = s_phi * s_psi + c_phi * c_psi * s_theta;

      // 第二行
      R(1, 0) = c_theta * s_psi;
      R(1, 1) = c_phi * c_psi + s_phi * s_theta * s_psi;
      R(1, 2) = c_phi * s_theta * s_psi - c_psi * s_phi;

      // 第三行
      R(2, 0) = -s_theta;
      R(2, 1) = c_theta * s_phi;
      R(2, 2) = c_phi * c_theta;

      return R;
    }
    void UpdateState()
    {

      // 更新 Aqp  Bqp   //
      for (int i = 0; i < h; ++i)
      {
        if (i == 0)
        {
          Temp_A.setIdentity();
        }
        A.block(0, 6, 3, 3) = Gait::TF_Z(KeyboardIns::desirex[i][2]);
        Temp_A = A * Temp_A;
        Aqp.block(13 * i, 0, 13, 13) = (Temp_A);
      }

      for (int i = 0; i < h; i++) // B0 - B9
      {
        for (int j = i + 1; j < h + 1; j++)
        {
          Matrix3f Ro = getRotationMatrix(KeyboardIns::desirex[i][2], KeyboardIns::desirex[i][1], KeyboardIns::desirex[i][0]);
          PInertia = Ro * BInertia * (Ro.transpose());

          if (Gait::sFai[0] == 0) // 摆动腿   用目标落地点 -  质心
          {
            B.block(6, 0, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::Pswend[0] - KF::X.block(0, 0, 3, 1));
          }
          else // 支撑腿   用 支撑足  - 质心
          {
            B.block(6, 0, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::FrPstend - KF::X.block(0, 0, 3, 1));
          }

          if (Gait::sFai[1] == 0) // 摆动腿   用目标落地点 -  质心
          {
            B.block(6, 3, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::Pswend[1] - KF::X.block(0, 0, 3, 1));
          }
          else // 支撑腿   用 支撑足  - 质心
          {
            B.block(6, 3, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::FlPstend - KF::X.block(0, 0, 3, 1));
          }

          if (Gait::sFai[2] == 0) // 摆动腿   用目标落地点 -  质心
          {
            B.block(6, 6, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::Pswend[2] - KF::X.block(0, 0, 3, 1));
          }
          else // 支撑腿   用 支撑足  - 质心
          {
            B.block(6, 6, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::RrPstend - KF::X.block(0, 0, 3, 1));
          }

          if (Gait::sFai[3] == 0) // 摆动腿   用目标落地点 -  质心
          {
            B.block(6, 9, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::Pswend[3] - KF::X.block(0, 0, 3, 1));
          }
          else // 支撑腿   用 支撑足  - 质心
          {
            B.block(6, 9, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::RlPstend - KF::X.block(0, 0, 3, 1));
          }

          if (j == i + 1)
          {
            Temp_A.setIdentity();
          }
          A.block(0, 6, 3, 3) = Gait::TF_Z(KeyboardIns::desirex[j][2]); // A(i+1)开始
          Bqp.block((j - 1) * 13, i * 12, 13, 12) = Temp_A * B;
          Temp_A = A * Temp_A;
        }
      }
      H = 2 * (Bqp.transpose() * Q * Bqp + R);
      g = 2 * Bqp.transpose() * Q * (Aqp * KeyboardIns::desirex[0] - D);
      qpOASES::returnValue status = solver.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    }
  };
};
