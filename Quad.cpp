#include "Quad.h"

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
    Eigen::Matrix3i StateTransformMatrix;
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
    Eigen::Matrix<float, 2, 4> Gd;
    Eigen::Matrix<float, 2, 4> Go;
    Eigen::Vector4f tFai, sFai;
    Eigen::Vector4f tst, tsw;
    Eigen::Vector3f FrPstend, FlPstend, RrPstend, RlPstend;
    float stperiod, swperiod; // 支撑和摆动周期  不同的步态不同
    vector<Eigen::Vector3f> FootdesirePos, FootdesireVelocity, Pstend, Pstsw;
    float dfooth = 0.06; // 期望抬腿高度
    void
    Gait_Init()
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
        Pstend[0] = FrPstend;
      }
      if (sFai[1] == 1)
      {
        FlPstend = KF::X.block(9, 0, 3, 1);
        Pstend[1] = FlPstend;
      }
      if (sFai[2] == 1)
      {
        RrPstend = KF::X.block(12, 0, 3, 1);
        Pstend[2] = RrPstend;
      }
      if (sFai[3] == 1)
      {
        RlPstend = KF::X.block(15, 0, 3, 1);
        Pstend[3] = RlPstend;
      }
    }

    Eigen::Matrix3f TF_Z(float dFaiz)
    {
      Eigen::Matrix3f TFZ;
      TFZ << cos(dFaiz), -sin(dFaiz), 0,
          sin(dFaiz), cos(dFaiz), 0,
          0, 0, 1;
      return TFZ;
    }

    Eigen::Vector3f SymPb1, SymPb2, SymPb3, SymPb4;
    vector<Eigen::Vector3f> SymPb;
    Eigen::Vector4f FaiZtouch;
    vector<Eigen::Vector3f> Pcomtouch, Psymtouch, Pswend, P1, P2, P3, P4;
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

      for (int i = 0; i < 4; i++)
      {
        // 无论支撑腿还是摆动腿 都要更新  质心触地位置 和 偏航角
        Pcomtouch[i] = KF::pcom + KF::B2W * KeyboardIns::dVb * ((1 - tsw[i]) * swperiod);
        FaiZtouch[i] = KF::Faiz + KeyboardIns::dWzb * ((1 - tsw[i]) * swperiod);
        // 世界坐标系下的每个足的对称点坐标
        /// @note  足底轨迹规划
        // 落足点坐标 无论摆动还是支撑腿 都应该一直预测  。 摆动时 要预防突发情况，及时改变落足点
        Psymtouch[i] = Pcomtouch[i] + TF_Z(FaiZtouch[i]) * SymPb[i];
        P1[i] = (KeyboardIns::dVO * stperiod) / 2;
        P2[i] = TF_Z(FaiZtouch[i]) * ((TF_Z(KeyboardIns::dWzO * stperiod / 2) * SymPb[i]) - SymPb[i]);
        P3[i] = kp * (KF::vcom - KeyboardIns::dVO);
        P4[i] = (KF::pcom[2] / 9.81) * (KF::vcom.cross(KF::B2W * KF::Wb));
        // 最终的落足点坐标
        Pswend[i] = Psymtouch[i] + P1[i] + P2[i] + P3[i] + P4[i];
        Pswend[i][2] = KeyboardIns::A[0] + KeyboardIns::A[1] * (Pswend[i][0]) + KeyboardIns::A[2] * (Pswend[i][1]);

        Pstsw[i] = Pswend[i] - Pstend[i];
        // 摆动腿 根据 终点和起点进行轨迹规划
        static float lastfoot = 0;
        if (sFai[i] == 0)
        {
          FootdesirePos[i][0] = Pstend[i][0] + Pstsw[i][0] * (3 * pow(tsw[i], 2) - 2 * pow(tsw[i], 3));
          FootdesirePos[i][1] = Pstend[i][1] + Pstsw[i][1] * (3 * pow(tsw[i], 2) - 2 * pow(tsw[i], 3));
          FootdesireVelocity[i][0] = Pstsw[i][0] * (6 * tsw[i] - 6 * pow(tsw[i], 2));
          FootdesireVelocity[i][1] = Pstsw[i][1] * (6 * tsw[i] - 6 * pow(tsw[i], 2));
          if (tsw[i] <= 0.5)
          {
            FootdesirePos[i][2] = Pstend[i][2] + dfooth * (12 * pow(tsw[i], 2) - 16 * pow(tsw[i], 3));
            lastfoot = FootdesirePos[i][2];
            FootdesireVelocity[i][2] = dfooth * (12 * tsw[i] - 24 * pow(tsw[i], 2));
          }
          else
          {
            FootdesirePos[i][2] = lastfoot + (Pswend[i][2] - lastfoot) * (12 * pow(tsw[i], 2) - 16 * pow(tsw[i], 3));
            FootdesireVelocity[i][2] = (Pswend[i][2] - dfooth) * (12 * tsw[i] - 24 * pow(tsw[i], 2));
          }

          // 在这里进行 摆动腿规划
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
    Eigen::VectorXf jointpos(12), jointvel(12), jointForce(12);
    Eigen::Vector3f Flipb, Fripb, Rlipb, Rripb;
    Eigen::Vector3f FliPbv, FriPbv, RliPbv, RriPbv;
    Eigen::Vector3f qvfr, qvfl, qvrr, qvrl;
    Eigen::Matrix3f jocofr, jocofl, jocorl, jocorr;
    vector<float> Quat;
    float Faiz, Faix, Faiy;
    Eigen::Vector3f Wb;  // 角速度在本体系中的表示
    Eigen::Matrix3f WbS; // 反对称矩阵
    vector<Eigen::Vector3f> iPb;
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f &v)
    {
      Eigen::Matrix3f m;
      m << 0, -v.z(), v.y(),
          v.z(), 0, -v.x(),
          -v.y(), v.x(), 0;
      return m;
    }
    // 更新 B2W 旋转矩阵 读取角速度
    void B2WUpdate(const mjModel *model, const mjData *data, const std::string &sensor_name)
    {
      Quat = mujo::get_sensor_data(model, data, sensor_name); // 获得角度
      Eigen::Quaternionf q(Quat[0], Quat[1], Quat[2], Quat[3]);
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
      iPb[0] = Fripb;
      iPb[1] = Flipb;
      iPb[2] = Rripb;
      iPb[3] = Rlipb;
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
    Eigen::MatrixXf A(18, 18);
    Eigen::MatrixXf B(18, 3);
    Eigen::MatrixXf U(3, 1);
    Eigen::MatrixXf H(28, 18); // 把状态向量  映射到 观测向量
    Eigen::MatrixXf Q(18, 18);
    Eigen::MatrixXf R(28, 28);
    Eigen::MatrixXf _X(18, 1);
    Eigen::MatrixXf X(18, 1);
    Eigen::MatrixXf _P(18, 18);
    Eigen::MatrixXf P(18, 18);
    Eigen::MatrixXf Z(28, 1);
    Eigen::MatrixXf K(18, 28);

    Eigen::Matrix3f iden3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f _3x3 = Eigen::Matrix3f::Zero();
    Eigen::MatrixXf _3x12(3, 12);
    Eigen::MatrixXf _12x3(12, 3);

    Eigen::MatrixXf iden12(12, 12);
    Eigen::MatrixXf iden18(18, 18);
    Eigen::MatrixXf _12x12(12, 12);
    Eigen::Matrix<float, 1, 1> Onemat;
    Eigen::Vector3f pcom, vcom;
    void kf_Init(float t) // t 是控制步长
    {

      iden12 = Eigen::MatrixXf ::Identity(12, 12);
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
      P = Eigen::MatrixXf::Ones(18, 18);
      iden18 = Eigen::MatrixXf ::Identity(18, 18);
      Onemat << 1;
      Q.block(0, 0, 6, 6) = Eigen::MatrixXf::Identity(6, 6);
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
    Eigen::MatrixXf dVb(3, 1);
    Eigen::MatrixXf dVO(3, 1);
    Eigen::MatrixXf dWb(3, 1);
    Eigen::MatrixXf dWO(3, 1);
    Eigen::Matrix3f TFZ;
    Eigen::Vector3f dPO;
    Eigen::MatrixXf W(4, 3);
    Eigen::VectorXf Z(4), A(3), _A(3), N(3), N_(3);
    Eigen::Vector3f Tao, dFai;
    vector<Eigen::VectorXf> desirex;
    Eigen::MatrixXf D(13 * h, 1);
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
        enterTime += std::chrono::milliseconds(2); // 500hz

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
          dPO = KF::pcom;
          Tao = TFZ.inverse() * N_;
          dFaix = asin(Tao[1]);
          dFaiy = atan(Tao[0] / Tao[2]);
          dFai << dFaix, dFaiy, dFaiz;
          desirex[i] << dFai, dPO, KF::B2W * KF::Wb, KF::vcom, -9.81; // 此为当前状态
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
      for (int i = 0; i < h; i++)
      {
        D.block(13 * h, 1, 13, 1) = desirex[i + 1];
      }
    }
  };

  namespace ConvexMPC
  {

    Eigen::MatrixXf Temp_A(13, 13), A(13, 13), Continue_B(13, 13), B(13, 13);
    Eigen::Matrix3f BInertia, PInertia;
    float MPC_T = 0.01;
    // float h; // mpc预测步长
    Eigen::MatrixXf Q(13 * h, 13 * h), R(12 * h, 12 * h), Aqp(13 * h, 13),
        Bqp(13 * h, 12 * h), D(13 * h, 1), H(12 * h, 12 * h), g(12 * h, 1),
        lb(12 * h, 1), ub(12 * h, 1), lba(20 * h, 1), uba(20 * h, 1), Ampc(20 * h, 12 * h);
    Eigen::VectorXf vec(13);
    Eigen::MatrixXf temp(13, 13);
    Eigen::MatrixXf Umpc(12, 1);
    int nv = 12 * h;
    int nc = 20 * h;
    int nWSR = 100; // 最大工作集切换次数（Working Set Recalculations）
    float Fmax = 30;
    qpOASES::QProblem solver(nv, nc);
    float fri; // 摩擦系数
    Eigen::Matrix3f QUa2Mat(float w, float x, float y, float z)
    {
      Eigen::Matrix3f tran;
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
      A.block(3, 9, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T;
      A(11, 12) = 1 * MPC_T;

      // 初始化  本体系的惯性矩阵
      Eigen::Matrix3f R = QUa2Mat(-0.000543471, 0.713435, -0.00173769, 0.700719);
      Eigen::Vector3f diaginertia(0.107027, 0.0980771, 0.0244531);
      float m = 6.291;
      Eigen::Matrix3f I_principal = diaginertia.asDiagonal();
      Eigen::Vector3f P(0.021112, 0, -0.005366);
      // 旋转定理 +平行移轴定理
      BInertia = (R * I_principal * (R.transpose())) + m * (P.transpose() * P * Eigen::Matrix3f::Identity() - P * P.transpose());
      // 平行移轴定理

      // 初始化 B 的固定值
      B.setZero();
      B.block(9, 0, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T / 6.921;
      B.block(9, 3, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T / 6.921;
      B.block(9, 6, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T / 6.921;
      B.block(9, 9, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T / 6.921;
      // for(int i=0;i<h;i++)
      vec << 25, 25, 10, 1, 1, 100, 0, 0, 0.3, 0.2, 0.2, 20, 0;
      temp = vec.asDiagonal();
      Q.setZero();
      for (int i = 0; i < h; ++i)
      {
        Q.block(i * 13, i * 13, 13, 13) = Q;
      }
      R.setZero();
      R = Eigen::MatrixXf::Identity(12 * h, 12 * h) * 0.00005;
      Aqp.setZero();
      Bqp.setZero();
      //   h = 50; // 预测步长    预测时域不能超过半个步态周期
      //  调用 qpoases 求解

      qpOASES::Options option;
      option.setToMPC();
      option.printLevel = qpOASES::PL_NONE;
      solver.setOptions(option);

      lb.setZero();
      lba.setZero();
      uba.setZero();
      Eigen::VectorXf vub;
      vub << 1e10, 1e10, 1e10, 1e10, Fmax;
      for (int i = 0; i < h; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          uba.block(20 * i + j * 5, 1, 5, 1) = vub;
        }
      }
      Eigen::MatrixXf t(5, 3);
      t << -1, 0, fri,
          0, -1, fri,
          1, 0, fri,
          0, 1, fri,
          0, 0, 1;
      for (int i = 0; i < h; ++i)
      {
        for (int j = 0; j < 4; ++j)
        {
          Ampc.block(20 * h + 5 * j, 12 * h + 3 * j, 5, 3) = t;
        }
      }
    }

    Eigen::Matrix3f getRotationMatrix(double psi, double theta, double phi)
    {
      // 预先计算三角函数值
      double c_psi = std::cos(psi);
      double s_psi = std::sin(psi);
      double c_theta = std::cos(theta);
      double s_theta = std::sin(theta);
      double c_phi = std::cos(phi);
      double s_phi = std::sin(phi);

      // 初始化 3x3 矩阵
      Eigen::Matrix3f R;

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
        Eigen::Matrix3f Ro = getRotationMatrix(KeyboardIns::desirex[i][2], KeyboardIns::desirex[i][1], KeyboardIns::desirex[i][0]);
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
        for (int j = i + 1; j < h + 1; j++)
        {

          if (j == i + 1)
          {
            Temp_A.setIdentity();
          }
          A.block(0, 6, 3, 3) = Gait::TF_Z(KeyboardIns::desirex[j][2]); // A(i+1)开始
          Bqp.block((j - 1) * 13, i * 12, 13, 12) = Temp_A * B;
          Temp_A = A * Temp_A;
        }
      }
      ub.setConstant(30);
      for (int i = 0; i < 4; i++)
      {
        if (Gait::sFai[i] == 0) // 摆动腿
        {
          for (int j = 0; j < h; j++)
          {
            ub.block(12 * j + 3 * i, 1, 3, 1) = Eigen::Vector3f::Zero();
          }
        }
      }
      H = 2 * (Bqp.transpose() * Q * Bqp + R);
      g = 2 * Bqp.transpose() * Q * (Aqp * KeyboardIns::desirex[0] - D);

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_qp = H.cast<double>();
      Eigen::VectorXd g_qp = g.cast<double>();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_qp = Ampc.cast<double>();
      Eigen::VectorXd lb_qp = lb.cast<double>();
      Eigen::VectorXd ub_qp = ub.cast<double>();
      Eigen::VectorXd lba_qp = lba.cast<double>();
      Eigen::VectorXd uba_qp = uba.cast<double>();

      // 2. 调用 qpOASES
      //     qpOASES 接受的参数是 real_t *(通常是 double *)
      //         使用.data() 获取 Eigen 矩阵内部的原始指针 int nWSR_actual = nWSR;
      qpOASES::returnValue status = solver.init(
          H_qp.data(),
          g_qp.data(),
          A_qp.data(),
          lb_qp.data(),
          ub_qp.data(),
          lba_qp.data(),
          uba_qp.data(),
          nWSR);

      if (status == qpOASES::SUCCESSFUL_RETURN)
      {
        // 获取解
        double xOpt[12 * h];
        solver.getPrimalSolution(xOpt);
        Umpc << xOpt[0], xOpt[1], xOpt[2], xOpt[3], xOpt[4], xOpt[5], xOpt[6], xOpt[7], xOpt[8], xOpt[9], xOpt[10], xOpt[11];
        // 将 xOpt 传给下游控制器
      }
      else
      {
        std::cout << "MPC SOlve  Failed--------" << std::endl;
      }
    }
  };

  namespace WBC
  {
    struct node
    {
      std::weak_ptr<node> parent;
      std::vector<std::shared_ptr<node>> child;
      int const num;
      node(int const n) : num(n) {}
      void add_child(const std::shared_ptr<node> &c)
      {
        child.push_back(c);
      }
    };

    void bind(const std::shared_ptr<node> &parent, const std::shared_ptr<node> &child)
    {

      parent->add_child(child);
      child->parent = parent;
    }
    std::shared_ptr<node> node0 = std::make_shared<node>(0);
    std::shared_ptr<node> node1 = std::make_shared<node>(1);
    std::shared_ptr<node> node2 = std::make_shared<node>(2);
    std::shared_ptr<node> node3 = std::make_shared<node>(3);
    std::shared_ptr<node> node4 = std::make_shared<node>(4);
    std::shared_ptr<node> node5 = std::make_shared<node>(5);
    std::shared_ptr<node> node6 = std::make_shared<node>(6);
    std::shared_ptr<node> node7 = std::make_shared<node>(7);
    std::shared_ptr<node> node8 = std::make_shared<node>(8);
    std::shared_ptr<node> node9 = std::make_shared<node>(9);
    std::shared_ptr<node> node10 = std::make_shared<node>(10);
    std::shared_ptr<node> node11 = std::make_shared<node>(11);
    std::shared_ptr<node> node12 = std::make_shared<node>(12);
    std::shared_ptr<node> node13 = std::make_shared<node>(13);
    vector<std::shared_ptr<node>> Vnode;
    void CreatTree()
    {
      bind(node0, node1);
      bind(node1, node2);
      bind(node1, node5);
      bind(node1, node8);
      bind(node1, node11);
      bind(node2, node3);
      bind(node3, node4);
      bind(node5, node6);
      bind(node6, node7);
      bind(node8, node9);
      bind(node9, node10);
      bind(node11, node12);
      bind(node12, node13);
      Vnode.push_back(node0);
      Vnode.push_back(node1);
      Vnode.push_back(node2);
      Vnode.push_back(node3);
      Vnode.push_back(node4);
      Vnode.push_back(node5);
      Vnode.push_back(node6);
      Vnode.push_back(node7);
      Vnode.push_back(node8);
      Vnode.push_back(node9);
      Vnode.push_back(node10);
      Vnode.push_back(node11);
      Vnode.push_back(node12);
      Vnode.push_back(node13);
    }

    Eigen::Matrix3f TFX(float angle)
    {
      Eigen::Matrix3f m;
      m << 1, 0, 0,
          0, cos(angle), -sin(angle),
          0, sin(angle), cos(angle);
      return m;
    }

    Eigen::Matrix3f TFY(float angle)
    {
      Eigen::Matrix3f m;
      m << cos(angle), 0, sin(angle),
          0, 1, 0,
          -sin(angle), 0, cos(angle);
      return m;
    }

    // parent node to child node matrix
    Eigen::MatrixXf Transform_P2C(std::shared_ptr<node> node)
    {
      Eigen::MatrixXf TFMatrix(6, 6);
      if (node->num == 0)
      {
        return Eigen::MatrixXf::Identity(6, 6);
      }
      else if (node->num == 1)
      {
        TFMatrix << KF::B2W.inverse(), Eigen::Matrix3f::Zero(),
            -KF::B2W.inverse() * KF::skewSymmetric(KF::pcom), KF::B2W.transpose();
        return TFMatrix;
      }
      else if (node->num == 2 || node->num == 5 || node->num == 8 || node->num == 11)
      {
        Eigen::Vector3f P;
        if (node->num == 2)
          P << hx, -hy, 0;
        else if (node->num == 5)
          P << hx, hy, 0;
        else if (node->num == 8)
          P << -hx, -hy, 0;
        else if (node->num == 11)
          P << -hx, hy, 0;

        TFMatrix << TFX(KF::jointpos[node->num - 2]).transpose(), Eigen::Matrix3f::Zero(),
            -TFX(KF::jointpos[node->num - 2]).transpose() * KF::skewSymmetric(P), TFX(KF::jointpos[node->num - 2]).transpose();
        return TFMatrix;
      }
      else
      {
        Eigen::Vector3f P;
        if (node->num == 3)
          P << 0, -l1, 0;
        else if (node->num == 6)
          P << 0, l1, 0;
        else if (node->num == 9)
          P << 0, -l1, 0;
        else if (node->num == 12)
          P << 0, l1, 0;
        else
        {
          P << 0, 0, -l2;
        }

        TFMatrix << TFY(KF::jointpos[node->num - 2]).transpose(), Eigen::Matrix3f::Zero(),
            -TFY(KF::jointpos[node->num - 2]).transpose() * KF::skewSymmetric(P), TFY(KF::jointpos[node->num - 2]).transpose();
        return TFMatrix;
      }
    }

    // Child node to Parent node matrix
    Eigen::MatrixXf Transform_C2P(std::shared_ptr<node> node)
    {
      Eigen::MatrixXf TFMatrix(6, 6);
      if (node->num == 0)
      {
        return Eigen::MatrixXf::Identity(6, 6);
      }
      else if (node->num == 1)
      {
        TFMatrix << KF::B2W, Eigen::Matrix3f::Zero(),
            KF::B2W * KF::skewSymmetric(KF::pcom), KF::B2W;
        return TFMatrix;
      }
      else if (node->num == 2 || node->num == 5 || node->num == 8 || node->num == 11)
      {
        Eigen::Vector3f P;
        if (node->num == 2)
          P << hx, -hy, 0;
        else if (node->num == 5)
          P << hx, hy, 0;
        else if (node->num == 8)
          P << -hx, -hy, 0;
        else if (node->num == 11)
          P << -hx, hy, 0;

        TFMatrix << TFX(KF::jointpos[node->num - 2]), Eigen::Matrix3f::Zero(),
            TFX(KF::jointpos[node->num - 2]) * KF::skewSymmetric(P), TFX(KF::jointpos[node->num - 2]);
        return TFMatrix;
      }
      else
      {
        Eigen::Vector3f P;
        if (node->num == 3)
          P << 0, -l1, 0;
        else if (node->num == 6)
          P << 0, l1, 0;
        else if (node->num == 9)
          P << 0, -l1, 0;
        else if (node->num == 12)
          P << 0, l1, 0;
        else
        {
          P << 0, 0, -l2;
        }

        TFMatrix << TFY(KF::jointpos[node->num - 2]), Eigen::Matrix3f::Zero(),
            TFY(KF::jointpos[node->num - 2]) * KF::skewSymmetric(P), TFY(KF::jointpos[node->num - 2]);
        return TFMatrix;
      }
    }

    // Child node to Parent node matrix
    Eigen::MatrixXf Transform_C2PF(std::shared_ptr<node> node)
    {
      Eigen::MatrixXf TFMatrix(6, 6);
      if (node->num == 0)
      {
        return Eigen::MatrixXf::Identity(6, 6);
      }
      else if (node->num == 1)
      {
        TFMatrix << KF::B2W,
            KF::B2W * KF::skewSymmetric(KF::pcom),
            Eigen::Matrix3f::Zero(),
            KF::B2W;
        return TFMatrix;
      }
      else if (node->num == 2 || node->num == 5 || node->num == 8 || node->num == 11)
      {
        Eigen::Vector3f P;
        if (node->num == 2)
          P << hx, -hy, 0;
        else if (node->num == 5)
          P << hx, hy, 0;
        else if (node->num == 8)
          P << -hx, -hy, 0;
        else if (node->num == 11)
          P << -hx, hy, 0;

        TFMatrix << TFX(KF::jointpos[node->num - 2]),
            TFX(KF::jointpos[node->num - 2]) * KF::skewSymmetric(P),
            Eigen::Matrix3f::Zero(),
            TFX(KF::jointpos[node->num - 2]);
        return TFMatrix;
      }
      else
      {
        Eigen::Vector3f P;
        if (node->num == 3)
          P << 0, -l1, 0;
        else if (node->num == 6)
          P << 0, l1, 0;
        else if (node->num == 9)
          P << 0, -l1, 0;
        else if (node->num == 12)
          P << 0, l1, 0;
        else
        {
          P << 0, 0, -l2;
        }

        TFMatrix << TFY(KF::jointpos[node->num - 2]),
            TFY(KF::jointpos[node->num - 2]) * KF::skewSymmetric(P),
            Eigen::Matrix3f::Zero(),
            TFY(KF::jointpos[node->num - 2]);
        return TFMatrix;
      }
    }

    // make_shared<node>
    // 用零空间求解多任务带优先级的位置 速度 加速度
    // 按任务优先级
    //  广义qdot qddot
    Eigen::MatrixXf qdot(18, 1), qddot(18, 1), qcmd(18, 1), qdotcmd(18, 1),
        qddotcmd(18, 1), detqcmd(18, 1), q(18, 1), qdotcmde(18, 1), qddotcmde(18, 1);
    // 4个任务雅可比矩阵
    Eigen::MatrixXf J1, J2(3, 18), J3(3, 18), J4, J1q, J2q(3, 1), J3q(3, 1), J4q, e(3, 1), x(3, 1), JA, NA;
    Eigen::Matrix3f tran2; // 任务2 用的
    Eigen::MatrixXf Q1(6, 6), Q2(12, 12), G(18, 18), CE, Ce, CI, Ci, Mf, Jcf, Cf, CA, _CA, CA_;
    float kp, kd;

    // 空间速度 空间加速度 {0} 到任意坐标系的变换矩阵   Si从 i=1 开始
    vector<Eigen::MatrixXf> Vspace, Aspace, Aspaced, X02I, X02If, Si, fi, Vji, qidot, Jb, XQi, Jbi, AspaceQ, VspaceQ, I, Ic;
    Eigen::MatrixXf XCi(6, 6), C(18, 1), M(18, 18);
    vector<int> pi;

    Eigen::MatrixXf WideInverse(const Eigen::MatrixXf &mat)
    {
      return mat.transpose() * ((mat * mat.transpose()).inverse());
    }
    Eigen::MatrixXf Vcross(const Eigen::MatrixXf &V)
    {
      Eigen::MatrixXf Vx(6, 6);
      Vx << KF::skewSymmetric(Eigen::Vector3f(V.block(0, 0, 3, 1))), Eigen::Matrix3f::Zero(),
          KF::skewSymmetric(Eigen::Vector3f(V.block(3, 0, 3, 1))), KF::skewSymmetric(Eigen::Vector3f(V.block(0, 0, 3, 1)));
      return Vx;
    }

    Eigen::MatrixXf Fcross(const Eigen::MatrixXf &V)
    {
      Eigen::MatrixXf Vx(6, 6);
      Vx << KF::skewSymmetric(Eigen::Vector3f(V.block(0, 0, 3, 1))), KF::skewSymmetric(Eigen::Vector3f(V.block(3, 0, 3, 1))),
          Eigen::Matrix3f::Zero(), KF::skewSymmetric(Eigen::Vector3f(V.block(0, 0, 3, 1)));
      return Vx;
    }
    void WBC_Init()
    {
      Si[0] = Eigen::MatrixXf::Zero(6, 6); // 没有 Si0 随便初始
      Si[1] = Eigen::MatrixXf::Identity(6, 6);
      Si[2] << 1, 0, 0, 0, 0, 0;
      Si[5] << 1, 0, 0, 0, 0, 0;
      Si[8] << 1, 0, 0, 0, 0, 0;
      Si[11] << 1, 0, 0, 0, 0, 0;

      Si[3] << 0, 1, 0, 0, 0, 0;
      Si[4] << 0, 1, 0, 0, 0, 0;
      Si[6] << 0, 1, 0, 0, 0, 0;
      Si[7] << 0, 1, 0, 0, 0, 0;
      Si[9] << 0, 1, 0, 0, 0, 0;
      Si[10] << 0, 1, 0, 0, 0, 0;
      Si[12] << 0, 1, 0, 0, 0, 0;
      Si[13] << 0, 1, 0, 0, 0, 0;

      X02I[0] = Eigen::MatrixXf::Identity(6, 6);
      X02If[0] = Eigen::MatrixXf::Identity(6, 6);
      Vspace[0].resize(6, 1);
      Aspace[0].resize(6, 1);
      Vspace[0].setZero(6, 1);
      Aspace[0].setZero(6, 1);
      Aspaced[0].resize(6, 1);
      Aspaced[0] << 0, 0, 0, 0, 0, 9.81;
      J2 << KF::B2W, Eigen::MatrixXf::Zero(3, 15);
      J3 << Eigen::Matrix3f::Zero(), KF::B2W, Eigen::MatrixXf::Zero(3, 12);
      pi.push_back(4);
      pi.push_back(7);
      pi.push_back(10);
      pi.push_back(13);
      Jb[0].resize(6, 18);
      Jb[1].resize(6, 18);
      Jb[2].resize(6, 18);
      Jb[3].resize(6, 18);
      Jbi[0].resize(3, 18);
      Jbi[1].resize(3, 18);
      Jbi[2].resize(3, 18);
      Jbi[3].resize(3, 18);
      XCi << Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(),
          -KF::skewSymmetric(Eigen::Vector3f(0, 0, -l3)), Eigen::Matrix3f::Identity();
      J2q << Eigen::MatrixXf::Zero(3, 1);
      J3q << Eigen::MatrixXf::Zero(3, 1);

      //
      vector<Eigen::Vector3f> P;
      vector<Eigen::Vector4f> quat;
      vector<float> mass;
      vector<Eigen::Vector3f> diagnertia;
      P[0] << 0.021112, 0, -0.005366;
      mass[0] = 6.921;
      diagnertia[0] << 0.107027, 0.0980771, 0.0244531;
      P[1] << 0.0054, -0.00194, -0.000105;
      mass[1] = 0.678;
      diagnertia[1] << 0.00088403, 0.000596003, 0.000479967;
      P[2] << -0.00374, 0.0223, -0.0327;
      mass[2] = 1.152;
      diagnertia[2] << 0.00594973, 0.00584149, 0.000878787;
      P[3] << 0.00629595, 0.000622121, -0.141417;
      mass[3] = 0.241352;
      diagnertia[3] << 0.0014901, 0.00146356, 5.31397e-05;

      quat[0] << -0.000543471, 0.713435, -0.00173769, 0.700719;
      quat[1] << 0.498237, 0.505462, 0.499245, 0.497014;
      quat[2] << 0.551623, -0.0200632, 0.0847635, 0.829533;
      quat[3] << 0.703508, -0.00450087, 0.00154099, 0.710672;
      quat[4] << 0.497014, 0.499245, 0.505462, 0.498237;
      quat[5] << 0.829533, 0.0847635, -0.0200632, 0.551623;
      quat[6] << 0.710672, 0.00154099, -0.00450087, 0.703508;
      quat[7] << 0.499245, 0.497014, 0.498237, 0.505462;
      quat[8] << 0.551623, -0.0200632, 0.0847635, 0.829533;
      quat[9] << 0.703508, -0.00450087, 0.00154099, 0.710672;
      quat[10] << 0.505462, 0.498237, 0.497014, 0.499245;
      quat[11] << 0.829533, 0.0847635, -0.0200632, 0.551623;
      quat[12] << 0.710672, 0.00154099, -0.00450087, 0.703508;

      for (int i = 0; i < 13; ++i)
      {
        Eigen::Matrix3f R = ConvexMPC::QUa2Mat(quat[i][0], quat[i][1], quat[i][2], quat[i][3]);
        int r = (i == 0) ? 0 : ((i - 1) % 3 + 1);
        I[i] = (R * diagnertia[r].asDiagonal() * R.transpose()) + mass[r] * (P[r].transpose() * P[r] * Eigen::Matrix3f::Identity() - P[r] * P[r].transpose());
      }
    }
    // multi-Rigid-Body dynamics algorithm
    void Dynamcis_Update()
    {
      int J1num = 0;
      int J4num = 0;
      qidot[0] << KF::Wb, KF::B2W.transpose() * KF::vcom;
      for (int i = 0; i < 12; ++i)
      {
        qidot[i + 1] << KF::jointvel[i];
      }

      for (int i = 1; i < 14; ++i)
      {
        X02I[i] = Transform_P2C(Vnode[i]) * X02I[Vnode[i]->parent.lock()->num];
        X02If[i] = (X02I[i].inverse()).transpose();
        Vji[i - 1] = Si[i] * qidot[i - 1];
        Vspace[i] = Transform_P2C(Vnode[i]) * Vspace[Vnode[i]->parent.lock()->num] + Vji[i - 1];
        // 忽略了 qddot 为了求 J1q J4q

        Aspace[i] = Transform_P2C(Vnode[i]) * Aspace[Vnode[i]->parent.lock()->num] + Vcross(Vspace[i]) * (Si[i] * qidot[i - 1]);
        Aspaced[i] = Transform_P2C(Vnode[i]) * Aspaced[Vnode[i]->parent.lock()->num] + Vcross(Vspace[i]) * (Si[i] * qidot[i - 1]);
        // 旋转惯量 没写
        fi[i - 1] = I[i - 1] * Aspaced[i] + Fcross(Vspace[i]) * (I[i - 1] * Vspace[i]);
      }
      // 反推 更新C矩阵  更新 Ic 组合刚体 空间惯量
      for (int i = 13; i > 0; --i)
      {
        if (i == 1)
          C.block(0, 0, 6, 1) = Si[i].transpose() * fi[i - 1];
        else
          C.block(i + 4, 0, 1, 1) = Si[i].transpose() * fi[i - 1];
        if (Vnode[i]->parent.lock()->num != 0)
        {
          fi[Vnode[i]->parent.lock()->num - 1] = fi[Vnode[i]->parent.lock()->num - 1] + Transform_C2P(Vnode[i]) * fi[i - 1];
        }

        Ic[i - 1] = I[i - 1];
        for (auto node : Vnode[i]->child)
        { // 这里 i 的child 是j 们， 所以i->j  和j->i 是P2C 或者 C2P 的关系 ，直接用
          Ic[i - 1] = Ic[i - 1] + Transform_C2PF(Vnode[node->num]) * Ic[node->num] * Transform_P2C(Vnode[node->num]);
        }

        if (i == 1)
        {
          M.block(0, 0, 6, 6) = Si[i].transpose() * Ic[i - 1] * Si[i];
        }
        else
        {
          M.block(i + 4, i + 4, 1, 1) = Si[i].transpose() * Ic[i - 1] * Si[i];
        }
        int j = i;
        Eigen::MatrixXf Xt(6, 6) = Eigen::MatrixXf::Identity(6, 6);
        while (Vnode[j]->parent.lock()->num > 0)
        {
          Xt = Xt * Transform_P2C(Vnode[j]);
          if (Vnode[j]->parent.lock()->num == 1)
          {
            M.block(i + 4, 0, 1, 6) = Si[i].transpose() * Ic[i - 1] * Xt * Si[Vnode[j]->parent.lock()->num];

            M.block(0, i + 4, 6, 1) = M.block(i + 4, 0, 1, 6).transpose();
          }
          else
          {
            M.block(i + 4, Vnode[j]->parent.lock()->num + 4, 1, 1) = Si[i].transpose() * Ic[i - 1] * Xt * Si[Vnode[j]->parent.lock()->num];

            M.block(Vnode[j]->parent.lock()->num + 4, i + 4, 1, 1) = Si[i].transpose() * Ic[i - 1] * Xt * Si[Vnode[j]->parent.lock()->num];
          }

          j = Vnode[j]->parent.lock()->num;
        }
      }

      for (int i = 0; i < 4; ++i)
      {
        int j = pi[i];
        XQi[i] << X02I[pi[i]].block(3, 3, 3, 3).transpose(), Eigen::Matrix3f::Zero(),
            Eigen::Matrix3f::Zero(), X02I[pi[i]].block(3, 3, 3, 3).transpose();
        Eigen::MatrixXf Xjpi = Eigen::MatrixXf::Identity(6, 6);
        Jb[i].block(0, pi[i] + 4, 6, 1) = Si[pi[i]];
        while (Vnode[j]->parent.lock()->num > 0)
        {
          Xjpi = Xjpi * Transform_P2C(Vnode[j]);
          j = Vnode[j]->parent.lock()->num;
          if (j == 1)
            Jb[i].block(0, 0, 6, 6) = Xjpi * Si[j];
          else
            Jb[i].block(0, j + 4, 6, 1) = Xjpi * Si[j];
        }
        // 转向定向足底坐标系
        Jbi[i] = (XQi[i] * XCi * Jb[i]).block(3, 0, 3, 18);
        AspaceQ[i] = (XQi[i] * XCi) * Aspace[pi[i]];
        VspaceQ[i] = (XQi[i] * XCi) * Vspace[pi[i]];

        if (Gait::sFai[i] == 1) // 支撑腿
        {
          J1.block(3 * J1num, 0, 3, 18) = Jbi[i];
          J1q.block(3 * J1num, 0, 3, 1) = AspaceQ[i].block(3, 0, 3, 1) + VspaceQ[i].block(0, 0, 3, 1).cross(VspaceQ[i].block(3, 0, 3, 1));
          J1num++;
        }
        else // 摆动腿
        {
          J4.block(3 * J4num, 0, 3, 18) = Jbi[i];
          J4q.block(3 * J4num, 0, 3, 1) = AspaceQ[i].block(3, 0, 3, 1) + VspaceQ[i].block(0, 0, 3, 1).cross(VspaceQ[i].block(3, 0, 3, 1));

          J4num++;
        }
      }
    }
    void WBC_Update()
    {
      // 第一个任务 支撑腿跟随
      detqcmd.setZero();
      qdotcmde.setZero();
      qddotcmde = WideInverse(J1) * (-J1q);
      for (int i = 0; i < 3; ++i)
      {
        switch (i)
        {
        case 0:
        { // 第二个 机身转动
          tran2 << cos(KeyboardIns::desirex[0][1]) * cos(KeyboardIns::desirex[0][2]), -sin(KeyboardIns::desirex[0][2]), 0,
              cos(KeyboardIns::desirex[0][1]) * sin(KeyboardIns::desirex[0][2]), cos(KeyboardIns::desirex[0][2]), 0,
              -sin(KeyboardIns::desirex[0][1]), 0, 1;
          Eigen::Vector3f dfai = {KeyboardIns::desirex[1][0], KeyboardIns::desirex[1][1], KeyboardIns::desirex[1][2]};
          Eigen::Vector3f fai = {KeyboardIns::desirex[0][0], KeyboardIns::desirex[0][1], KeyboardIns::desirex[0][2]};
          Eigen::Vector3f dwO = {KeyboardIns::desirex[1][6], KeyboardIns::desirex[1][7], KeyboardIns::desirex[1][8]};

          e = tran2 * (dfai - fai);
          x = kd * (dwO - KF::B2W * KF::Wb) + kp * (e);
          JA.block(0, 0, J1.rows(), 18) = J1;
          NA = Eigen::MatrixXf::Identity(JA.cols(), JA.cols()) - WideInverse(JA) * JA;
          detqcmd = detqcmd + WideInverse(J2 * NA) * (e - J2 * detqcmd);
          qdotcmde = qdotcmde + WideInverse(J2 * NA) * (dwO - J2 * qdotcmde);
          qddotcmde = qddotcmde + WideInverse(J2 * NA) * (x - J2q - J2 * qddotcmde);
          break;
        }
        case 1: // 机身平动
        {
          Eigen::Vector3f dPo = {KeyboardIns::desirex[1][3], KeyboardIns::desirex[1][4], KeyboardIns::desirex[1][5]};
          Eigen::Vector3f dVO = {KeyboardIns::desirex[1][9], KeyboardIns::desirex[1][10], KeyboardIns::desirex[1][11]};

          e = dPo - KF::pcom;
          x = kd * (dVO - KF::vcom) + kp * (dPo - KF::pcom);
          JA.block(J1.rows(), 0, 3, 18) = J2;
          NA = Eigen::MatrixXf::Identity(JA.cols(), JA.cols()) - WideInverse(JA) * JA;
          detqcmd = detqcmd + WideInverse(J3 * NA) * (e - J3 * detqcmd);
          qdotcmde = qdotcmde + WideInverse(J3 * NA) * (dVO - J3 * qdotcmde);
          qddotcmde = qddotcmde + WideInverse(J3 * NA) * (x - J3q - J3 * qddotcmde);
          break;
        }
        case 2: // 摆动腿
        {
          Eigen::VectorXf ee, Pfoot, dPfoot, Vfoot, dVfoot;
          int num = 0;
          for (int j = 0; j < 4; ++j)
          {
            if (Gait::sFai[j] == 0)
            {
              Pfoot.block(3 * num, 0, 3, 1) = KF::pcom + KF::B2W * KF::iPb[j];
              Vfoot.block(3 * num, 0, 3, 1) = XQi[j] * XCi * Vspace[pi[j]].block(3, 0, 3, 1);
              dPfoot.block(3 * num, 0, 3, 1) = Gait::FootdesirePos[j];
              dVfoot.block(3 * num, 0, 3, 1) = Gait::FootdesireVelocity[j];
              num++;
            }
          }
          ee = dPfoot - Pfoot;
          x = kd * (dVfoot - Vfoot) + kp * (ee);
          JA.block(J1.rows() + 3, 0, 3, 18) = J3;
          NA = Eigen::MatrixXf::Identity() - WideInverse(JA) * JA;
          detqcmd = detqcmd + WideInverse(J4 * NA) * (ee - J4 * detqcmd);
          qdotcmd = qdotcmde + WideInverse(J4 * NA) * (dVfoot - J4 * qdotcmde);
          qddotcmd = qddotcmde + WideInverse(J4 * NA) * (x - J4q - J4 * qddotcmde);
          break;
        }
        }
      }
      qcmd = q + detqcmd;

      // quadprog求解 松弛优化变量
      Q1 = Eigen::MatrixXf::Identity(6, 6);
      Q2 = Eigen::MatrixXf::Identity(12, 12) * 0.005;
      G.setZero();
      G.block(0, 0, 6, 6) = Q1;
      G.block(6, 6, 12, 12) = Q2;
      CE = Mf - Jcf.transpose();
      Ce = -Jcf.transpose() * ConvexMPC::Umpc + Cf + Mf * qddotcmd;

      double cost = quadprogpp::solve_quadprog(G_qp, g_qp, CE_qp, ce0_qp, CI_qp, ci0_qp, x_qp);
    }

  };
};
