#include "Quad.h"
#include "xbox.h"

namespace Quad
{

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
    FSMstate lastState;
    void Init_FSM()
    {
      StateTransformMatrix << 0, 1, 0, 1, 0, 1, 0, 1, 0;
      currentState = stand;
      lastState = currentState;
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
    float dfooth = 0.03; // 期望抬腿高度

    void Gait_Init()
    {

      FootdesirePos.resize(4, Eigen::Vector3f::Zero());
      FootdesireVelocity.resize(4, Eigen::Vector3f::Zero());
      Pstend.resize(4, Eigen::Vector3f::Zero());
      Pstsw.resize(4, Eigen::Vector3f::Zero());
      Pcomtouch.resize(4, Eigen::Vector3f::Zero());
      Psymtouch.resize(4, Eigen::Vector3f::Zero());
      Pswend.resize(4, Eigen::Vector3f::Zero());
      P1.resize(4, Eigen::Vector3f::Zero());
      P2.resize(4, Eigen::Vector3f::Zero());
      P3.resize(4, Eigen::Vector3f::Zero());
      P4.resize(4, Eigen::Vector3f::Zero());

      Gd << 0.5, 0.5, 0.5, 0.5,
          0.75, 0.75, 0.75, 0.75;
      Go << 0.5, 0, 0, 0.5,
          0.25, 0.75, 0.5, 0;
      FrPstend = KF::X.block(6, 0, 3, 1);
      FlPstend = KF::X.block(9, 0, 3, 1);
      RrPstend = KF::X.block(12, 0, 3, 1);
      RlPstend = KF::X.block(15, 0, 3, 1);

      for (int i = 0; i < 4; i++)
      {
        sFai[i] = 1; // 强制 4 条腿全部为支撑腿
      }
    }
    char currentGait = 0; // 默认选择一个步态
    // 进入当前步态的持续时间
    float Time = 0;
    // 步态周期
    float GaitPeriod = 0;

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
      //
      if (FSM::currentState == FSM::FSMstate::move)
      {
        Time += 0.002;

        if (Time > 1000.0f * GaitPeriod)
          Time = fmod(Time, GaitPeriod);
      }
      else
      {
        Time = 0.0f;
      }
    }
    // 改变步态 gait =0   trot    gait =1 walk
    void ChangeGait(int gait)
    {
      currentGait = gait;
      Time = 0;
      if (gait == 0)
      {
        GaitPeriod = 0.5;
        stperiod = 0.25;
        swperiod = 0.25;

        // GaitPeriod = 1;
        // stperiod = 0.75;
        // swperiod = 0.25;
      }
      else
      {
        GaitPeriod = 0.8;
        stperiod = 0.6;
        swperiod = 0.2;
      }
    }
    /// @brief update Gait 在进入move的状态时 才运行

    void UpdateGait()
    {
      if (GaitPeriod <= 1e-6f)
      {
        sFai.setOnes();
        tst.setOnes();
        tsw.setZero();
        return;
      }

      tng = fmod(Time, GaitPeriod);
      _tng = tng / GaitPeriod;
      // n = Time / GaitPeriod + 1;
      // tng = Time - (n - 1) * GaitPeriod;
      // _tng = tng / GaitPeriod; // 当前进度
      for (int i = 0; i < 4; i++)
      {
        if (_tng >= Go(currentGait, i))
          tFai[i] = _tng - Go(currentGait, i);
        else
          tFai[i] = _tng - Go(currentGait, i) + 1.0f;

        if (tFai[i] <= Gd(currentGait, i))
          sFai[i] = 1.0f;
        else
          sFai[i] = 0.0f;

        if (sFai[i] == 1.0f)
          tst[i] = tFai[i] / Gd(currentGait, i);
        else
          tst[i] = 0.0f;

        if (sFai[i] == 0.0f)
          tsw[i] = (tFai[i] - Gd(currentGait, i)) / (1.0f - Gd(currentGait, i));
        else
          tsw[i] = 0.0f;
      }

      if (FSM::currentState == FSM::FSMstate::stand)
      {
        for (int i = 0; i < 4; i++)
        {
          sFai[i] = 1;   // 强制 4 条腿全部为支撑腿
          tsw[i] = 0.0f; // 摆动进度清零
          tst[i] = 1.0f; // 处于完全支撑相

          // 将名义期望落足点锁定为当前实际抓地位置，防止滑移
          FootdesirePos[i] = Pstend[i];
          FootdesireVelocity[i].setZero();
        }
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
    float kp = 0.169;
    // 对称足底位置的XY坐标向量
    /// @brief 落足点规划 和 足底轨迹规划
    bool join = false;
    void FootTraj_Planning()
    {

      if (!join)
      {
        float off_set = -0.02;
        SymPb1 << hx + off_set, -hy - l1, 0;
        SymPb2 << hx + off_set, hy + l1, 0;
        SymPb3 << -hx + off_set, -hy - l1, 0;
        SymPb4 << -hx + off_set, hy + l1, 0;
        SymPb.push_back(SymPb1);
        SymPb.push_back(SymPb2);
        SymPb.push_back(SymPb3);
        SymPb.push_back(SymPb4);
        join = true;
      }

      for (int i = 0; i < 4; i++)
      {
        // std::cout << "pstend--->" + i << Pstend[i] << std::endl;
        // 无论支撑腿还是摆动腿 都要更新  质心触地位置 和 偏航角
        // Pcomtouch[i] = KF::pcom + KF::B2W * KeyboardIns::dVb * ((1 - tsw[i]) * swperiod);/
        Pcomtouch[i] = KF::pcom + KeyboardIns::dVO * ((1 - tsw[i]) * swperiod);

        FaiZtouch[i] = KF::Faiz + KeyboardIns::dWzb * ((1 - tsw[i]) * swperiod);
        // 世界坐标系下的每个足的对称点坐标
        /// @note  足底轨迹规划
        // 落足点坐标 无论摆动还是支撑腿 都应该一直预测  。 摆动时 要预防突发情况，及时改变落足点
        Psymtouch[i] = Pcomtouch[i] + TF_Z(FaiZtouch[i]) * SymPb[i];
        P1[i] = (KeyboardIns::dVO * stperiod) / 2.0f;
        // P1[i] = (KF::vcom * stperiod) / 2.0f;

        P2[i] = TF_Z(FaiZtouch[i]) * ((TF_Z(KeyboardIns::dWzO * stperiod / 2.0f) * SymPb[i]) - SymPb[i]);
        P3[i] = kp * (KF::vcom - KeyboardIns::dVO);

        P4[i] = (KF::pcom[2] / 9.81f) * (KF::vcom.cross(KF::B2W * KF::Wb));

        // 最终的落足点坐标
        Pswend[i] = Psymtouch[i] + P1[i] + P2[i] + P3[i] + P4[i];
        Pswend[i][2] = KeyboardIns::A[0] + KeyboardIns::A[1] * (Pswend[i][0]) + KeyboardIns::A[2] * (Pswend[i][1]);

        Pstsw[i] = Pswend[i] - Pstend[i];
        // 摆动腿 根据 终点和起点进行轨迹规划
        if (sFai[i] == 0)
        {
          float u = tsw[i];

          FootdesirePos[i][0] = Pstend[i][0] + Pstsw[i][0] * (3.0f * pow(u, 2) - 2.0f * pow(u, 3));
          FootdesirePos[i][1] = Pstend[i][1] + Pstsw[i][1] * (3.0f * pow(u, 2) - 2.0f * pow(u, 3));
          FootdesireVelocity[i][0] = Pstsw[i][0] * (6.0f * u - 6.0f * pow(u, 2)) / swperiod;
          FootdesireVelocity[i][1] = Pstsw[i][1] * (6.0f * u - 6.0f * pow(u, 2)) / swperiod;

          float h_start = Pstend[i][2];
          float h_end = Pswend[i][2];

          // 位置：起点到终点的线性插值 + 完美的半个正弦波抬高
          FootdesirePos[i][2] = h_start + (h_end - h_start) * u + dfooth * sin(M_PI * u);
          // 速度：严格遵守微积分链式法则求导
          FootdesireVelocity[i][2] = ((h_end - h_start) + dfooth * M_PI * cos(M_PI * u)) / swperiod;
          // float h_peak = dfooth;
          // if (u <= 0.5f)
          // {
          //   float t2 = 2.0f * u;
          //   float h_start = Pstend[i][2];
          //   FootdesirePos[i][2] = h_start + (h_peak - h_start) * (3.0f * pow(t2, 2) - 2.0f * pow(t2, 3));
          //   FootdesireVelocity[i][2] = (h_peak - h_start) * (6.0f * t2 - 6.0f * pow(t2, 2)) * 2.0f / swperiod;
          // }
          // else
          // {
          //   float t2 = 2.0f * u - 1.0f;
          //   float h_end = Pswend[i][2];
          //   FootdesirePos[i][2] = h_peak + (h_end - h_peak) * (3.0f * pow(t2, 2) - 2.0f * pow(t2, 3));
          //   FootdesireVelocity[i][2] = (h_end - h_peak) * (6.0f * t2 - 6.0f * pow(t2, 2)) * 2.0f / swperiod;
          // }
        }

        else
        {

          FootdesireVelocity[i].setZero();
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
    void B2WUpdate(const mjModel *model, mjData *data, const std::string &sensor_name)
    {
      Quat = mujo::get_sensor_data(model, data, sensor_name); // 获得角度
      Eigen::Quaternionf q(Quat[0], Quat[1], Quat[2], Quat[3]);

      // Faiz = q.toRotationMatrix().eulerAngles(2, 1, 0)[0]; // 获得当前偏航角
      //    std::cout << "faiz" << Faiz << std::endl;
      vector<float> temp = mujo::get_sensor_data(model, data, "imu_gyro");
      Wb << temp[0], temp[1], temp[2]; // 获得角速度数据 由于Imu坐标系和本体系方向相同  所以不用变换
      WbS = skewSymmetric(Wb);         // 获得反对称矩阵
      float w = Quat[0];
      float x = Quat[1];
      float y = Quat[2];
      float z = Quat[3];

      // 1. 横滚角 Roll (Faix)
      Faix = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

      // 2. 俯仰角 Pitch (Faiy) - 加入死区保护，防止浮点误差导致 asin 算出 NaN
      float sinp = 2.0f * (w * y - z * x);
      if (std::abs(sinp) >= 1.0f)
        Faiy = std::copysign(M_PI / 2.0f, sinp); // 超过90度强制截断
      else
        Faiy = std::asin(sinp);

      // 3. 偏航角 Yaw (Faiz)
      Faiz = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
      std::cout << "Faiz==" << Faiz << std::endl;
      B2W << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y,
          2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x,
          2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y;
    }

    // 获取 关节传感器数据
    void joint_sensor_data_update(const mjModel *model, mjData *data)
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
               l1 * cos(jointpos[9]) + l3 * sin(jointpos[9]) * cos(jointpos[10] + jointpos[11]) + l2 * cos(jointpos[10]) * sin(jointpos[9]) + hy,
               l1 * sin(jointpos[9]) - l3 * cos(jointpos[9]) * cos(jointpos[10] + jointpos[11]) - l2 * cos(jointpos[9]) * cos(jointpos[10])};

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
      _3x12.setZero();
      _12x3.setZero();
      Q.setZero();
      R.setZero();

      iPb.resize(4, Eigen::Vector3f::Zero());

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
      X << 0, 0, 0.254, 0, 0, 0, 0.203, -0.142, 0, 0.203, 0.142, 0, -0.184, -0.142, 0, -0.184, 0.142, 0; // 初始状态 应接近 真实状态
                                                                                                         // P = Eigen::MatrixXf::Ones(18, 18);
      P.setZero();
      P.block(0, 0, 3, 3) = 0.01f * Eigen::Matrix3f::Identity();
      P.block(3, 3, 3, 3) = 0.1f * Eigen::Matrix3f::Identity();
      P.block(6, 6, 12, 12) = 0.01f * Eigen::MatrixXf::Identity(12, 12);
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
        if (Gait::sFai[i] == 0) // 是否为摆动腿 将摆动腿测量噪声变大 降低他的贡献
        {
          // std::cout << "enter" << std::endl;
          Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = iden3;
          R.block(0 + 3 * i, 0 + 3 * i, 3, 3) = iden3 * 20000;
          R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = iden3 * 20000;
          R.block(24 + i, 24 + i, 1, 1) = Onemat * 20000;
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

      Eigen::MatrixXf S = H * _P * H.transpose() + R;
#ifdef solve
#undef solve
#endif
      Eigen::MatrixXf S_inv = S.ldlt().solve(Eigen::MatrixXf::Identity(S.rows(), S.cols()));
      K = _P * H.transpose() * S_inv;

      X = _X + K * (Z - H * _X);
      // 保证协方差矩阵的严格对称性 (Joseph form)，防止浮点误差导致 P 变得不对称发散
      P = (iden18 - K * H) * _P;
      P = 0.5f * (P + P.transpose()); // 强制对称化保护！
      // K = (_P * (H.transpose())) * ((H * _P * H.transpose() + R).inverse());
      // X = _X + K * (Z - H * _X);
      // P = (iden18 - K * H) * _P;
      pcom = X.block(0, 0, 3, 1);
      // std::cout<<"X---->"<<X<<std::endl;
      vcom = X.block(3, 0, 3, 1);
    }

  };

  // 用键盘命令代替手柄   先进行简单模拟 后边完善
  // 然后完成坡度估计
  namespace KeyboardIns
  {

    /// @brief 期望速度和角速度
    float dVxb = 0.0, dVyb = 0.0, dWzb = 0, dWzO = 0;
    float dFaiz, dHb = 0.29, dFaix, dFaiy;
    Eigen::Vector3f dVb;
    Eigen::Vector3f dVO;
    Eigen::Vector3f dWb;
    Eigen::Vector3f dWO;
    Eigen::Matrix3f TFZ;
    Eigen::Vector3f dPO;
    Eigen::MatrixXf W(4, 3);
    Eigen::VectorXf Z(4), A(3), _A(3), N(3), N_(3);
    Eigen::Vector3f Tao, dFai;
    vector<Eigen::VectorXf> desirex;
    Eigen::MatrixXf D(13 * h, 1);
    void Update_ins()
    {
      int xbox_fd;
      int len, type;
      int axis_value, button_value;
      int number_of_axis, number_of_buttons;

      memset(&map, 0, sizeof(xbox_map_t));
      bool xbox_connected = false;
      while (!xbox_connected)
      {
        xbox_fd = xbox_open("/dev/input/js0");
        if (xbox_fd < 0)
        {
          std::cout << "Failed to open Xbox controller. Make sure it is connected and the correct device path is used.\n";
          usleep(10 * 1000);
        }
        else
        {
          xbox_connected = true;
          std::cout
              << "Xbox controller connected successfully.\n";
        }
      }

      while (1)
      {
        auto startTime = std::chrono::steady_clock::now();

        len = xbox_map_read(xbox_fd, &map);

        if (len < 0)
        {
          usleep(10 * 1000);
          continue;
        }

        startTime += std::chrono::milliseconds(10); // 100hz
        std::this_thread::sleep_until(startTime);
      }

      xbox_close(xbox_fd);
    }
    Eigen::Vector3f lock_p;
    void Keyboard_init()
    {
      A.setZero();
      _A.setZero();
      N.setZero();
      N_.setZero();
      // h 为预测步长，需要记录 h+1 个状态，每个状态为 13 维向量
      desirex.resize(h + 1, Eigen::VectorXf::Zero(13));
      // dPO = {0, 0, 0.29};
      dFaiz = KF::Faiz; // 用当前偏航角初始 目标偏航角
      dWzO = dWzb;
      lock_p = KF::pcom;
    }
    float clip(float value, float min, float max)
    {
      return value < min ? min : (value > max ? max : value);
    }
    bool Lock_state = true;

    void Desire_ins_update(float MPCtime) // 这个仅能根据当前状态 估计下一次状态  ， 但是需要估计未来多个时间段的状态
    {

      float dy = clip(((float)-map.lx) / 32767.0f, -1.f, 1.f);
      float dx = clip(((float)-map.ly) / 32767.0f, -1.f, 1.f);
      float dz = clip(((float)-map.lt) / 32767.0f, -1.f, 1.f);
      dVxb = 0.7 * dVxb + 0.3 * dx;
      dVyb = 0.7 * dVyb + 0.3 * dy;
      dWzb = 0.7 * dWzb + 0.3 * dz;
      if (dVxb < 0.001f)
        dVxb = 0;
      if (dVyb < 0.001f)
        dVyb = 0;
      if (dWzb < 0.001f)
        dWzb = 0;
      std::cout << "dVxb-->" << dVxb << std::endl;
      std::cout << "dVyb-->" << dVyb << std::endl;
      std::cout << "dWzb-->" << dWzb << std::endl;

      if (dVxb == 0 && dVyb == 0 && dWzb == 0 && FSM::currentState == FSM::FSMstate::move)
      {
        // std::
        //   SystemControl::first_mpc = false;
        FSM::currentState = FSM::FSMstate::stand;
        lock_p = KF::pcom;
        //  Lock_state = true;
      }
      else if ((dVxb != 0 || dVyb != 0 || dWzb != 0) && FSM::currentState == FSM::FSMstate::stand)
      {
        //  SystemControl::first_mpc = false;
        FSM::currentState = FSM::FSMstate::move;
        Gait::ChangeGait(0);
        // Lock_state = false;
      }
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
          Eigen::Matrix3f WtW = W.transpose() * W;
          float det = WtW.determinant();
          if (det > 1e-8f)
          {
            Eigen::Vector3f A_new = WtW.inverse() * W.transpose() * Z;
            A = 0.2f * A_new + 0.8f * _A;
          }
          else
          {
            A = _A; // 保留上一帧，不更新
          }
          _A = A;
          // _A = A;
          // // 可能有错
          // A = (W.transpose() * W).inverse() * W.transpose() * Z;
          // // A = (W.transpose() * ((W * W.transpose()).inverse())) * Z;
          // A = 0.2 * A + 0.8 * _A; // 低通滤波
          N << -A[1], -A[2], 1;
          // 归一化的法向量
          N_ = N * (1.0f / pow((pow(N[0], 2) + pow(N[1], 2) + 1), 0.5));
          // std::cout<<"N---->"<<N_<<std::endl;
          dPO = KF::pcom;
          // Tao = TFZ.inverse() * N_;
          // dFaix = asin(Tao[1]);
          // dFaiy = atan(Tao[0] / Tao[2]);
          // dFai << dFaix, dFaiy, dFaiz;
          Eigen::Vector3f fai = {KF::Faix, KF::Faiy, KF::Faiz};

          desirex[i] << fai, KF::pcom, KF::B2W * KF::Wb, KF::vcom, -9.81; // 此为当前状态
        }
        dFaiz = dFaiz + dWzO * MPCtime;
        // Z轴旋转矩阵
        TFZ << cos(dFaiz), -sin(dFaiz), 0,
            sin(dFaiz), cos(dFaiz), 0,
            0, 0, 1;
        // 更新期望位置
        dVO = TFZ * dVb;

        dPO = dPO + dVO * MPCtime;
        if (FSM::currentState == FSM::FSMstate::stand)
          dPO = lock_p;
        // 计算期望滚摆角
        dPO[2] = dHb; // 期望机身高度
        Tao = TFZ.inverse() * N_;
        dFaix = asin(Tao[1]);
        dFaiy = atan(Tao[0] / Tao[2]);
        if (FSM::currentState == FSM::FSMstate::stand)
        {
          dFaix = 0;
          dFaiy = 0;
        } // 站立时强行保持水平

        dFai << dFaix, dFaiy, dFaiz;
        // 更新期望状态向量
        desirex[i + 1] << dFai, dPO, dWO, dVO, -9.81;
      }
      for (int i = 0; i < h; i++)
      {
        D.block(13 * i, 0, 13, 1) = desirex[i + 1];
      }
    }
  };

  namespace ConvexMPC
  {
    Eigen::MatrixXf Temp_A(13, 13), A(13, 13), Continue_B(13, 13);
    Eigen::Matrix3f BInertia, PInertia;

    // 动态维度矩阵，后续将在 UpdateState 中根据支撑腿数量 resize
    Eigen::MatrixXf Q, R, Aqp, Bqp, D, H, g, lb, ub, lba, uba, Ampc;

    Eigen::VectorXf vec(13);
    Eigen::MatrixXf temp(13, 13);
    Eigen::MatrixXf Umpc(12, 1); // 保留用于日志或调试
    Eigen::VectorXf Umpc_st;     // 专门用于传递给 WBC 的 3*nst 维向量
    int n_st = 0;                // 当前支撑腿数量

    float Fmax = 250.0f;
    float fri = 0.4f; // 摩擦系数
    Eigen::Vector4f MPCsFai;

    // 使用全局静态指针管理求解器，杜绝内存反复分配
    qpOASES::QProblem *qp_solver = nullptr;
    int last_n_vars = 0;

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
      A.setZero();
      A.setIdentity(13, 13);
      A.block(3, 9, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T;
      A(11, 12) = 1 * MPC_T;

      Eigen::Matrix3f Ro = QUa2Mat(-0.000543471, 0.713435, -0.00173769, 0.700719);
      Eigen::Vector3f diaginertia(0.107027, 0.0980771, 0.0244531);

      float mm = 6.921f;
      Eigen::Matrix3f I_principal = diaginertia.asDiagonal();
      Eigen::Vector3f P(0.021112, 0, -0.005366);

      BInertia = (Ro * I_principal * (Ro.transpose())) + mm * (P.dot(P) * Eigen::Matrix3f::Identity() - P * P.transpose());

      vec << 25, 25, 10, 1, 1, 100, 0, 0, 0.3, 0.2, 0.2, 20, 0;
      temp = vec.asDiagonal();

      // Q 矩阵尺寸固定，可以在此分配
      Q.resize(13 * h, 13 * h);
      Q.setZero();
      for (int i = 0; i < h; ++i)
      {
        Q.block(i * 13, i * 13, 13, 13) = temp;
      }

      // 注意：R, lb, ub, Aqp, Bqp 等矩阵，由于使用了降维技术，将全部在 UpdateState 中动态分配
    }

    Eigen::Matrix3f getRotationMatrix(double psi, double theta, double phi)
    {
      double c_psi = std::cos(psi), s_psi = std::sin(psi);
      double c_theta = std::cos(theta), s_theta = std::sin(theta);
      double c_phi = std::cos(phi), s_phi = std::sin(phi);

      Eigen::Matrix3f R;
      R(0, 0) = c_theta * c_psi;
      R(0, 1) = c_psi * s_phi * s_theta - c_phi * s_psi;
      R(0, 2) = s_phi * s_psi + c_phi * c_psi * s_theta;
      R(1, 0) = c_theta * s_psi;
      R(1, 1) = c_phi * c_psi + s_phi * s_theta * s_psi;
      R(1, 2) = c_phi * s_theta * s_psi - c_psi * s_phi;
      R(2, 0) = -s_theta;
      R(2, 1) = c_theta * s_phi;
      R(2, 2) = c_phi * c_theta;
      return R;
    }

    void UpdateState()
    {
      MPCsFai = Gait::sFai;

      // 1. 动态过滤出当前的有效支撑腿
      std::vector<int> act_legs;
      for (int i = 0; i < 4; i++)
      {
        if (MPCsFai[i] == 1)
          act_legs.push_back(i);
      }
      n_st = act_legs.size();

      // 防御机制：如果悬空，直接下发 0 力矩
      if (n_st == 0)
      {
        Umpc_st.resize(0);
        Umpc.setZero();
        return;
      }

      // 计算降维后的问题规模
      int n_vars = 3 * n_st * h; // QP优化变量数量 (60 或 120)
      int n_cons = 5 * n_st * h; // QP约束数量 (100 或 200)

      // 2. 动态分配底层矩阵的内存
      Aqp.resize(13 * h, 13);
      Bqp.resize(13 * h, n_vars);
      Bqp.setZero();

      R.resize(n_vars, n_vars);
      R.setIdentity();
      R *= 1e-5f * 5;

      lb.resize(n_vars, 1);
      ub.resize(n_vars, 1);
      Ampc.resize(n_cons, n_vars);
      Ampc.setZero();
      lba.resize(n_cons, 1);
      lba.setZero();
      uba.resize(n_cons, 1);

      // 3. 构建 Aqp 和 Bqp (使用降维拼接)
      for (int i = 0; i < h; ++i)
      {
        if (i == 0)
          Temp_A.setIdentity();
        A.block(0, 6, 3, 3) = Gait::TF_Z(KeyboardIns::desirex[i][2]).transpose() * MPC_T;
        Temp_A = A * Temp_A;
        Aqp.block(13 * i, 0, 13, 13) = Temp_A;
      }

      float m = 15.205f;
      for (int i = 0; i < h; i++)
      {
        Eigen::Matrix3f Ro = getRotationMatrix(KeyboardIns::desirex[i][2], 0, 0);

        //  Eigen::Matrix3f Ro = getRotationMatrix(KeyboardIns::desirex[i][2], KeyboardIns::desirex[i][1], KeyboardIns::desirex[i][0]);
        PInertia = Ro * BInertia * (Ro.transpose());

        // 仅对有效支撑腿计算 B 矩阵映射
        Eigen::MatrixXf B_step = Eigen::MatrixXf::Zero(13, 3 * n_st);
        for (int leg_idx = 0; leg_idx < n_st; leg_idx++)
        {
          int leg = act_legs[leg_idx];
          B_step.block(6, 3 * leg_idx, 3, 3) = (PInertia.inverse()) * KF::skewSymmetric(Gait::Pstend[leg] - KF::X.block(0, 0, 3, 1)) * MPC_T;
          B_step.block(9, 3 * leg_idx, 3, 3) = Eigen::Matrix3f::Identity() * MPC_T / m;
        }

        for (int j = i + 1; j < h + 1; j++)
        {
          if (j == i + 1)
            Temp_A.setIdentity();
          A.block(0, 6, 3, 3) = Gait::TF_Z(KeyboardIns::desirex[j][2]).transpose() * MPC_T;
          Bqp.block((j - 1) * 13, i * 3 * n_st, 13, 3 * n_st) = Temp_A * B_step;
          Temp_A = A * Temp_A;
        }
      }

      // 4. 构建力边界和摩擦锥约束
      Eigen::MatrixXf t(5, 3);
      t << -1, 0, fri, 0, -1, fri, 1, 0, fri, 0, 1, fri, 0, 0, 1;
      Eigen::VectorXf vub(5);
      vub << 1e8, 1e8, 1e8, 1e8, Fmax;
      //  vub << 100000.0f, 100000.0f, 100000.0f, 100000.f, Fmax;

      for (int i = 0; i < h; i++)
      {
        for (int leg_idx = 0; leg_idx < n_st; leg_idx++)
        {
          // 设置 X Y Z 三个维度的上下界
          lb(i * 3 * n_st + 3 * leg_idx + 0) = -Fmax * fri;
          ub(i * 3 * n_st + 3 * leg_idx + 0) = Fmax * fri;
          lb(i * 3 * n_st + 3 * leg_idx + 1) = -Fmax * fri;
          ub(i * 3 * n_st + 3 * leg_idx + 1) = Fmax * fri;
          lb(i * 3 * n_st + 3 * leg_idx + 2) = 0.f;
          ub(i * 3 * n_st + 3 * leg_idx + 2) = Fmax;

          // 填入摩擦锥
          Ampc.block(i * 5 * n_st + 5 * leg_idx, i * 3 * n_st + 3 * leg_idx, 5, 3) = t;
          uba.block(i * 5 * n_st + 5 * leg_idx, 0, 5, 1) = vub;
        }
      }

      // 5. 组装 QP 矩阵
      D = KeyboardIns::D; // 提取当前的 D 矩阵
      H = 2 * (Bqp.transpose() * Q * Bqp + R);
      g = 2 * Bqp.transpose() * Q * (Aqp * KeyboardIns::desirex[0] - D);

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_qp = H.cast<double>();
      Eigen::VectorXd g_qp = g.cast<double>();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_qp = Ampc.cast<double>();
      Eigen::VectorXd lb_qp = lb.cast<double>();
      Eigen::VectorXd ub_qp = ub.cast<double>();
      Eigen::VectorXd lba_qp = lba.cast<double>();
      Eigen::VectorXd uba_qp = uba.cast<double>();

      // 6. 智能管理 qpOASES 求解器生命周期 (仅在变量维度变化时重建求解器)
      if (n_vars != last_n_vars || qp_solver == nullptr)
      {
        if (qp_solver != nullptr)
          delete qp_solver;
        qp_solver = new qpOASES::QProblem(n_vars, n_cons);
        qpOASES::Options option;
        option.setToMPC();
        option.printLevel = qpOASES::PL_NONE;
        qp_solver->setOptions(option);
        last_n_vars = n_vars;
      }

      int nWSR_actual = 100;
      qpOASES::returnValue status = qp_solver->init(
          H_qp.data(), g_qp.data(), A_qp.data(),
          lb_qp.data(), ub_qp.data(), lba_qp.data(), uba_qp.data(),
          nWSR_actual);

      if (status == qpOASES::SUCCESSFUL_RETURN)
      {
        double xOpt[n_vars];
        qp_solver->getPrimalSolution(xOpt);

        // 7. 直接提取完美对齐的支撑腿结果给 WBC
        Umpc_st.resize(3 * n_st);
        for (int i = 0; i < 3 * n_st; i++)
        {
          Umpc_st[i] = xOpt[i];
        }

        // (可选) 填充 12 维的 Umpc 用于日志或外部可视化
        Umpc.setZero();
        for (int leg_idx = 0; leg_idx < n_st; leg_idx++)
        {
          Umpc.block(3 * act_legs[leg_idx], 0, 3, 1) = Umpc_st.block(3 * leg_idx, 0, 3, 1);
        }
      }
      else
      {
        std::cout << "MPC Solve Failed--------" << status << std::endl;
      }
    }

    void MPC_Reset()
    {
      if (qp_solver != nullptr)
      {
        delete qp_solver;
        qp_solver = nullptr;
      }
      last_n_vars = 0;
      n_st = 0;
      Umpc.setZero();
      Umpc_st.resize(0);
    }
  };

  namespace WBC
  {
    bool is_tree_created = false; // 新增：安全锁
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

    void bind(std::shared_ptr<node> &parent, std::shared_ptr<node> &child)
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
      is_tree_created = true; // 新增：树建立完毕，允许计算
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

        TFMatrix << KF::B2W.transpose(), Eigen::Matrix3f::Zero(),
            -KF::B2W.transpose() * KF::skewSymmetric(KF::pcom), KF::B2W.transpose();
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
        // TFMatrix << KF::B2W, Eigen::Matrix3f::Zero(),
        //     KF::B2W * KF::skewSymmetric(KF::pcom), KF::B2W;
        TFMatrix << KF::B2W, Eigen::Matrix3f::Zero(),
            KF::skewSymmetric(KF::pcom) * KF::B2W, KF::B2W;
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
            // KF::B2W * KF::skewSymmetric(KF::pcom),
            KF::skewSymmetric(KF::pcom) * KF::B2W,
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

    float kp = 150.f;
    float kd = 100.f;
    // 空间速度 空间加速度 {0} 到任意坐标系的变换矩阵   Si从 i=1 开始
    vector<Eigen::MatrixXf> Vspace, Aspace, Aspaced, X02I, X02If, Si, fi, Vji, qidot, Jb, XQi, Jbi, AspaceQ, VspaceQ, I, Ic;
    Eigen::MatrixXf XCi(6, 6), C(18, 1), M(18, 18);
    vector<int> pi;

    // 阻尼最小二乘法
    Eigen::MatrixXf WideInverse(const Eigen::MatrixXf &mat)
    {
      if (mat.rows() == 0 || mat.cols() == 0)
        return Eigen::MatrixXf::Zero(mat.cols(), mat.rows());

      float lambda = 1e-4f; // 阻尼系数，保护奇点
      Eigen::MatrixXf I = Eigen::MatrixXf::Identity(mat.rows(), mat.rows());

      // J# = J^T * (J * J^T + lambda^2 * I)^-1
      return mat.transpose() * (mat * mat.transpose() + lambda * lambda * I).inverse();
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
      // 先为 vector 分配空间，总共 14 个刚体节点 (0~13)
      Si.resize(14);
      X02I.resize(14);
      X02If.resize(14);
      Vspace.resize(14);
      Aspace.resize(14);
      Aspaced.resize(14);
      fi.resize(14);
      Vji.resize(14);
      qidot.resize(14);
      I.resize(14);
      Ic.resize(14);

      // 腿部的相关参数，总共 4 条腿
      Jb.resize(4);
      Jbi.resize(4);
      XQi.resize(4);
      AspaceQ.resize(4);
      VspaceQ.resize(4);

      Si[0] = Eigen::MatrixXf::Zero(6, 6); // 没有 Si0 随便初始
      Si[1] = Eigen::MatrixXf::Identity(6, 6);

      for (int i = 2; i < 14; ++i)
      {
        Si[i].setZero(6, 1);
      }
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

      // //
      vector<Eigen::Vector3f> P;
      P.resize(13);
      vector<Eigen::Vector4f> quat;
      quat.resize(13);
      vector<float> mass;
      mass.resize(4);
      vector<Eigen::Vector3f> diagnertia;
      diagnertia.resize(4);

      P[0] << 0.021112f, 0.0f, -0.005366f;

      // 1~3: FR
      P[1] << -0.0054f, -0.00194f, -0.000105f;       // FR_hip
      P[2] << -0.00374f, 0.0223f, -0.0327f;          // FR_thigh
      P[3] << 0.00629595f, 0.000622121f, -0.141417f; // FR_calf

      // 4~6: FL
      P[4] << -0.0054f, 0.00194f, -0.000105f;         // FL_hip
      P[5] << -0.00374f, -0.0223f, -0.0327f;          // FL_thigh
      P[6] << 0.00629595f, -0.000622121f, -0.141417f; // FL_calf

      // 7~9: RR
      P[7] << 0.0054f, -0.00194f, -0.000105f;        // RR_hip
      P[8] << -0.00374f, 0.0223f, -0.0327f;          // RR_thigh
      P[9] << 0.00629595f, 0.000622121f, -0.141417f; // RR_calf

      // 10~12: RL
      P[10] << 0.0054f, 0.00194f, -0.000105f;          // RL_hip
      P[11] << -0.00374f, -0.0223f, -0.0327f;          // RL_thigh
      P[12] << 0.00629595f, -0.000622121f, -0.141417f; // RL_calf

      mass[0] = 6.921;
      diagnertia[0] << 0.107027, 0.0980771, 0.0244531;
      mass[1] = 0.678;
      diagnertia[1] << 0.00088403, 0.000596003, 0.000479967;
      mass[2] = 1.152;
      diagnertia[2] << 0.00594973, 0.00584149, 0.000878787;
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
        //  I[i] = (R * diagnertia[r].asDiagonal() * R.transpose()) + mass[r] * (P[r].transpose() * P[r] * Eigen::Matrix3f::Identity() - P[r] * P[r].transpose());
        //====================================================//====================================================
        Eigen::Matrix3f I_3x3 = (R * diagnertia[r].asDiagonal() * R.transpose()) +
                                mass[r] * (P[i].dot(P[i]) * Eigen::Matrix3f::Identity() - P[i] * P[i].transpose());
        // 计算质心偏移向量的反对称矩阵 c_x
        Eigen::Matrix3f c_cross = KF::skewSymmetric(P[i]);

        // 重设尺寸为 6x6，并依照公式 5.40 将 4 个块拼装成完整的六维空间惯量
        I[i].resize(6, 6);
        I[i] << I_3x3, mass[r] * c_cross,
            mass[r] * c_cross.transpose(), mass[r] * Eigen::Matrix3f::Identity();
        //====================================================//====================================================//====================================================
      }
    }

    void WBC_Reset()
    {
      q.setZero();
      qdot.setZero();
      qddot.setZero();
      qcmd.setZero();
      qdotcmd.setZero();
      qddotcmd.setZero();
      qdotcmde.setZero();
      qddotcmde.setZero();
      detqcmd.setZero();
      M.setZero();
      C.setZero();
      J1.resize(0, 18);
      J4.resize(0, 18);
      J1q.resize(0, 1);
      J4q.resize(0, 1);
    }
    // multi-Rigid-Body dynamics algorithm
    void Dynamcis_Update()
    {
      M.setZero();
      C.setZero();

      int n_st = 0;
      for (int i = 0; i < 4; i++)
      {
        if (Gait::sFai[i] == 1.0f)
          n_st++;
      }
      int n_sw = 4 - n_st;

      n_sw = 4 - n_st;
      J1.resize(3 * n_st, 18);
      J1.setZero();
      J1q.resize(3 * n_st, 1);
      J1q.setZero();

      J4.resize(3 * n_sw, 18);
      J4.setZero();
      J4q.resize(3 * n_sw, 1);
      J4q.setZero();
      int J1num = 0;
      int J4num = 0;
      // Eigen 使用 << 逗号初始化前，必须提前分配好尺寸！
      qidot[0].resize(6, 1);
      qidot[0] << KF::Wb, KF::B2W.transpose() * KF::vcom;
      for (int i = 0; i < 12; ++i)
      {
        qidot[i + 1].resize(1, 1);
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
          // fi[Vnode[i]->parent.lock()->num - 1] = fi[Vnode[i]->parent.lock()->num - 1] + Transform_C2P(Vnode[i]) * fi[i - 1];
          fi[Vnode[i]->parent.lock()->num - 1] = fi[Vnode[i]->parent.lock()->num - 1] + Transform_C2PF(Vnode[i]) * fi[i - 1];
        }

        Ic[i - 1] = I[i - 1];
        for (auto node : Vnode[i]->child)
        { // 这里 i 的child 是j 们， 所以i->j  和j->i 是P2C 或者 C2P 的关系 ，直接用

          Ic[i - 1] = Ic[i - 1] + Transform_C2PF(Vnode[node->num]) * Ic[node->num - 1] * Transform_P2C(Vnode[node->num]);
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
        Eigen::MatrixXf Xt = Eigen::MatrixXf::Identity(6, 6);
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
        //
        XQi[i].resize(6, 6);
        XQi[i] << X02I[pi[i]].block(3, 3, 3, 3).transpose(), Eigen::Matrix3f::Zero(),
            Eigen::Matrix3f::Zero(), X02I[pi[i]].block(3, 3, 3, 3).transpose();
        Eigen::MatrixXf Xjpi = Eigen::MatrixXf::Identity(6, 6);
        // Jb 使用 .block 赋值前，必须分配 6x18 的尺寸并清零！
        Jb[i].resize(6, 18);
        Jb[i].setZero();
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
        //  转向定向足底坐标系
        Jbi[i] = (XQi[i] * XCi * Jb[i]).block(3, 0, 3, 18);
        AspaceQ[i] = (XQi[i] * XCi) * Aspace[pi[i]];
        VspaceQ[i] = (XQi[i] * XCi) * Vspace[pi[i]];

        //======================
        if (Gait::sFai[i] == 1.0f) // 支撑腿
        {
          J1.block(3 * J1num, 0, 3, 18) = Jbi[i];
          // 显式转换为 Vector3f
          J1q.block(3 * J1num, 0, 3, 1) = AspaceQ[i].block(3, 0, 3, 1) + Eigen::Vector3f(VspaceQ[i].block(0, 0, 3, 1)).cross(Eigen::Vector3f(VspaceQ[i].block(3, 0, 3, 1)));
          J1num++;
        }
        else // 摆动腿
        {
          J4.block(3 * J4num, 0, 3, 18) = Jbi[i];
          // 显式转换为 Vector3f
          J4q.block(3 * J4num, 0, 3, 1) = AspaceQ[i].block(3, 0, 3, 1) + Eigen::Vector3f(VspaceQ[i].block(0, 0, 3, 1)).cross(Eigen::Vector3f(VspaceQ[i].block(3, 0, 3, 1)));
          J4num++;
        }

        //===================
      }
    }

    void WBC_Update(const mjModel *model, mjData *data)
    {
      J2.setZero(3, 18);
      J3.setZero(3, 18);
      J2.block<3, 3>(0, 0) = KF::B2W;
      J3.block<3, 3>(0, 3) = KF::B2W;

      J2q.setZero();
      Eigen::Vector3f v_body = KF::B2W.transpose() * KF::vcom;
      J3q = KF::B2W * (KF::Wb.cross(KF::B2W.transpose() * KF::vcom));
      //  std::cout << "WBC_Update" << std::endl;
      //===================================
      // 0. 准备当前状态 q 和 qdot
      q.block(0, 0, 3, 1) << KF::Faix, KF::Faiy, KF::Faiz;
      q.block(3, 0, 3, 1) = KF::pcom;
      q.block(6, 0, 12, 1) = KF::jointpos;

      qdot.block(0, 0, 3, 1) = KF::Wb;
      qdot.block(3, 0, 3, 1) = KF::B2W.transpose() * KF::vcom;
      qdot.block(6, 0, 12, 1) = KF::jointvel;

      // 初始化层级矩阵与零空间
      Eigen::MatrixXf J_prev = J1;
      Eigen::MatrixXf NA = Eigen::MatrixXf::Identity(18, 18) - WideInverse(J_prev) * J_prev;

      detqcmd.setZero();
      qdotcmde.setZero();
      qddotcmde = WideInverse(J1) * (-J1q);
      // ---- 准备错误记录数据容器 ----
      Eigen::Vector3f err_ori, err_angvel, err_pos, err_vel;
      // 由于摆动腿数量会变，足端误差我们先初始化为零
      Eigen::Vector3f err_footpos = Eigen::Vector3f::Zero();
      Eigen::Vector3f err_footvel = Eigen::Vector3f::Zero();
      // ---- 任务 2：机身姿态转动控制 ----
      tran2 << cos(KeyboardIns::desirex[0][1]) * cos(KeyboardIns::desirex[0][2]), -sin(KeyboardIns::desirex[0][2]), 0,
          cos(KeyboardIns::desirex[0][1]) * sin(KeyboardIns::desirex[0][2]), cos(KeyboardIns::desirex[0][2]), 0,
          -sin(KeyboardIns::desirex[0][1]), 0, 1;
      Eigen::Vector3f dfai = {KeyboardIns::desirex[1][0], KeyboardIns::desirex[1][1], KeyboardIns::desirex[1][2]};
      Eigen::Vector3f fai = {KeyboardIns::desirex[0][0], KeyboardIns::desirex[0][1], KeyboardIns::desirex[0][2]};
      Eigen::Vector3f dwO = {KeyboardIns::desirex[1][6], KeyboardIns::desirex[1][7], KeyboardIns::desirex[1][8]};

      float kp_fai = 70;
      float kd_w = 50;
      e = tran2 * (dfai - fai);
      x = kd_w * (dwO - KF::B2W * KF::Wb) + kp_fai * (e);
      //   x = kd_w * (dwO - KF::Wb) + kp_fai * (e);

      err_ori = dfai - fai;                // 1. 姿态角误差
      err_angvel = dwO - KF::B2W * KF::Wb; // 2. 角速度误差

      detqcmd += WideInverse(J2 * NA) * (e - J2 * detqcmd);
      qdotcmde += WideInverse(J2 * NA) * (dwO - J2 * qdotcmde);
      qddotcmde += WideInverse(J2 * NA) * (x - J2q - J2 * qddotcmde);

      // 叠加任务 1 和任务 2 的雅可比求新零空间
      Eigen::MatrixXf JA2(J_prev.rows() + J2.rows(), 18);
      JA2 << J_prev, J2;
      J_prev = JA2;
      NA = Eigen::MatrixXf::Identity(18, 18) - WideInverse(J_prev) * J_prev;

      // ---- 任务 3：机身平动控制 ----
      Eigen::Vector3f dPo = {KeyboardIns::desirex[1][3], KeyboardIns::desirex[1][4], KeyboardIns::desirex[1][5]};
      Eigen::Vector3f dVO = {KeyboardIns::desirex[1][9], KeyboardIns::desirex[1][10], KeyboardIns::desirex[1][11]};
      float kp_pos = 50;
      float kd_vel = 25;
      e = dPo - KF::pcom;
      x = kd_vel * (dVO - KF::vcom) + kp_pos * (dPo - KF::pcom);
      err_pos = dPo - KF::pcom; // 3. 躯干位置误差
      err_vel = dVO - KF::vcom; // 4. 躯干速度误差

      detqcmd += WideInverse(J3 * NA) * (e - J3 * detqcmd);
      qdotcmde += WideInverse(J3 * NA) * (dVO - J3 * qdotcmde);
      qddotcmde += WideInverse(J3 * NA) * (x - J3q - J3 * qddotcmde);

      // 叠加任务 1、2、3 的雅可比求新零空间
      Eigen::MatrixXf JA3(J_prev.rows() + J3.rows(), 18);
      JA3 << J_prev, J3;
      J_prev = JA3;
      NA = Eigen::MatrixXf::Identity(18, 18) - WideInverse(J_prev) * J_prev;

      // ---- 任务 4：摆动腿足端轨迹控制 (最低优先级) ----
      Eigen::VectorXf ee(3 * J4.rows() / 3), Pfoot(3 * J4.rows() / 3), dPfoot(3 * J4.rows() / 3);
      Eigen::VectorXf Vfoot(3 * J4.rows() / 3), dVfoot(3 * J4.rows() / 3);
      int num = 0;
      for (int j = 0; j < 4; ++j)
      {
        if (Gait::sFai[j] == 0)
        { // 找到摆动腿
          Pfoot.block(3 * num, 0, 3, 1) = KF::pcom + KF::B2W * KF::iPb[j];
          Vfoot.block(3 * num, 0, 3, 1) = (XQi[j] * XCi * Vspace[pi[j]]).block(3, 0, 3, 1);
          dPfoot.block(3 * num, 0, 3, 1) = Gait::FootdesirePos[j];
          dVfoot.block(3 * num, 0, 3, 1) = Gait::FootdesireVelocity[j];
          // 仅记录第一只找到的摆动腿作为可视化参考
          if (num == 0)
          {
            err_footpos = Gait::FootdesirePos[j] - (KF::pcom + KF::B2W * KF::iPb[j]); // 5. 足端位置误差
            err_footvel = Gait::FootdesireVelocity[j] - Vfoot.block(0, 0, 3, 1);      // 6. 足端速度误差
          }
          num++;
        }
      }

      if (num > 0)
      {
        ee = dPfoot - Pfoot;
        float kd_swing = 30;
        float kp_swing = 50;
        x = kd_swing * (dVfoot - Vfoot) + kp_swing * (ee);

        detqcmd += WideInverse(J4 * NA) * (ee - J4 * detqcmd);
        qdotcmd = qdotcmde + WideInverse(J4 * NA) * (dVfoot - J4 * qdotcmde);
        qddotcmd = qddotcmde + WideInverse(J4 * NA) * (x - J4q - J4 * qddotcmde);
      }
      else
      { // 如果没有摆动腿 (比如4足站立阶段)
        qdotcmd = qdotcmde;
        qddotcmd = qddotcmde;
      }

      qcmd = q + detqcmd;
      // ================= 数据可视化映射 (新增) =================
      auto logWBCErrors = [&]()
      {
        std::stringstream ss;
        ss << "{";
        // 1. 姿态角误差 (Roll, Pitch, Yaw)
        ss << "\"err_ori_r\":" << err_ori[0] << ",\"err_ori_p\":" << err_ori[1] << ",\"err_ori_y\":" << err_ori[2] << ",";
        // 2. 角速度误差
        ss << "\"err_angvel_x\":" << err_angvel[0] << ",\"err_angvel_y\":" << err_angvel[1] << ",\"err_angvel_z\":" << err_angvel[2] << ",";
        // 3. 躯干位置误差 (X, Y, Z)
        ss << "\"err_pos_x\":" << err_pos[0] << ",\"err_pos_y\":" << err_pos[1] << ",\"err_pos_z\":" << err_pos[2] << ",";
        // 4. 躯干速度误差
        ss << "\"err_vel_x\":" << err_vel[0] << ",\"err_vel_y\":" << err_vel[1] << ",\"err_vel_z\":" << err_vel[2] << ",";
        // 5. 摆动足位置误差 (以代表腿为例)
        ss << "\"err_footpos_x\":" << err_footpos[0] << ",\"err_footpos_y\":" << err_footpos[1] << ",\"err_footpos_z\":" << err_footpos[2] << ",";
        // 6. 摆动足速度误差
        ss << "\"err_footvel_x\":" << err_footvel[0] << ",\"err_footvel_y\":" << err_footvel[1] << ",\"err_footvel_z\":" << err_footvel[2];
        ss << "}";
        visualizer.sendData(ss.str());
      };

      logWBCErrors(); // 执行发送

      int n_st = 0;
      for (int i = 0; i < 4; i++)
      {
        if (Gait::sFai[i] == 1.0f)
          n_st++;
      }

      // 必须有支撑腿才进行优化
      if (n_st > 0)
      {
        int n_var = 18 + 3 * n_st; // 优化变量 X =[delta_q(18); delta_f(3*n_st)] (式 4.45)

        quadprogpp::Matrix<double> G_qp, CE_qp, CI_qp;
        quadprogpp::Vector<double> g_qp, ce0_qp, ci0_qp, x_qp;

        G_qp.resize(n_var, n_var);
        g_qp.resize(n_var);
        for (int i = 0; i < n_var; i++)
        {
          g_qp[i] = 0.0; // g 向量为 0 (因为目标函数无一次项)
          for (int j = 0; j < n_var; j++)
            G_qp[i][j] = 0.0;
        }

        // 1. 构建 G 矩阵 (式 4.46)
        // Q1 = I (18x18), Q2 = 0.005 * I (3n_st x 3n_st)
        for (int i = 0; i < 18; i++)
          G_qp[i][i] = 1.0;
        for (int i = 0; i < 3 * n_st; i++)
          G_qp[18 + i][18 + i] = 0.005;

        // 2. 构建等式约束 CE^T * X + ce0 = 0 (式 4.49, 4.50)
        int n_eq = 6;
        CE_qp.resize(n_var, n_eq);
        ce0_qp.resize(n_eq);

        Eigen::MatrixXf Mf = M.block(0, 0, 6, 18);                       // 提取 M 矩阵前 6 行
        Eigen::MatrixXf Cf = C.block(0, 0, 6, 1);                        // 提取 C 向量前 6 行
        Eigen::MatrixXf Jcf_T = J1.block(0, 0, 3 * n_st, 6).transpose(); // 提取 J1 对应机身的前 6 列的转置

        Eigen::MatrixXf CE_eigen(6, n_var);
        CE_eigen.block(0, 0, 6, 18) = Mf;
        CE_eigen.block(0, 18, 6, 3 * n_st) = -Jcf_T;
        Eigen::VectorXf current_Umpc_st(3 * n_st);
        current_Umpc_st.setZero();

        int st_idx = 0;
        for (int i = 0; i < 4; i++)
        {
          if (Gait::sFai[i] == 1.0f)
          { // 只挑出当前真正在地上的腿
            // 从 12 维的 Umpc 里，摘出这根腿对应的力
            current_Umpc_st.block(3 * st_idx, 0, 3, 1) = ConvexMPC::Umpc.block(3 * i, 0, 3, 1);
            st_idx++;
          }
        }
        Eigen::VectorXf ce0_eigen = -Jcf_T * current_Umpc_st + Cf + Mf * qddotcmd;
        //  Eigen::VectorXf ce0_eigen = -Jcf_T * ConvexMPC::Umpc_st + Cf + Mf * qddotcmd;

        // quadprogpp 期望的等式约束矩阵维度为 (n_var x n_eq)，所以赋值时需要转置
        for (int i = 0; i < n_var; i++)
        {
          for (int j = 0; j < n_eq; j++)
          {
            CE_qp[i][j] = CE_eigen(j, i);
          }
        }
        for (int j = 0; j < n_eq; j++)
          ce0_qp[j] = ce0_eigen(j);

        // 3. 构建不等式约束 CI^T * X + ci0 >= 0 (式 4.56, 4.57)
        int n_ineq = 10 * n_st; // 每条支撑腿对应 10 个不等式约束
        CI_qp.resize(n_var, n_ineq);
        ci0_qp.resize(n_ineq);
        for (int i = 0; i < n_var; i++)
          for (int j = 0; j < n_ineq; j++)
            CI_qp[i][j] = 0.0;

        float mu = ConvexMPC::fri; // 请确保 ConvexMPC::fri 已经被赋了初值 (如 0.5)
        Eigen::MatrixXf CA_leg(5, 3);
        CA_leg << -1, 0, mu,
            0, -1, mu,
            1, 0, mu,
            0, 1, mu,
            0, 0, 1;

        Eigen::VectorXf c_bar_leg(5), c_under_leg(5);
        c_bar_leg << 10000, 10000, 10000, 10000, ConvexMPC::Fmax; // 摩擦锥上限设为很大的值
        c_under_leg << 0, 0, 0, 0, 0;

        Eigen::MatrixXf CA = Eigen::MatrixXf::Zero(5 * n_st, 3 * n_st);
        Eigen::VectorXf c_bar(5 * n_st), c_under(5 * n_st);

        for (int i = 0; i < n_st; i++)
        {
          CA.block(5 * i, 3 * i, 5, 3) = CA_leg;
          c_bar.segment(5 * i, 5) = c_bar_leg;
          c_under.segment(5 * i, 5) = c_under_leg;
        }

        Eigen::MatrixXf CI_T_eigen = Eigen::MatrixXf::Zero(10 * n_st, n_var);
        CI_T_eigen.block(0, 18, 5 * n_st, 3 * n_st) = -CA;
        CI_T_eigen.block(5 * n_st, 18, 5 * n_st, 3 * n_st) = CA;

        Eigen::VectorXf ci0_eigen(10 * n_st);
        ci0_eigen.segment(0, 5 * n_st) = c_bar - CA * current_Umpc_st;
        ci0_eigen.segment(5 * n_st, 5 * n_st) = CA * current_Umpc_st - c_under;

        // quadprogpp 期望矩阵维度为 (n_var x n_ineq)
        for (int i = 0; i < n_var; i++)
        {
          for (int j = 0; j < n_ineq; j++)
          {
            CI_qp[i][j] = CI_T_eigen(j, i);
          }
        }
        for (int j = 0; j < n_ineq; j++)
          ci0_qp[j] = ci0_eigen(j);

        // 4. 调用 quadprogpp 进行求解
        double cost = quadprogpp::solve_quadprog(G_qp, g_qp, CE_qp, ce0_qp, CI_qp, ci0_qp, x_qp);

        // 5. 提取松弛变量结果，计算最终电机的下发扭矩 (式 4.61)
        Eigen::VectorXf delta_q(18), delta_f(3 * n_st);
        for (int i = 0; i < 18; i++)
          delta_q(i) = x_qp[i];
        for (int i = 0; i < 3 * n_st; i++)
          delta_f(i) = x_qp[18 + i];

        Eigen::VectorXf qddot_final = qddotcmd + delta_q;    // 最终广义加速度
        Eigen::VectorXf f_final = current_Umpc_st + delta_f; // 最终足底力

        Eigen::MatrixXf Mj = M.block(6, 0, 12, 18);
        Eigen::MatrixXf Cj = C.block(6, 0, 12, 1);
        Eigen::MatrixXf Jcj_T = J1.block(0, 6, 3 * n_st, 12).transpose(); // 仅取关节部分 (后12列) 的雅可比转置

        // 最终计算出的关节力矩 (可直接结合 PD 控制器发送给电机)
        Eigen::VectorXf tau = Mj * qddot_final + Cj - Jcj_T * f_final;

        // 此处你可以将 tau 与 PD 的计算结果相加

        PDcontrol::PDcontrol(model, data, qcmd.bottomRows(12), qdotcmd.bottomRows(12), tau);
      }
    }

  };
  namespace PDcontrol
  {
    float kp = 28.f;
    float kd = 2.f;
    std::vector<float> home = {0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8};

    // 根据 Go2 XML 提取的真实机械关节限位 (单位: rad)
    //  顺序: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11)
    const float q_min[12] = {
        -1.0472f, -1.5708f, -2.7227f, // FR (侧摆, 前大腿, 小腿)
        -1.0472f, -1.5708f, -2.7227f, // FL
        -1.0472f, -0.5236f, -2.7227f, // RR (侧摆, 后大腿, 小腿)
        -1.0472f, -0.5236f, -2.7227f  // RL
    };

    const float q_max[12] = {
        1.0472f, 3.4907f, 0.0f, // FR (小腿上限设为0，防止膝盖反折)
        1.0472f, 3.4907f, 0.0f, // FL
        1.0472f, 4.5379f, 0.0f, // RR
        1.0472f, 4.5379f, 0.0f  // RL
    };

    inline float clamp(float val, float min_val, float max_val)
    {
      return std::max(min_val, std::min(val, max_val));
    }

    void PDcontrol(const mjModel *model, mjData *data,
                   const Eigen::VectorXf &q_des,
                   const Eigen::VectorXf &qdot_des,
                   const Eigen::VectorXf &tau_ff)
    {
      for (int i = 0; i < 12; i++)
      {
        // 对期望关节角进行限位截断
        float safe_q_des = clamp(q_des[i], q_min[i], q_max[i]);

        float err_q = safe_q_des - KF::jointpos[i];
        float err_dq = qdot_des[i] - KF::jointvel[i];

        float tau_cmd = tau_ff[i] + kp * err_q + kd * err_dq;

        float torque_limit = 23.7f;
        if (i % 3 == 2)
          torque_limit = 35.55f; // 小腿 (Knee) 电机的扭矩更大

        if (tau_cmd > torque_limit)
          tau_cmd = torque_limit;
        if (tau_cmd < -torque_limit)
          tau_cmd = -torque_limit;

        data->ctrl[i] = tau_cmd;
      }
    }
  };
  namespace SystemControl
  {

    // 系统总初始化
    void System_Init(mjModel *model, mjData *data, float dt)
    {

      //  初始化卡尔曼滤波与状态估计
      KF::kf_Init(dt);
      KF::joint_sensor_data_update(model, data);
      KF::B2WUpdate(model, data, "imu_quat"); // 请确保 xml 中 IMU 传感器名称一致
      KF::Kfsolver();
      KeyboardIns::Keyboard_init();
      FSM::Init_FSM();

      Gait::Gait_Init();

      ConvexMPC::MPC_init();

      WBC::CreatTree();
      WBC::WBC_Init();

      std::cout << "Quadruped  Initialization Completed!" << std::endl;
    }
    // 一键复位函数
    void Reset_Robot(const mjModel *model, mjData *data, float dt)
    {
      std::cout << "\n[WARNING] XBOX 'A' PRESSED! RESETTING ROBOT STATE...\n"
                << std::endl;

      // 1. 物理引擎状态重置 (回到 keyframe 0)
      mj_resetDataKeyframe(model, data, 0);
      mj_forward(model, data); // 正向传播
      for (int i = 0; i < 12; i++)
      {
        data->ctrl[i] = 0.0f;
      }

      // 2. 状态估计器重置 (清空卡尔曼积分误差)
      KF::kf_Init(dt);
      KF::joint_sensor_data_update(model, data);
      KF::B2WUpdate(model, data, "imu_quat");
      KF::FootUpdate();

      // 3. 指令与状态机重置
      FSM::Init_FSM();
      KeyboardIns::Keyboard_init();
      // 4. 步态与轨迹重置
      Gait::Gait_Init();
      Gait::SymPb.clear();
      Gait::ChangeGait(0);
      Gait::join = false; // 让 SymPb 重新初始化

      // 5. MPC 内存释放
      ConvexMPC::MPC_Reset();
      ConvexMPC::MPC_init();
      // 6. 控制循环时钟重置
      first_mpc = false;
      WBC::WBC_Reset();
      std::cout << "[SUCCESS] ROBOT RESET COMPLETED!\n"
                << std::endl;
    }
    bool first_mpc = false;

    // 主控制循环 (通常在 MuJoCo 的控制回调函数中以 500Hz 运行)
    void Control_Step(const mjModel *model, mjData *data, float dt)
    {

      static int last_btn_a = 0;
      if (map.a == 1 && last_btn_a == 0)
      {
        Reset_Robot(model, data, dt);
        last_btn_a = 1;
        return;
      }
      last_btn_a = map.a;
      static auto enter = std::chrono::high_resolution_clock::now();
      static int times = 0;
      auto now = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - enter).count() >= 1000)
      {
        std::cout << "T--======================================->" << times << std::endl;
        times = 0;
        enter = now;
      }

      KF::joint_sensor_data_update(model, data);
      KF::B2WUpdate(model, data, "imu_quat");

      KF::FootUpdate();

      KF::Kfsolver();

      Gait::Pstend_Update();
      // // 2. 指令更新
      KeyboardIns::Desire_ins_update(MPC_T); // MPC_T 预测步长为 0.01s
      // // 3. 步态与轨迹规划 (怎么走)
      Gait::TimeUpdate();
      Gait::UpdateGait();
      Gait::FootTraj_Planning();

      static int cout = 0;
      cout++;
      if (cout % 10 == 0)
      {
        Quad::ConvexMPC::UpdateState();
        if (!first_mpc)
          first_mpc = true;
        cout = 0;
      }
      times++;
      if (first_mpc)
      {
        WBC::Dynamcis_Update();
        WBC::WBC_Update(model, data);
      }

      auto printorques = [&]()
      {
        std::stringstream ss;
        ss << "{";

        // MuJoCo 中，控制量的个数存储在 model->nu 中
        for (int i = 0; i < model->nu; ++i)
        {
          ss << "\"tor" << i << "\":" << data->ctrl[i];

          // 如果不是最后一个元素，加上逗号分隔
          if (i < model->nu - 1)
          {
            ss << ",";
          }
        }

        ss << "}";

        visualizer.sendData(ss.str());
      };

      printorques(); // 调用 Lambda
    }
  }

};
