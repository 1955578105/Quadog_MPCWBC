#include "Self_mujoco_lib.h"

/*
  mujoco自定义函数库
*/
namespace mujo
{
  // 根据 传感器的名 返回 传感器数据
  std::vector<float> get_sensor_data(const mjModel *model, const mjData *data,
                                     const std::string &sensor_name)
  {
    // 1. 先获取 id
    int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
    if (sensor_id == -1)
    {
      std::cout << "no found sensor" << std::endl;
      return std::vector<float>();
    }
    // 2.根据id 获取 起始地址
    int data_pos = model->sensor_adr[sensor_id];
    // 3. 根据id  获取数据大小（维度）
    std::vector<float> sensor_data(model->sensor_dim[sensor_id]);
    for (int i = 0; i < sensor_data.size(); i++)
    {
      sensor_data[i] = data->sensordata[data_pos + i];
    }
    return sensor_data;
  }

};
