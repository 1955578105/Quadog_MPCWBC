#ifndef __MUJO_H
#define __MUJO_H

#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

namespace mujo
{

  std::vector<float> get_sensor_data(const mjModel *model, const mjData *data,
                                     const std::string &sensor_name);
};

#endif