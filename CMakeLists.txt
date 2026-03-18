cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)



set(CMAKE_BUILD_TYPE Release)
# CMakeLists.txt 终极编译选项

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -ffast-math")

#release版本
set(MUJOCO_PATH "/home/mujoco/mujoco-3.3.0-linux-x86_64/mujoco-3.3.0")
set(qpOASES_PATH "/home/qpOASES2/qpOASES")
set(QuadProg_PATH "/home/QuadProgpp")

include_directories(${qpOASES_PATH}/include)
include_directories(${QuadProg_PATH}/src)

include_directories(${MUJOCO_PATH}/include)
include_directories(${MUJOCO_PATH}/simulate)
include_directories(/home/eigen/eigen3/Eigen)
include_directories(../)
link_directories(${MUJOCO_PATH}/bin)
link_directories(${qpOASES_PATH}/build/libs)
link_directories(${QuadProg_PATH}/build/src)

set(MUJOCO_LIB ${MUJOCO_PATH}/lib/libmujoco.so)


add_executable(basic main.cc simulate.cc glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc Quad.cpp Self_mujoco_lib.cpp xbox.cc)

#后两个使用
target_link_libraries(basic ${MUJOCO_LIB} glut GL GLU glfw lodepng libqpOASES.a libquadprog.a)


#simulate编译
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
# file(GLOB SIMULATE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc)
# add_executable(simulate ${SIMULATE_SRC})
# target_link_libraries(simulate mujoco::mujoco glut GL GLU glfw lodepn