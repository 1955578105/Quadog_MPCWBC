cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)

#编译安装，从cmake安装位置opt使用
# set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
# find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
# if (mujoco_FOUND)
#     message(STATUS "Find mujoco:  ${mujoco_INCLUDE_DIRS}, ${mujoco_LIBRARIES}")
# endif()



#release版本
set(MUJOCO_PATH "/home/mujoco/mujoco-3.3.0-linux-x86_64/mujoco-3.3.0")
include_directories(${MUJOCO_PATH}/include)
include_directories(${MUJOCO_PATH}/simulate)
include_directories(/home/eigen/eigen3/Eigen)
include_directories(../)
link_directories(${MUJOCO_PATH}/bin)
set(MUJOCO_LIB ${MUJOCO_PATH}/lib/libmujoco.so)


add_executable(basic main.cc simulate.cc glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc Quad.cpp Self_mujoco_lib.cpp)

#后两个使用
target_link_libraries(basic ${MUJOCO_LIB} glut GL GLU glfw lodepng)


#simulate编译
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
# file(GLOB SIMULATE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc)
# add_executable(simulate ${SIMULATE_SRC})
# target_link_libraries(simulate mujoco::mujoco glut GL GLU glfw lodepn