cmake_minimum_required(VERSION 3.8)
project(q1_description)

find_package(ament_cmake REQUIRED)

# 纯描述包：只安装 URDF、网格、launch 与 rviz 配置，不编译任何代码。
foreach(dir meshes urdf launch rviz config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()