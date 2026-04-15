cmake_minimum_required(VERSION 3.5)
project(tm5_900)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes urdf worlds protos)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()  

install(PROGRAMS
  scripts/mimic_joint_node.py
  RENAME mimic_joint_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()