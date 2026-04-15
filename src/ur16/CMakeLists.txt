cmake_minimum_required(VERSION 3.5)
project(ur16)

# ROS 2 Temel Paketlerini Bul
find_package(ament_cmake REQUIRED)

# Dosyaların (URDF, Mesh, Launch) kurulacağı yerleri belirtiyoruz
install(DIRECTORY
  config
  launch
  meshes  # <--- BU SATIR ÇOK ÖNEMLİ! EKSİKSE DOSYALAR KOPYALANMAZ.
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()