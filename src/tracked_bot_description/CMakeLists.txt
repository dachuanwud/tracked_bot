cmake_minimum_required(VERSION 3.8)
project(tracked_bot_description)

find_package(ament_cmake REQUIRED)

# === 把资源目录复制到 install/share/${PROJECT_NAME}/ 下 ===
install(
  DIRECTORY launch urdf rviz meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

