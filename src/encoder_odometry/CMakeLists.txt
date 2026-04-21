cmake_minimum_required(VERSION 3.5)
project(encoder_odometry)

# =========================
# 🔹 Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# =========================
# 🔹 Build Node
# =========================
add_executable(encoder_odom_node src/encoder_odom_node.cpp)

ament_target_dependencies(encoder_odom_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
)

# =========================
# 🔹 Install Executable
# =========================
install(TARGETS
  encoder_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# 🔹 Install Folders (🔥 المهم هنا)
# =========================
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# 🔹 Final
# =========================
ament_package()
