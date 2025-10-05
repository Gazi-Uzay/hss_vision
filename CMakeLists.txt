cmake_minimum_required(VERSION 3.8)
project(hss_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(hss_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Python paketini kur
ament_python_install_package(hss_vision)

# --- ÖNEMLİ: config ve launch klasörlerini share/ altına kopyala ---
# config/ içinde camera_profiles.yaml vb. olmalı
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# launch/ içinde camera_driver.launch.py veya system.launch.py vb. olmalı
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
