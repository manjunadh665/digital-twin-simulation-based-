cmake_minimum_required(VERSION 3.8)
project(digital_twin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz world config scripts pythonfiles
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
    scripts/record_bag.sh
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(pythonfiles)

install(PROGRAMS
    pythonfiles/anomaly_detection.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME anomaly_detector
)

ament_package()
