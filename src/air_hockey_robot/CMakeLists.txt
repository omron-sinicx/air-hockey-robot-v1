cmake_minimum_required(VERSION 3.8)
project(air_hockey_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  message("Clang: yes")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs)
find_package(rclcpp_action)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake")
# find_package(Dynamixel)

set(msg_files
  "msg/VisualPoint.msg"
  "msg/VisualPoints.msg"
  "msg/ObjectsState.msg"
  "msg/ObjectState.msg"
  "msg/Trajectory.msg"
  "msg/ObjectPrediction.msg"
  "msg/MotorCommands.msg"
  "msg/Roi.msg"
  "msg/PuckVector.msg"
  "msg/GameStatus.msg"
  "msg/ArmMotionPlan.msg"
  "msg/ArmStatus.msg"
  "msg/Dynamixel.msg"
  "msg/ObjectMotion.msg"
  "msg/ObjectMotionPrediction.msg"
  "msg/TargetPoint.msg"
)

set(srv_files
  "srv/ArmMotionPlan.srv"
)

set(action_files
  "action/MotorControll.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# ament_export_dependencies(rosidl_default_runtime)

# add_executable(object_motion_predictor src/object_motion_predictor.cpp src/object_motion_predictor_node.cpp)
# ament_target_dependencies(object_motion_predictor rclcpp)

# add_executable(test_node src/test.cpp src/test_node.cpp)
# ament_target_dependencies(test_node rclcpp sensor_msgs)
# ament_target_dependencies(test_node rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(camera_driver src/camera_driver.cpp src/camera_driver_node.cpp)
ament_target_dependencies(camera_driver rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(object_detector src/object_detector.cpp src/object_detector_node.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(velocity_estimator src/velocity_estimator.cpp src/velocity_estimator_node.cpp)
ament_target_dependencies(velocity_estimator rclcpp)

add_executable(object_predictor src/object_predictor.cpp src/object_predictor_node.cpp)
ament_target_dependencies(object_predictor rclcpp)

# add_executable(communicate_motors src/communicate_motors.cpp src/communicate_motors_node.cpp)
# ament_target_dependencies(communicate_motors rclcpp rclcpp_action)

# add_executable(display_image src/display_image.cpp src/display_image_main.cpp)
# ament_target_dependencies(display_image rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(frame_analyzer src/frame_analyzer.cpp src/frame_analyzer_main.cpp)
ament_target_dependencies(frame_analyzer rclcpp sensor_msgs cv_bridge OpenCV)

# # add_executable(mallet_detector src/frame_analyzer.cpp src/mallet_detector_node.cpp)
# # ament_target_dependencies(mallet_detector rclcpp sensor_msgs cv_bridge OpenCV)
# add_executable(mallet_detector src/human_mallet_detector.cpp src/human_mallet_detector_node.cpp)
# ament_target_dependencies(mallet_detector rclcpp sensor_msgs cv_bridge OpenCV)

# # add_executable(robo_arm_detector src/frame_analyzer.cpp src/robo_arm_detector_node.cpp)
# # ament_target_dependencies(robo_arm_detector rclcpp sensor_msgs cv_bridge OpenCV)
# add_executable(robo_arm_detector src/robot_mallet_detector.cpp src/robot_mallet_detector_node.cpp)
# ament_target_dependencies(robo_arm_detector rclcpp sensor_msgs cv_bridge OpenCV)

# add_executable(puck_movement_analyzer src/puck_movement_analyzer.cpp src/puck_movement_analyzer_main.cpp)
# ament_target_dependencies(puck_movement_analyzer rclcpp)

# add_executable(mallet_motion_analyzer src/mallet_motion_analyzer.cpp src/mallet_motion_analyzer_node.cpp)
# ament_target_dependencies(mallet_motion_analyzer rclcpp)

# add_executable(robo_arm_motion_analyzer src/puck_movement_analyzer.cpp src/robo_arm_motion_analyzer_node.cpp)
# ament_target_dependencies(robo_arm_motion_analyzer rclcpp)

# add_executable(game_controller src/game_controller.cpp src/game_controller_main.cpp)
# ament_target_dependencies(game_controller rclcpp rclcpp_action)

# add_executable(motor_controller src/motor_controller.cpp src/motor_controller_main.cpp)
# ament_target_dependencies(motor_controller rclcpp rclcpp_action)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(camera_driver "${cpp_typesupport_target}")
target_link_libraries(object_detector "${cpp_typesupport_target}")
target_link_libraries(velocity_estimator "${cpp_typesupport_target}")
target_link_libraries(object_predictor "${cpp_typesupport_target}")
# target_link_libraries(communicate_motors "${cpp_typesupport_target}" -ldxl_x64_cpp)
# target_link_libraries(test_node "${cpp_typesupport_target}")
target_link_libraries(frame_analyzer "${cpp_typesupport_target}")
# target_link_libraries(mallet_detector "${cpp_typesupport_target}")
# target_link_libraries(robo_arm_detector "${cpp_typesupport_target}")
# target_link_libraries(display_image "${cpp_typesupport_target}")
# target_link_libraries(puck_movement_analyzer "${cpp_typesupport_target}")
# target_link_libraries(mallet_motion_analyzer "${cpp_typesupport_target}") # prob better to call executables ???_node
# target_link_libraries(robo_arm_motion_analyzer "${cpp_typesupport_target}")
# target_link_libraries(game_controller "${cpp_typesupport_target}")
# target_link_libraries(motor_controller "${cpp_typesupport_target}" -ldxl_x64_cpp)
# target_link_libraries(object_motion_predictor "${cpp_typesupport_target}")

# # For python
# ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  # test_node
  camera_driver
  object_detector
  velocity_estimator
  object_predictor
  # communicate_motors
  frame_analyzer
  #   # display_image
#   # mallet_detector
#   # robo_arm_detector
#   # puck_movement_analyzer
#   # mallet_motion_analyzer
#   # robo_arm_motion_analyzer
#   # game_controller
  # motor_controller
#   # object_motion_predictor
  DESTINATION lib/${PROJECT_NAME})

# install(PROGRAMS
#   scripts/arm_motion_planner_node.py
#   # scripts/arm_motion_planner.py # maybe no need to install this
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
