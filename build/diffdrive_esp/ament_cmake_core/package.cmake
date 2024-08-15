set(_AMENT_PACKAGE_NAME "diffdrive_esp")
set(diffdrive_esp_VERSION "0.0.0")
set(diffdrive_esp_MAINTAINER "Dr.-Ing. Denis Štogl <denis.stogl@stoglrobotics.de>, Bence Magyar <bence.magyar.robotics@gmail.com>, Christoph Froehlich <christoph.froehlich@ait.ac.at>")
set(diffdrive_esp_BUILD_DEPENDS "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "libserial-dev")
set(diffdrive_esp_BUILDTOOL_DEPENDS "ament_cmake")
set(diffdrive_esp_BUILD_EXPORT_DEPENDS "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "libserial-dev")
set(diffdrive_esp_BUILDTOOL_EXPORT_DEPENDS )
set(diffdrive_esp_EXEC_DEPENDS "controller_manager" "diff_drive_controller" "joint_state_broadcaster" "joint_state_publisher_gui" "robot_state_publisher" "ros2_control_demo_description" "ros2_controllers_test_nodes" "ros2controlcli" "ros2launch" "rviz2" "xacro" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "libserial-dev")
set(diffdrive_esp_TEST_DEPENDS "ament_cmake_gtest" "ament_cmake_pytest" "launch_testing_ament_cmake" "launch_testing_ros" "liburdfdom-tools" "xacro")
set(diffdrive_esp_GROUP_DEPENDS )
set(diffdrive_esp_MEMBER_OF_GROUPS )
set(diffdrive_esp_DEPRECATED "")
set(diffdrive_esp_EXPORT_TAGS)
list(APPEND diffdrive_esp_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
