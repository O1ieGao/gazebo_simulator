cmake_minimum_required(VERSION 3.8)
project(hrstek_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(schedular_msg_files 
  msg/AgentMonitorASMsg.msg
  msg/Robotmonitor.msg
  msg/Robotmonitoras.msg
  msg/AgentFtptMsg.msg
  msg/AgentFtptListMsg.msg
  msg/Path2NodeMsg.msg
  msg/Circle.msg
  msg/AgentStateListMsg.msg
  msg/TaskFeedback2dMsg.msg
  msg/Trajectory2dMsg.msg
  msg/TrajectoryPoint2dMsg.msg
  msg/TrajectoryPoint2dOptionMsg.msg
  msg/Action.msg
  msg/ActionWaitParams.msg
  msg/TaskComplex.msg
  msg/TaskSimplex.msg
  msg/TaskSimplex2dMsg.msg
  msg/AllTaskSimplex2dMsg.msg
  msg/NodeStateMsg.msg
  msg/BatteryInfo.msg
  msg/ChargingStationMsg.msg
  msg/ChargingStationStatusMsg.msg
  msg/ChargingStationStatusArrayMsg.msg
  msg/Detection.msg
  msg/DetectionArray.msg
  msg/RoadmapNode2D.msg
  srv/TaskSimplex2dSrv.srv
  srv/TaskSimplex2dSrvRequest.srv
  srv/ChargingDoneStatusSrv.srv
)

# set(perceptron_msg_files
    # msg/BoundingBox2d.msg
# )

# set(grasp_msg_files
  # msg/Setrealtimepush.msg
  # msg/Armsoftversion.msg
  # msg/Sixforce.msg
  # msg/Jointerrorcode.msg
  # msg/Forcepositionmovejoint.msg
  # # msg/Forcepositionmovejoint75.msg
  # msg/Forcepositionmovepose.msg
  # msg/Setforceposition.msg
  # msg/Getallframe.msg
  # msg/Jointpos.msg
  # # msg/Jointpos75.msg
  # msg/Cartepos.msg
  # msg/Jointerrclear.msg
  # msg/Gripperpick.msg
  # msg/Gripperset.msg
  # msg/Handangle.msg
  # msg/Handforce.msg
  # msg/Handposture.msg
  # msg/Handseq.msg
  # msg/Handspeed.msg
  # msg/Armoriginalstate.msg
  # msg/Armstate.msg
  # msg/Liftspeed.msg
  # msg/Liftheight.msg
  # msg/Liftstate.msg
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${schedular_msg_files}
  # ${perceptron_msg_files}
  # ${grasp_msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

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