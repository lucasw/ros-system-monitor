cmake_minimum_required(VERSION 3.0)
project(ros_system_monitor VERSION 0.1.0)

find_package(catkin REQUIRED COMPONENTS
  # roslint
  rospy
  std_msgs
  diagnostic_msgs
)

# TODO(lucasw) make this pass
# roslint_python()
catkin_package()

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  bin/chrony_monitor.py
  bin/cpu_monitor.py
  bin/hdd_monitor.py
  bin/mem_monitor.py
  bin/net_monitor.py
  bin/ntp_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
