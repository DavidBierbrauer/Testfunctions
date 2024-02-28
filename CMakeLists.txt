cmake_minimum_required(VERSION 3.8)
project(haive_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/Power.msg"
  "msg/PolarHaiveActuatorPosition.msg"
  "srv/GetTemperature.srv"
  "srv/SwitchPower.srv"
  "srv/GoToThetaR.srv"
  "srv/GoToXY.srv"
  "srv/GoToZ.srv"
  "action/Navigate2D.action"
  "srv/Inception.srv"
  
  DEPENDENCIES sensor_msgs geometry_msgs
  )



ament_package()
