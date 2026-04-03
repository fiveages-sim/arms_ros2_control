cmake_minimum_required(VERSION 3.8)
project(dobot_force_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EnableFTSensor.srv"
  "srv/SixForceHome.srv"
  "srv/GetForce.srv"
  "srv/ForceDriveMode.srv"
  "srv/ForceDriveSpeed.srv"
  "srv/StopDrag.srv"
  "srv/FCForceMode.srv"
  "srv/FCSetDeviation.srv"
  "srv/FCSetForceLimit.srv"
  "srv/FCSetMass.srv"
  "srv/FCSetStiffness.srv"
  "srv/FCSetDamping.srv"
  "srv/FCOff.srv"
  "srv/FCSetForceSpeedLimit.srv"
  "srv/FCSetForce.srv"
)

ament_package()
