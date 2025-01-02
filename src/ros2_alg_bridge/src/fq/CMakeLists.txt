cmake_minimum_required(VERSION 3.8)
project(fq)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  
  "msgs/Header.msg"
  "msgs/BaseBytes.msg"
  "msgs/BaseBaseData.msg"
  "msgs/BaseTargetInfo.msg"
  "msgs/BaseOrientation3D.msg"
  "msgs/BasePose.msg"
  "msgs/BaseAirplaneKinematicsData.msg"
  "msgs/BaseBeInterferenceData.msg"
  "msgs/BaseBeReconnaissanceData.msg"
  "msgs/BaseDamageConditionData.msg"
  "msgs/BaseInterferenceData.msg"
  "msgs/BaseKinematicsData.msg"
  "msgs/BaseLoadData.msg"
  "msgs/BaseReconnaissanceData.msg"
  "msgs/BasePlaneAttr.msg"
  "msgs/BaseZoneAttr.msg"
  "msgs/BaseBoundingBox.msg"
  
  "msgs/ActorAirplane.msg"
  "msgs/ActorAirplanes.msg"
  "msgs/ActorBuilding.msg"
  "msgs/ActorBuildings.msg"
  "msgs/ActorDroneSwarm.msg"
  "msgs/ActorDroneSwarms.msg"
  "msgs/ActorVehicle.msg"
  "msgs/ActorVehicles.msg"
  "msgs/ActorWarship.msg"
  "msgs/ActorWarships.msg"
  "msgs/ActorZone.msg"
  "msgs/ActorZones.msg"
  "msgs/ActorHitInfo.msg"
  "msgs/ActorHitInfos.msg"
  "msgs/ActorControlInfo.msg"
  "msgs/ActorControlInfos.msg"
  "msgs/ActorChangeDirectionInfo.msg"
  "msgs/ActorChangeDirectionInfos.msg"
  "msgs/ActorReleaseChaffInfo.msg"
  "msgs/ActorReleaseChaffInfos.msg"
  "msgs/ActorHuman.msg"
  "msgs/ActorHumans.msg"
  "msgs/ActorEquipment.msg"
  "msgs/ActorEquipments.msg"

  DEPENDENCIES geometry_msgs
)

ament_package()
