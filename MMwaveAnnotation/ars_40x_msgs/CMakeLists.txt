cmake_minimum_required(VERSION 3.5)
project(ars_40x_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
	${PROJECT_NAME}
	"msg/ClusterGeneral.msg"
	"msg/ClusterGeneralArray.msg"
	"msg/ObjectGeneral.msg"
	"msg/ObjectGeneralArray.msg"
	"msg/RadarState.msg"
	"msg/SpeedInformation.msg"
	"msg/YawRateInformation.msg"
	"msg/ObjectExtended.msg"
	"msg/ObjectExtendedArray.msg"
	"srv/RadarConfig.srv"
	"srv/RadarFilter.srv"
	DEPENDENCIES std_msgs
)

ament_package()
