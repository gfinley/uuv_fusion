cmake_minimum_required(VERSION 2.8.3)
project(cavr_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  nav_msgs
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES 
    StateMeta.msg 
    BVTPingMeta.msg 
    NavSatFixAug.msg 
    VLBVMsg.msg 
    GSS_Nav_Sol.msg
    Heartbeat.msg
    ObsAvoid.msg
    MissionControl.msg
    SeaCAN_Rudder_Feedback.msg
    SeaCAN_RudderCtrl_Status.msg
    SeaCAN_Throttle_Status.msg
    SeaCAN_Engine_Status.msg
    SeaCAN_Rudder_Command.msg
    SeaCAN_Throttle_Command.msg
    Partition.msg
    Waypoint.msg
    MissionWaypoint.msg
    MissionWaypoints.msg
    UXV_State.msg
    RawFeature.msg
    Feature.msg
    FeatureDB.msg
    Edge.msg
    Node.msg
    Graph.msg
    Airmar_Triducer.msg
    ATLAS_Config_Status.msg
    ATLAS_Environment.msg
    RemusAcommsRangerCcl.msg
    RemusBattery.msg
    RemusStatus.msg
    RemusVersion.msg
    RemusDepthOverride.msg
    RemusFinControl.msg
    RemusFluorometer.msg
    RemusXBodyControl.msg
    RemusFullOverride.msg
    RemusHover.msg
    RemusRequest.msg
    RemusRedirect.msg
    RemusReturnClosest.msg
    RemusSetDepth.msg
    RemusSetDestination.msg
    RemusSetHeading.msg
    RemusSet.msg
    RemusSetPosition.msg
    RemusSetSpeed.msg
    RemusModemMessage.msg
    RemusADCP.msg
    RemusCTD.msg
    RemusRoute.msg
    RemusRun.msg
    RTWDebug.msg
    CtrlLL.msg
    CtrlHL.msg
    CtrlCmd.msg
    CtrlErr.msg
    RemusState.msg
    RemusFluorometer.msg
    RemusBattery.msg
    RemusVersion.msg
    RemusNavFix.msg
    RemusStatusResponse.msg
    RemusRudderCmd.msg
    RemusDivePlaneCmd.msg
    RemusThrusterCmds.msg
    RemusNavFix.msg
    RemusTrajectory.msg
    uModemData.msg
    uModemBytes.msg
    uModemPositReq.msg
    RemusUSBL.msg
    RemusNavError.msg
    AComms.msg
    ModemQuery.msg
    ModemRange.msg
    ModemPingLBL.msg
    ModemReplyLBL.msg
    Path.msg
    Line.msg
    MapFeature.msg
    TopoFeature.msg
    TopoState.msg
    Odometry.msg
    PointWithCovariance.msg
    PointWithCovarianceStamped.msg
    ScanEagleRun.msg
    KLV.msg
    ScanEagleOrbitOverride.msg
    Snmp.msg
    WaveRelayNeighborSNR.msg
    AlticamCmd.msg
    ScanEagleCmd.msg
    MapParams.msg
    NCS_Solution.msg
    NCS_Run.msg
    AdaptiveSubmod.msg
    ##Metadata.msg
    ##GlobalPosition.msg
    ##LatLon.msg
    ##Pose.msg
    ##CTD.msg
    ##Environmental.msg
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
    PFMapQuery.srv
    FeatureMapQuery.srv
    TopoMapQuery.srv
 )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES MissionGoal.action PathGoal.action DynamicMissionGoals.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs   
  geometry_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
  cavr_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES com_cavr_msgs
 CATKIN_DEPENDS message_runtime nav_msgs roscpp geometry_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(com_cavr_msgs
#   src/${PROJECT_NAME}/com_cavr_msgs.cpp
# )

## Declare a cpp executable
# add_executable(com_cavr_msgs_node src/com_cavr_msgs_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(com_cavr_msgs_node com_cavr_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(com_cavr_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS com_cavr_msgs com_cavr_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_com_cavr_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
