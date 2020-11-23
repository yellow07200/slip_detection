cmake_minimum_required(VERSION 2.8.3)
project(slip_detection_davis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules)
find_package(OpenCV 3.3.0 REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
actionlib
  actionlib_msgs
  dynamic_reconfigure
  moveit_core
  pluginlib
  geometry_msgs
  message_generation
  rosconsole
  roscpp
  rospy
  sensor_msgs
  control_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_simple_controller_manager
  std_msgs
  tf
  cv_bridge
  image_transport
  cmake_modules
  pcl_conversions
  pcl_msgs
  pcl_ros
  dvs_msgs
  geometry_msgs
  baxter_core_msgs
  baxter_gripper_server
  
)
 
add_message_files(
   FILES
   SlipDetectionDataComplete_analysis_final3.msg
   SlipDetectionDataComplete_analysis_final4.msg
   CompressiveForceGrasp_stable_Final.msg
)
 add_service_files(
   FILES
  object_test.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   baxter_core_msgs
 )

include_directories ("/home/user/catkin_ws/opencv-3.4.0/include")
include_directories( ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include/")

include_directories ("/home/user/catkin_ws/src/slip_detection_davis/include/")
find_package(Boost REQUIRED COMPONENTS thread system)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  INCLUDE_DIRS include
  LIBRARIES multi_robot_system
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_generation rosconsole roscpp rospy sensor_msgs std_msgs tf control_msgs  cmake_modules cv_bridge image_transport baxter_core_msgs baxter_gripper_server)

###########
## Build ##
###########
include_directories ("/home/user/catkin_ws/src/baxter_gripper_server/include/")

include_directories ("/home/user/catkin_ws/src/slip_detection_davis/matlab_related/include/")
include_directories ("${PROJECT_SOURCE_DIR}/include/")
find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
find_package (VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
find_package(actionlib REQUIRED)
include_directories(${actionlib_INCLUDE_DIRS})


#add_executable(slip_data_test src/slip_data_test.cpp)
#target_link_libraries(slip_data_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${actionlib_LIBRARIES} ${Eigen_LIBRARIES} )
#add_dependencies(slip_data_test ${catkin_LIBRARIES})


#add_executable(data_processing_test_node src/data_processing_test_node.cpp src/davisdataprocessing.cpp src/CornerDetectorARC.cpp src/CornerDetectorFAST.cpp src/CornerDetectorHARRIS.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp)
#target_link_libraries(data_processing_test_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} )
#add_dependencies(data_processing_test_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES})


#add_executable(object_detection_test_node src/ObjectDetection.cpp src/ObjectDetectionTest.cpp)
#target_link_libraries(object_detection_test_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${OpenCV_LIBRARIES})
#add_dependencies(object_detection_test_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )


#add_executable(slip_detect_soft_hand_node src/SoftHandDetection.cpp src/slip_detect_soft_test.cpp)
#target_link_libraries(slip_detect_soft_hand_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${OpenCV_LIBRARIES})
#add_dependencies(slip_detect_soft_hand_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )


#add_executable(visual_servo_node src/VisualServoing.cpp src/visual_servoing_node.cpp src/CornerDetectorHARRIS.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp)
#target_link_libraries(visual_servo_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} ) 
#add_dependencies(visual_servo_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )



#add_executable(experiment_node src/softhandslipdetection.cpp src/experiment_test_node.cpp  src/CornerDetectorHARRIS.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp)
#target_link_libraries(experiment_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${OpenCV_LIBRARIES})
#add_dependencies(experiment_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES})

#add_executable(SlipDetectionRawTest_node src/SlipDetectionRawTest.cpp src/SlipDetectionRawTestNode.cpp src/CornerDetectorHARRIS.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp)
#target_link_libraries(SlipDetectionRawTest_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${OpenCV_LIBRARIES} )
#add_dependencies(SlipDetectionRawTest_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )

#working control_node
add_executable(detect_control_node src/softhandslipdetection.cpp src/experiment_test_node.cpp src/CornerDetectorHARRIS.cpp src/CornerDetectorARC.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniAggregationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniDefuzzificationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniImplicationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzifyInputs.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs_terminate.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/rtGetInf.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/rt_nonfinite.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/trapmf1.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/trimf1.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyOperators.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs_initialize.cpp)
target_link_libraries(detect_control_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} ) 
add_dependencies(detect_control_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )




#add_executable(detect_control_node_Fast src/softhandslipdetection.cpp src/experiment_test_node.cpp src/CornerDetectorFAST.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniAggregationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniDefuzzificationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniImplicationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzifyInputs.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs_terminate.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/rtGetInf.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/rt_nonfinite.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/trapmf1.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/trimf1.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyOperators.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs_initialize.cpp)
#target_link_libraries(detect_control_node_Fast ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} ) 
#add_dependencies(detect_control_node_Fast ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )


#add_executable(detect_control_node_ARC src/softhandslipdetection.cpp src/experiment_test_node.cpp src/CornerDetectorARC.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniAggregationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniDefuzzificationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyMamdaniImplicationMethod.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzifyInputs.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs_terminate.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/rtGetInf.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/rt_nonfinite.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/trapmf1.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/trimf1.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/applyOperators.cpp ${PROJECT_SOURCE_DIR}/matlab_related/src/fuzzy2inputs_initialize.cpp)
#target_link_libraries(detect_control_node_ARC ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} ) 
#add_dependencies(detect_control_node_ARC ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )



#add_executable(Compressive_force_experiment_node src/CompressiveForceGrasp.cpp src/CompressiveForceGrasp_node.cpp  src/CornerDetectorHARRIS.cpp src/harris_fixed_distinct_queue.cpp src/harris_distinct_queue.cpp)
#target_link_libraries(Compressive_force_experiment_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${OpenCV_LIBRARIES})
#add_dependencies(Compressive_force_experiment_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}  )

# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#add_executable(testing_corner_edge src/test_node.cpp src/test_data_processing.cpp )
#target_link_libraries(data_processing_test_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
#add_dependencies(data_processing_test_node ${catkin_LIBRARIES}  ${Eigen_LIBRARIES})
#############
## Install ##
#############
#install(slip_data_test
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
