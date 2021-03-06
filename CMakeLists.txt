cmake_minimum_required(VERSION 2.8.3)
project(brass_gazebo_plugins)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(ignition-math2 REQUIRED)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_python_setup()

add_message_files(
  FILES
  BrassBump.msg
)

add_service_files(
  FILES
  ToggleHeadlamp.srv
  SetCharge.srv
  SetCharging.srv
  SetKinectMode.srv
  SetLidarMode.srv
  SetCameraMode.srv
)

generate_dynamic_reconfigure_options(
  cfg/CameraSynchronizer.cfg
  cfg/GazeboRosCamera.cfg
  cfg/GazeboRosOpenniKinect.cfg
  cfg/Hokuyo.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
  ${IGNITION-MATH2_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
  ${Boost_LIBRARYY_DIRS}
)


if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  catkin_package(INCLUDE_DIRS include LIBRARIES gazebo_ros_harness)
endif()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${IGNITION-MATH2_CXX_FLAGS} -std=c++11")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
#  vision_reconfigure 
#  gazebo_ros_utils 
 gazebo_ros_camera_utils 
 gazebo_ros_camera 
#  gazebo_ros_depth_camera 
   gazebo_ros_openni_kinect  
   gazebo_ros_battery
   gazebo_ros_headlamp
   gazebo_ros_laser 
  gazebo_ros_kobuki
#  gazebo_ros_block_laser 
#  gazebo_ros_bumper
#  gazebo_ros_template
#  gazebo_ros_projector
#  gazebo_ros_prosilica
#  gazebo_ros_video
#  gazebo_ros_planar_move

  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  camera_info_manager
  std_msgs
)



add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Executables
add_executable(hokuyo_node src/hokuyo_node.cpp)
add_dependencies(hokuyo_node ${PROJECT_NAME}_gencfg)
target_link_libraries(hokuyo_node
  ${catkin_LIBRARIES}
)

add_definitions(-fPIC) # what is this for?


add_library(gazebo_ros_camera_utils src/gazebo_ros_camera_utils.cpp)
add_dependencies(gazebo_ros_camera_utils ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_camera_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_camera src/gazebo_ros_camera.cpp)
add_dependencies(gazebo_ros_camera ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils CameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_openni_kinect src/gazebo_ros_openni_kinect.cpp)
add_dependencies(gazebo_ros_openni_kinect ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_openni_kinect gazebo_ros_camera_utils DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_battery SHARED src/energy_monitor.cc)
add_dependencies(gazebo_ros_battery ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_battery ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_headlamp src/control_light.cc)
add_dependencies(gazebo_ros_headlamp ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_headlamp ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

add_library(gazebo_ros_laser src/gazebo_ros_laser.cpp)
add_dependencies(gazebo_ros_laser ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_laser RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_kobuki src/gazebo_ros_kobuki.cpp src/gazebo_ros_kobuki_updates.cpp src/gazebo_ros_kobuki_loads.cpp src/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_kobuki ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

##
## Add your new plugin here
##

## Template
#add_library(gazebo_ros_template src/gazebo_ros_template.cpp)
#target_link_libraries(gazebo_ros_template ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
#  hokuyo_node
#  vision_reconfigure
#  camera_synchronizer
#  gazebo_ros_utils
  gazebo_ros_camera_utils
  gazebo_ros_camera
#  gazebo_ros_depth_camera
  gazebo_ros_openni_kinect
  gazebo_ros_battery
  gazebo_ros_headlamp
  gazebo_ros_laser
#  gazebo_ros_openni_kinect
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )



install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS scripts/set_wrench.py scripts/set_pose.py scripts/gazebo_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# Tests
# These need to be run with -j1 flag because gazebo can't be run
# in parallel.
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(set_model_state-test
                    test/set_model_state_test/set_model_state_test.test
                    test/set_model_state_test/set_model_state_test.cpp)
  target_link_libraries(set_model_state-test ${catkin_LIBRARIES})


  if (ENABLE_DISPLAY_TESTS)
    add_rostest_gtest(depth_camera-test
                      test/camera/depth_camera.test
                      test/camera/depth_camera.cpp)
    target_link_libraries(depth_camera-test ${catkin_LIBRARIES})
    add_rostest_gtest(multicamera-test
                      test/camera/multicamera.test
                      test/camera/multicamera.cpp)
    target_link_libraries(multicamera-test ${catkin_LIBRARIES})
    add_rostest_gtest(camera-test
                      test/camera/camera.test
                      test/camera/camera.cpp)
    target_link_libraries(camera-test ${catkin_LIBRARIES})
    add_rostest_gtest(distortion_barrel_test
                      test/camera/distortion_barrel.test
                      test/camera/distortion.cpp)
    target_link_libraries(distortion_barrel_test ${catkin_LIBRARIES})
    add_rostest_gtest(distortion_pincushion_test
                      test/camera/distortion_pincushion.test
                      test/camera/distortion.cpp)
    target_link_libraries(distortion_pincushion_test ${catkin_LIBRARIES})
  endif()
endif()
