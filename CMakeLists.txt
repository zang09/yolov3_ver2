##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(yolov3_ver2)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS qt_build roscpp cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  roslib
)

#add_message_files
#generate_messages

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolov3_ver2
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

set (OpenCV_INClUDE_DIRS
 /usr/local/include
 /usr/local/include/opencv2
 )

set(OpenCV_LIB_DIR
    /usr/local/lib
    )

set(Opencv_LIBS
    opencv_core
    opencv_highgui
    opencv_imgcodecs
    opencv_videoio
    opencv_imgproc
    opencv_features2d
    opencv_xfeatures2d
    opencv_flann
    opencv_calib3d
    )


set(Darknet_INCLUDE_DIRS
  /home/robit/catkin_ws/src/yolov3_ver2/darknet/src
)

set(Darknet_LIB_DIR
  /home/robit/catkin_ws/src/yolov3_ver2/darknet
)

set(Darknet_LIBS
    darknet
)


################################################################################
# Build
################################################################################
include_directories(${catkin_INCLUDE_DIRS} ${Darknet_INCLUDE_DIRS})
link_directories(${Darknet_LIB_DIR})

add_executable(yolov3_ver2 src/yolov3_ver2.cpp src/darknetdetector.cpp)
target_link_libraries(yolov3_ver2 ${catkin_LIBRARIES} ${Opencv_LIBS} ${Darknet_LIBS})
install(TARGETS yolov3_ver2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


