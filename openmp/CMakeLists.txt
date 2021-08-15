cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_ros
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  message_filters
  image_transport
  cv_bridge
  tf
  tf_conversions
  tf2_ros
  tf2_eigen
  message_generation
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
message(STATUS "***** Open CV version: ${OpenCV_VERSION} *****")
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) 
  set(OpenMP_LIBS gomp)
endif()
####

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  includeTools
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL gazebo_ros OpenMP
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS includeTools
)

catkin_install_python(PROGRAMS
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(CALIBRATION_LIBRARY
    source/findSphericalTargetImage.cpp
    source/findSphericalTargetPointCloud.cpp
    ##includeTools/optimizationProblemTools/ceresSolver3Dto2DTool.cpp
    includeTools/optimizationProblemTools/circleParametersOptimization.cpp
    includeTools/optimizationProblemTools/eigenSolver3Dto3DTool.cpp
    includeTools/optimizationProblemTools/removeOutliersListTool.cpp
    includeTools/optimizationProblemTools/several3Dto3DCalibrationTool.cpp
    includeTools/optimizationProblemTools/several3Dto2DCalibrationTool.cpp
    includeTools/imageTools/generalProcessingTool.cpp
    includeTools/imageTools/sphericalWarperTool.cpp
    includeTools/pointCloudTools/clusterTool.cpp
    includeTools/pointCloudTools/cropboxTool.cpp
    includeTools/pointCloudTools/normalEstimationTool.cpp
    includeTools/pointCloudTools/planeSegmentationTool.cpp
    includeTools/pointCloudTools/sphereSegmentationTool.cpp
    includeTools/pointCloudTools/cropboxTool.cpp
    includeTools/rosTools/publishTools.cpp
    includeTools/eigenTools/rotationMatrixTools.cpp
    includeTools/eigenTools/generalTools.cpp
    )
add_library(${PROJECT_NAME} ${CALIBRATION_LIBRARY})
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
      ${EIGEN_LIBRARIES} ${pluginlib_DIR} -lstdc++fs ${OpenMP_LIBS})

add_library(animated_sphere_setup_0 source/animatedSphereSetup0.cpp)
target_link_libraries(animated_sphere_setup_0 ${GAZEBO_LIBRARIES} )

add_library(animated_sphere_setup_1 source/animatedSphereSetup1.cpp)
target_link_libraries(animated_sphere_setup_1 ${GAZEBO_LIBRARIES} )

add_executable(pointCloudRecorder source/pcdRecorder.cpp)
target_link_libraries(pointCloudRecorder ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${pluginlib_DIR} -lstdc++fs)

add_executable(publishCameraInfo source/publishCameraInfo.cpp)
target_link_libraries(publishCameraInfo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${pluginlib_DIR} -lstdc++fs)

add_executable(calibrateSensors source/calibrateSensors.cpp)
target_compile_options(calibrateSensors PRIVATE ${OpenMP_FLAGS})
add_dependencies(calibrateSensors ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibrateSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME} animated_sphere_setup_0 animated_sphere_setup_1 ${OpenMP_LIBS})

add_executable(simulateCalibrateSensors source/simulateCalibrateSensors.cpp)
target_link_libraries(simulateCalibrateSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME} animated_sphere_setup_0 animated_sphere_setup_1 ${OpenMP_LIBS})

add_executable(idealCalibrateSensors source/idealCalibrateSensors.cpp)
target_link_libraries(idealCalibrateSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME} animated_sphere_setup_0 animated_sphere_setup_1 ${OpenMP_LIBS})
