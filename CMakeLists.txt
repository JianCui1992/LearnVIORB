cmake_minimum_required(VERSION 3.5)
project(vi_orb)

set(VTRACKER_DIR /home/jianc/uos_stable)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native ")

#glew
include_directories(SYSTEM ${VTRACKER_DIR}/src/uos_depend/glew/16.04/include)
link_directories(${VTRACKER_DIR}/src/uos_depend/glew/16.04/lib)

#pangolin
include_directories(SYSTEM ${VTRACKER_DIR}/src/uos_3rdparty/pangolin/16.04/include)
link_directories(${VTRACKER_DIR}/src/uos_3rdparty/pangolin/16.04/lib)

#eigen
include_directories(SYSTEM ${VTRACKER_DIR}/src/uos_depend/eigen3/include)

#opencv
include_directories(SYSTEM ${VTRACKER_DIR}/src/uos_3rdparty/opencv33/16.04/include)
link_directories(${VTRACKER_DIR}/src/uos_3rdparty/opencv33/16.04/lib)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_calib3d opencv_features2d opencv_imgproc opencv_imgcodecs opencv_video)

#cholmod
include_directories(SYSTEM ${VTRACKER_DIR}/src/uos_depend/carto/16.04/include/suitesparse)
link_directories(${VTRACKER_DIR}/src/uos_depend/carto/16.04/lib)

#ceres 
include_directories(SYSTEM ${VTRACKER_DIR}/src/uos_depend/carto/16.04/include)
link_directories(${VTRACKER_DIR}/src/uos_depend/carto/16.04/lib)
set(CERES_LIBS protobuf glog webp lua5.2 gomp lapack f77blas atlas spqr cxsparse)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/src/common
${PROJECT_SOURCE_DIR}/src/data
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(LIBS 
${OpenCV_LIBS}
pangolin
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
cholmod
lapack f77blas
GLEW
GL
)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc

include/Frame.h
include/KeyFrame.h
include/Tracking.h
include/LocalMapping.h

src/IMU/configparam.h
src/IMU/configparam.cpp

src/IMU/imudata.h
src/IMU/imudata.cpp
src/IMU/IMUPreintegrator.h
src/IMU/IMUPreintegrator.cpp
src/IMU/so3.cpp
src/IMU/so3.h
src/IMU/NavState.h
src/IMU/NavState.cpp

src/IMU/g2otypes.h
src/IMU/g2otypes.cpp

src/data/data.cpp
src/common/common_include.cpp
)
target_link_libraries(${PROJECT_NAME}
${LIBS}
)

add_executable(VIO
src/test.cpp
)

target_link_libraries(VIO
${LIBS}
${PROJECT_NAME}
)
