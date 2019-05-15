cmake_minimum_required(VERSION 3.13)
project(slam)

set(CMAKE_CXX_STANDARD 17)

# 增加opencv的依赖
find_package(OpenCV REQUIRED)

#find_package(libb64 REQUIRED)


# 增加PCL库的依赖
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )

find_package(PCL REQUIRED)
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 添加头文件和库文件
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})


# 添加g2o的依赖
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O REQUIRED)


# CSparse
FIND_PACKAGE(CSparse REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})



ADD_EXECUTABLE(main main.cpp)

ADD_EXECUTABLE(test_opencv test/test_opencv.cpp)
TARGET_LINK_LIBRARIES(test_opencv ${OpenCV_LIBS} b64)


ADD_EXECUTABLE(generate_pointcloud test/generatePointCloud.cpp)
TARGET_LINK_LIBRARIES(generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})


ADD_LIBRARY(slambase SHARED test/slamBase.cpp)
TARGET_LINK_LIBRARIES(slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})


ADD_EXECUTABLE(detectFeatures test/detectFeatures.cpp)
TARGET_LINK_LIBRARIES(detectFeatures slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})


ADD_EXECUTABLE(joinPointCloud test/joinPointCloud.cpp)
TARGET_LINK_LIBRARIES(joinPointCloud slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})


ADD_EXECUTABLE(visualOdometry test/visualOdometry.cpp)
TARGET_LINK_LIBRARIES(visualOdometry slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})


ADD_EXECUTABLE(slamEnd test/slamEnd.cpp)
TARGET_LINK_LIBRARIES(slamEnd
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)


ADD_EXECUTABLE(slam test/slam.cpp)
TARGET_LINK_LIBRARIES(slam
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)