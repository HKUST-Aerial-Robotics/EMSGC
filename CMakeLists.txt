cmake_minimum_required(VERSION 2.8.3)
project(emsgc)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    third-party/gco
    third-party/delaunator/include
    ${OpenCV_INCLUDE_DIRS})

set(HEADERS
    include/emsgc/core/event_motion_segmentation.h
    include/emsgc/core/image_warped_events.h
    include/emsgc/core/numerical_deriv.h
    include/emsgc/core/contrastFunctor.h
    include/emsgc/container/EventQueueMat.h
    include/emsgc/container/EventMRF.h
    include/emsgc/container/PerspectiveCamera.h
    include/emsgc/tools/utils.h
    include/emsgc/tools/TicToc.h
    third-party/gco/energy.h
    third-party/gco/block.h
    third-party/gco/GCoptimization.h
    third-party/gco/graph.h
    third-party/gco/LinkedBlockList.h
    third-party/delaunator/include/delaunator.hpp)

set(SOURCES
    src/container/PerspectiveCamera.cpp
    src/core/event_motion_segmentation.cpp
    src/core/image_warped_events.cpp
    src/core/optim_contrast_gsl.cpp
    src/core/numerical_deriv.cpp
    src/core/optimizeLabels.cpp
    src/core/optimizeModels.cpp
    third-party/gco/GCoptimization.cpp
    third-party/gco/graph.cpp
    third-party/gco/LinkedBlockList.cpp
    third-party/gco/maxflow.cpp)
cs_add_library(${PROJECT_NAME}_LIB ${HEADERS} ${SOURCES})

## Node
cs_add_executable(emsgc_main emsgc_main.cpp)
target_link_libraries(emsgc_main ${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp ${gsl_LIBRARIES})
