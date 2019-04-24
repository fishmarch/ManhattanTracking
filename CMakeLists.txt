cmake_minimum_required(VERSION 3.13)
project(ManhattanTracking)

set(CMAKE_CXX_COMPILER "g++")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

link_directories(${PROJECT_SOURCE_DIR}/lib)
#add_subdirectory(${PROJECT_SOURCE_DIR}/src)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED )
find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        src/Initializer.cpp
        src/Tracking.cpp
        src/Frame.cpp
        src/System.cpp
        src/base.cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PCL_LIBRARIES}
        )

add_executable(manhattanTrack src/main.cpp)
target_link_libraries(manhattanTrack ${PROJECT_NAME})