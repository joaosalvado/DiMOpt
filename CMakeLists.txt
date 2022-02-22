cmake_minimum_required(VERSION 3.10)

# set the project name
project(mropt)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++17")
#set(CMAKE_CXX_CLANG_TIDY "/usr/bin/clang-tidy-11")
#set(CMAKE_CXX_CPPCHECK "/usr/bin/cppcheck")
#set(CMAKE_CXX_CPPLINT "/usr/local/bin/cpplint")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")



#Project directories
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/")
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
set(Python3_EXECUTABLE "/usr/bin/python3")

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(casadi)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(Threads REQUIRED)
find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})

find_package(Matplot++ REQUIRED)

add_subdirectory(include/)
add_subdirectory(src/)
add_subdirectory(tests/)

#include_directories(${CMAKE_PROJECT_DIRECTORY}/include/)