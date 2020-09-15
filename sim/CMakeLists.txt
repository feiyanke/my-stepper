cmake_minimum_required(VERSION 3.8)
project(cpp_test1)

set(CMAKE_CXX_STANDARD 11)
find_package(Boost COMPONENTS system REQUIRED)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS} -static -lws2_32")
if(WIN32)
    link_libraries(ws2_32)
endif(WIN32)

if(Boost_FOUND)
    MESSAGE( STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}.")
    MESSAGE( STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}.")
    MESSAGE( STATUS "Boost_LIB_VERSION = ${Boost_LIB_VERSION}.")
endif()

include_directories(${Boost_INCLUDE_DIRS})

add_executable(cpp_test1 my_stepper.h my_stepper.cpp main.cpp)
target_link_libraries (cpp_test1 ${Boost_LIBRARIES})
