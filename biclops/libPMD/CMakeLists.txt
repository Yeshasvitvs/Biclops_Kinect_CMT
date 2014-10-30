PROJECT(libPMD)

cmake_minimum_required(VERSION 2.8)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(LIBRARY_OUTPUT_PATH ${libPMD_SOURCE_DIR}/lib)

include_directories(include ${biclops_SOURCE_DIR}/libUtils/include)



file(GLOB pmd_SOURCES RELATIVE ${libPMD_SOURCE_DIR} src/*.cpp)

ADD_DEFINITIONS(-Wall)

ADD_LIBRARY(PMD ${pmd_SOURCES})

link_directories(${libUtils_SOURCE_DIR}/Utils)


