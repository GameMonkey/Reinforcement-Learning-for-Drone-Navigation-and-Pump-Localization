cmake_minimum_required(VERSION 3.30)
project(gridHelperLib)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

ExternalProject_add(json-ext
        URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.zip
        URL_HASH SHA512=5c671c0bc2406d12a65a1da7aac43c3a1936370fbc4f06460042e75a8459a8ae64f4d4b7e3818107845b003b251a1ba69bf77495ab978b21db159dead6829681
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
)

include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)


add_library(gridHelperLib SHARED gridHelper.cpp
        json_reader.h
        json_reader.cpp
        gridHelper.h)

add_executable(gridHelper gridHelper.cpp
        json_reader.h
        json_reader.cpp
        gridHelper.h
        gridHelperTester.cpp)

add_dependencies(gridHelperLib json-ext)
add_dependencies(gridHelper json-ext)

add_custom_target(BuildAllTargets)
add_dependencies(BuildAllTargets gridHelperLib gridHelper)