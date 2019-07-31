cmake_minimum_required(VERSION 3.14)
project(rbpf)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

include(cmake/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(rxcpp REQUIRED)
find_package(RoutingKit REQUIRED)

include_directories(lib)

add_executable(rbpf
        main.cpp
        lib/Particle.cpp
        lib/Particle.h
        lib/rbpf.h
        lib/rbpf.cpp
        lib/ekf.cpp
        lib/ekf.h
        lib/State.h
        lib/filter.cpp
        lib/filter.h
        )

add_executable(routing routing/routing.cpp)

target_link_libraries(routing ${RoutingKit_LIBRARIES})