cmake_minimum_required(VERSION 3.14)
project(rbpf)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

include(cmake/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(rxcpp REQUIRED)
find_package(RoutingKit REQUIRED)
find_package(gazebo REQUIRED)
find_package(frozen REQUIRED)
find_package(pugixml REQUIRED)

include_directories(lib ${GAZEBO_INCLUDE_DIRS} gazebo-osm)

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
        lib/routing.h lib/Point.cpp lib/Point.h lib/Route.cpp lib/Route.h lib/RoutingEngine.h lib/OSMOfflineRoutingEngine.cpp lib/OSMOfflineRoutingEngine.h)

add_executable(routing routing/routing.cpp)

target_link_libraries(routing ${RoutingKit_LIBRARIES})

add_executable(sim sim/Simulation.cpp sim/Simulation.h sim/sim.cpp sim/Agent.cpp sim/Agent.h)

add_library(gazebo-osm SHARED
        gazebo-osm/OSMWorld.h
        gazebo-osm/OSMWorld.cpp
        gazebo-osm/osm.h
        gazebo-osm/osm.cpp gazebo-osm/Sdf.cpp gazebo-osm/Sdf.h)

find_package(Catch2 REQUIRED)

add_executable(rbpf-tests
        tests/main.cpp tests/gazebo-osm/gazebo-osm-tests.cpp)

target_link_libraries(rbpf-tests
        Catch2::Catch2
        gazebo-osm
        ${GAZEBO_LIBRARIES}
        ${pugixml_LIBS}
        ${RoutingKit_LIBS})
