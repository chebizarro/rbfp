// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <catch2/catch.hpp>

#include <osm.h>
#include <iostream>
#include <Sdf.h>

using namespace osm;

TEST_CASE( "Test PBF to SDF Conversion", "[Gazebo OSM]" ) {

    auto pbf = "/home/bizarro/Documents/Projects/rbpf/tests/gazebo-osm/data/test.pbf";

    SECTION( "parser pbf to sdf" ) {

        auto result = pbf_to_sdf(pbf);

        REQUIRE_FALSE(result.empty());

    }

    SECTION( "Test SDF Class" ) {

        Sdf sdf { pbf };

        auto result = sdf.to_string();

        INFO(result)

        REQUIRE_FALSE(result.empty());
    }
}