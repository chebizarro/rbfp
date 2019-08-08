// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include "osm.h"

#include <routingkit/osm_decoder.h>
#include <routingkit/osm_graph_builder.h>
#include <routingkit/id_mapper.h>
#include <routingkit/geo_dist.h>

#include <pugixml.hpp>

#include <frozen/unordered_map.h>
#include <frozen/string.h>

#include <sstream>
#include <iostream>

using namespace RoutingKit;

struct OSMNode {
    double lat;
    double lon;
    const TagMap& tags;
};

struct OSMWay {
    const std::vector<std::uint64_t> & node_list;
    const TagMap& tags;
};

static constexpr std::pair<frozen::string, float> highwayList[] = {
        { "footway",        0.3 },
        { "pedestrian",       3 },
        { "motorway",        14 },
        { "motorway_link",   13 },
        { "trunk",           12 },
        { "trunk_link",      11 },
        { "primary",         10 },
        { "primary_link",     9 },
        { "secondary",        8 },
        { "secondary_link",   7 },
        { "tertiary",         6 },
        { "tertiary_link",    5 },
        { "residential",      3 },
        { "steps",          0.8 }
};

static constexpr auto highway_types = frozen::make_unordered_map(highwayList);

static constexpr std::pair<frozen::string, frozen::string> models[] = {
        { "stop",               "stop_sign" },
        { "street_lamp",        "lamp_post" },
        { "traffic_signals",    "construction_cone" },
        { "fire hydrant",       "fire_hydrant" },
        { "give_way",           "speed_limit" },
        { "bus_stop",           "robocup14_spl_goal" },
        { "fuel",               "gas_station" }
};

static constexpr auto model_types = frozen::make_unordered_map(models);

static constexpr std::pair<frozen::string, frozen::string> amenities[] = {
        {"school",          "Purple" },
        {"post_office",     "Orange" },
        {"university",      "Purple" },
        {"library",         "Purple" },
        {"bar",             "Blue" },
        {"cafe",            "Blue" },
        {"pub",             "Blue" },
        {"restaurant",      "Blue" },
        {"fast_food",       "Blue" },
        {"college",         "Purple" },
        {"kindergarten",    "Purple" }
};

static constexpr auto amenity_colors = frozen::make_unordered_map(amenities);

auto init_sdf(pugi::xml_document & doc, const std::string & version) {
    auto root = doc.append_child("sdf");
    root.append_attribute("version") = version.c_str();
    auto world = root.append_child("world");
    world.append_attribute("name") = "default";
    return world;
}



auto setup_sdf(pugi::xml_document & doc, const std::string & version) {

    auto root = doc.append_child("sdf");
    root.append_attribute("version") = version.c_str();
    auto world = root.append_child("world");

    world.append_attribute("name") = "default";

    auto spherical = world.append_child("spherical_coordinates");
    spherical
        .append_child("surface_model")
        .set_value("EARTH_WGS84");
    spherical.append_child("latitude_deg");
    spherical.append_child("longitude_deg");
    spherical
        .append_child("elevation")
        .set_value("0.0");
    spherical
        .append_child("heading_deg")
        .set_value("0");

    world
        .append_child("include")
        .append_child("uri")
        .set_value("model://sun");

    auto physics = world.append_child("physics");
    physics.append_attribute("name") = "default_physics";
    physics.append_attribute("default") = "0";
    physics.append_attribute("type") = "ode";
    physics
        .append_child("max_step_size")
        .set_value("0.001");
    physics
        .append_child("real_time_factor")
        .set_value("1");
    physics
        .append_child("real_time_update_rate")
        .set_value("1000");

    auto ground = world.append_child("model");
    ground.append_attribute("name") = "ground_plane";
    ground
        .append_child("static")
        .set_value("1");
    auto link = ground.append_child("link");
    link.append_attribute("name") = "link";

    auto collision = link.append_child("collision");
    collision.append_attribute("name") = "collision";

    auto plane = collision
        .append_child("geometry")
        .append_child("plane");
    plane
        .append_child("normal")
        .set_value("0 0 1");
    plane
        .append_child("size")
        .set_value("500 500");

    auto visual = link.append_child("visual");
    visual.append_attribute("name") = "visual";
    auto plane_v = visual
        .append_child("geometry")
        .append_child("plane");
    plane_v
        .append_child("normal")
        .set_value("0 0 1");
    plane_v
        .append_child("size")
        .set_value("500 500");

    auto material = visual.append_child("material");
    auto script = material.append_child("script");
    script
        .append_child("uri")
        .set_value("file://media/materials/scripts/gazebo.material");
    script
        .append_child("name")
        .set_value("Gazebo/Grey");

    link.append_child("gravity").set_value("1");

    return world;
}


auto include_model(pugi::xml_node & world, const char* model_name) {
    std::string model("model://");
    model += model_name;
    auto include = world.append_child("include");
    include
        .append_child("uri")
        .set_value(model.c_str());
    return include;
}

void add_model(pugi::xml_node & world, const char* main_model, const char* model_name) {

    auto include = include_model(world, main_model);
    include
        .append_child("name")
        .set_value(model_name);
    include
        .append_child("static")
        .set_value("true");
    include
        .append_child("pose")
        .set_value("");
}

void add_road(pugi::xml_node & world, const char* road_name) {
    auto road = world.append_child("road");
    road.append_attribute("name") = road_name;
    auto script = road
        .append_child("material")
        .append_child("script");
    script
        .append_child("uri")
        .set_value("file://media/materials/scripts/gazebo.material");
    script
        .append_child("name")
        .set_value("Gazebo/Black");
}

void add_building(pugi::xml_node & world, const char* building_name) {

    auto building = world.append_child("model");
    building.append_attribute("name") = building_name;
    building
        .append_child("static")
        .set_value("true");
    auto pose = building.append_child("pose");

}




auto parse_way(uint64_t osm_way_id, const std::vector<std::uint64_t> & node_list, const TagMap&tags) {

    std::cout << "Tags for:" << osm_way_id << ":\n";

    for (auto tag : tags) {
        std::cout << "\t" << tag.key << ":" << tag.value << std::endl;
    }

    const char* highway = tags["highway"];
    if(highway) {
        auto typeHighway = highway_types.find(frozen::string(highway, strlen(highway)));
        if (typeHighway != highway_types.end()) {
            std::ostringstream rn;
            const char* road_name = tags["name"];
            if (road_name)
                rn << road_name << "_" << osm_way_id;
            else
                rn << typeHighway->first.data() << "_" << osm_way_id;


        }
    }

}

struct xml_string_writer: pugi::xml_writer {
    std::string result;

    void write(const void* data, size_t size) override {
        result.append(static_cast<const char*>(data), size);
    }
};


std::string osm::pbf_to_sdf(const std::string & pbf_file, const std::string & version) {

    pugi::xml_document doc;
    auto world = init_sdf(doc, version);

    std::unordered_map<uint64_t, std::pair<double, double>> nodes;
    std::unordered_map<uint64_t, OSMWay> ways;

    float min_latitude = 0.0, max_latitude = 0.0, min_longtitude = 0.0, max_longtitude = 0.0;

    ordered_read_osm_pbf(
            pbf_file,
            [&](uint64_t osm_node_id, double lat, double lon, const TagMap&tags) {
                nodes[osm_node_id] = { lat, lon };

                if (lat > max_latitude || max_latitude == 0) {
                    max_latitude = lat;
                } else if (lat < min_latitude || min_latitude == 0) {
                    min_latitude = lat;
                }

                if (lon > max_longtitude || max_longtitude == 0) {
                    max_longtitude = lon;
                } else if (lon < min_longtitude || min_longtitude == 0) {
                    min_longtitude = lon;
                }

                //std::cout << osm_node_id << std::endl;


            },
            [&](uint64_t osm_way_id, const std::vector<std::uint64_t> & node_list, const TagMap&tags) {
                parse_way(osm_way_id, node_list, tags);
            },
            nullptr
    );

    //std::cout << min_latitude << ":" << min_longtitude << "/" << max_latitude << ":" << max_longtitude << std::endl;

    xml_string_writer writer;
    doc.print(writer);
    return writer.result;

}