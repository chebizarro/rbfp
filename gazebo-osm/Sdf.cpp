// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include "Sdf.h"
#include <sstream>
#include <routingkit/osm_decoder.h>

using namespace osm;
using namespace RoutingKit;

struct xml_string_writer: pugi::xml_writer {
    std::string result;

    void write(const void* data, size_t size) override {
        result.append(static_cast<const char*>(data), size);
    }
};

Sdf::Sdf(const char *pbf_file, const char *version) {

    add_world();

    ordered_read_osm_pbf(
            pbf_file,
            [&](uint64_t osm_node_id, double lat, double lon, const TagMap&tags) {
                id_map[osm_node_id] = latitude.size();
                latitude.push_back(lat);
                longtitude.push_back(lon);
            },
            [&](uint64_t osm_way_id, const std::vector<std::uint64_t> & node_list, const TagMap&tags) {

            },
            nullptr
    );

    add_physics();
    add_ground();
    include_model("sun");
    // add buildings
    // add roads
    // add spherical coordinates
}

void Sdf::add_building(const char *building_name, std::vector<osm::Point> &points, const char *color) {
    auto building = world.append_child("model");
    building.append_attribute("name") = building_name;
    building.append_child("static").set_value("true");

    building
        .append_child("pose")
        .set_value("");


}

void Sdf::add_ground() {
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

    auto script = visual
        .append_child("material")
        .append_child("script");
    script
        .append_child("uri")
        .set_value("file://media/materials/scripts/gazebo.material");
    script
        .append_child("name")
        .set_value("Gazebo/Grey");

    link
        .append_child("gravity")
        .set_value("1");
}

void Sdf::add_model(const char* main_model, const char* model_name) {

    auto include = include_model(main_model);
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

void Sdf::add_physics(const char* name, int def, const char* type, float step_size, float real_time, float update_factor) {
    auto physics = world.append_child("physics");
    physics.append_attribute("name") = name;
    physics.append_attribute("default") = "0";
    physics.append_attribute("type") = type;
    physics
        .append_child("max_step_size")
        .set_value("0.001");
    physics
        .append_child("real_time_factor")
        .set_value("1");
    physics
        .append_child("real_time_update_rate")
        .set_value("1000");
}

void Sdf::add_road(const char* road_name, float width) {
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
    road
        .append_child("width")
        .set_value(std::to_string(width).c_str());
}

void Sdf::add_road_point(pugi::xml_node & road, std::vector<osm::Point> &points) {
    road
        .append_child("point")
        .set_value(" ");
}

void Sdf::add_spherical_coords(float lat, float lon, float elevation, float heading) {
    auto spherical = world.append_child("spherical_coordinates");
    spherical
        .append_child("surface_model")
        .set_value("EARTH_WGS84");
    spherical
        .append_child("latitude_deg")
        .set_value(std::to_string(lat).c_str());
    spherical
        .append_child("longitude_deg")
        .set_value(std::to_string(lon).c_str());
    spherical
        .append_child("elevation")
        .set_value(std::to_string(elevation).c_str());
    spherical
        .append_child("heading_deg")
        .set_value(std::to_string(heading).c_str());
}

void Sdf::add_world() {
    auto root = doc.append_child("sdf");
    root.append_attribute("version") = version;
    world = root.append_child("world");
    world.append_attribute("name") = "default";
}

pugi::xml_node Sdf::include_model(const char* model_name) {
    std::string model("model://");
    model += model_name;
    auto include = world.append_child("include");
    include
        .append_child("uri")
        .set_value(model.c_str());
    return include;
}

std::string Sdf::to_string() noexcept {
    xml_string_writer writer;
    doc.print(writer);
    return writer.result;
}

pugi::xml_document& Sdf::to_xml() noexcept {
    return doc;
}