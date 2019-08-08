// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/osm_graph_builder.h>
#include <routingkit/id_mapper.h>
#include <routingkit/osm_profile.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <fstream>

using namespace RoutingKit;
using namespace std;

inline auto lat2m(float lat) {
    auto y = logf(tanf((90 + lat) * M_PI / 360)) / (M_PI / 180);
    y = y * 20037508.34 / 180;
    return y;
}

inline auto lon2m(float lon) {
    auto x = lon * 20037508.34 / 180;
    return x;
}

inline auto deg2m(float lat, float lon) {
    auto x = lon2m(lon);
    auto y = lat2m(lat);
    return std::pair<float, float>(x, y);
}

int main(int argc, char*argv[]) {

    auto pbf_file = "/home/bizarro/Downloads/tesc_export.pbf";

    auto mapping = load_osm_id_mapping_from_pbf(
            pbf_file,
            nullptr,
            [&](uint64_t osm_way_id, const TagMap&tags){
                return is_osm_way_used_by_pedestrians(osm_way_id, tags, nullptr);
            },
            nullptr,
            true
    );

    auto routing_graph = load_osm_routing_graph_from_pbf(
            pbf_file,
            mapping,
            [&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&way_tags){
                return OSMWayDirectionCategory::open_in_both;
            },
            nullptr,
            nullptr
    );

    IDMapper routing_node_mapper(mapping.is_routing_node);

    auto tail = invert_inverse_vector(routing_graph.first_out);

    // Build the shortest path index
    auto ch = ContractionHierarchy::build(
            routing_graph.first_out.size()-1,
            tail, routing_graph.head,
            routing_graph.geo_distance
    );

    // Build the index to quickly map latitudes and longitudes
    GeoPositionToNode map_geo_position(routing_graph.latitude, routing_graph.longitude);

    // Besides the CH itself we need a query object.
    ContractionHierarchyQuery ch_query(ch);

    // Use the query object to answer queries from stdin to stdout
    float from_latitude = 47.0715024;
    float from_longitude = -122.9754289;
    float to_latitude = 47.0733177;
    float to_longitude = -122.9775811;

    unsigned from = map_geo_position.find_nearest_neighbor_within_radius(from_latitude, from_longitude, 30).id;
    if(from == invalid_id){
        cout << "No node within 30m from source position" << endl;
    }
    unsigned to = map_geo_position.find_nearest_neighbor_within_radius(to_latitude, to_longitude, 30).id;
    if(to == invalid_id){
        cout << "No node within 30m from target position" << endl;
    }

    long long start_time = get_micro_time();
    ch_query.reset().add_source(from).add_target(to).run();
    auto distance = ch_query.get_distance();
    auto path = ch_query.get_node_path();
    long long end_time = get_micro_time();

    cout << "To get from "<< from << " to "<< to << " one needs " << distance << " milliseconds." << endl;
    cout << "This query was answered in " << (end_time - start_time) << " microseconds." << endl;
    cout << "The path is";

    auto first_out_arc = routing_graph.first_out;
    auto head = routing_graph.head;
    auto latitude = routing_graph.latitude;
    auto longitude = routing_graph.longitude;

    for(auto x:path) {

        auto [l, t] = deg2m(latitude[x], longitude[x]);

        cout << " " << routing_node_mapper.to_global(x);
        cout << ":" << l << ", " << t;
    }
    cout << endl;

    float min_latitude = lat2m(min_element_of(latitude));
    float max_latitude = lat2m(max_element_of(latitude));

    for(auto&y:latitude){
        y = lat2m(y);
        y -= min_latitude;
        y *= 800;
        y /= (max_latitude - min_latitude);
    }

    float min_longitude = lon2m(min_element_of(longitude));
    float max_longitude = lon2m(max_element_of(longitude));

    for(auto&x:longitude){
        x = lon2m(x);
        x -= min_longitude;
        x *= 800;
        x /= (max_longitude - min_longitude);
    }

    string svg_file = "test.svg";

    std::ofstream out(svg_file);
    if(!out)
        throw std::runtime_error("Can not open file " + svg_file);

    out <<
        "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 800 800\">\n";
    for(unsigned x=0; x<first_out_arc.size()-1; ++x){
        for(unsigned xy=first_out_arc[x]; xy<first_out_arc[x+1]; ++xy){
            unsigned y=head[xy];

            out << "<line "
                << "x1=\"" << longitude[x] << "\" y1=\"" << 800-latitude[x] << "\" "
                << "x2=\"" << longitude[y] << "\" y2=\""<< 800-latitude[y] <<"\" style=\"stroke:#000000;\"/>\n";
        }
    }
    for(auto x:path) {
        out << "<circle "
            << "cx = \"" << longitude[x] << "\" "
            << "cy = \"" << 800 - latitude[x] << "\" "
            << "r = \"1\" style=\"stroke:#FF0000;\"/>";
    }
    out << "</svg>\n";

    return 0;
}

int whatevs() {

    // Load a car routing graph from OpenStreetMap-based data
    auto graph = simple_load_osm_pedestrian_routing_graph_from_pbf("/home/bizarro/Downloads/tesc_export.pbf");
    auto tail = invert_inverse_vector(graph.first_out);


    auto mapping = load_osm_id_mapping_from_pbf(
            "/home/bizarro/Downloads/tesc_export.pbf",
            nullptr,
            [&](uint64_t osm_way_id, const TagMap&tags){
                return is_osm_way_used_by_pedestrians(osm_way_id, tags, nullptr);
            },
            nullptr,
            true
    );
    IDMapper routing_node_mapper(mapping.is_routing_node);

    // Build the shortest path index
    auto ch = ContractionHierarchy::build(
            graph.node_count(),
            tail, graph.head,
            graph.geo_distance
    );

    // Build the index to quickly map latitudes and longitudes
    GeoPositionToNode map_geo_position(graph.latitude, graph.longitude);

    // Besides the CH itself we need a query object.
    ContractionHierarchyQuery ch_query(ch);

    // Use the query object to answer queries from stdin to stdout
    float from_latitude, from_longitude, to_latitude, to_longitude;
    while(cin >> from_latitude >> from_longitude >> to_latitude >> to_longitude){
        unsigned from = map_geo_position.find_nearest_neighbor_within_radius(from_latitude, from_longitude, 30).id;
        if(from == invalid_id){
            cout << "No node within 30m from source position" << endl;
            continue;
        }
        unsigned to = map_geo_position.find_nearest_neighbor_within_radius(to_latitude, to_longitude, 30).id;
        if(to == invalid_id){
            cout << "No node within 30m from target position" << endl;
            continue;
        }

        long long start_time = get_micro_time();
        ch_query.reset().add_source(from).add_target(to).run();
        auto distance = ch_query.get_distance();
        auto path = ch_query.get_node_path();
        long long end_time = get_micro_time();


        cout << "To get from "<< from << " to "<< to << " one needs " << distance << " milliseconds." << endl;
        cout << "This query was answered in " << (end_time - start_time) << " microseconds." << endl;
        cout << "The path is";
        for(auto x:path)
            cout << " " << routing_node_mapper.to_global(x);
        cout << endl;


    }
}