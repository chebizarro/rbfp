// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_SDF_H
#define RBPF_SDF_H

#include <string>
#include <vector>
#include <pugixml.hpp>
#include <map>

namespace osm {

    struct Point {
        float x;
        float y;
        float z;
    };

    class Sdf {

    public:
        explicit Sdf(const char *, const char* = "1.6");

        std::string to_string() noexcept;

        pugi::xml_document & to_xml() noexcept;


    private:
        void add_building(const char *, std::vector<Point> &, const char *);

        void add_ground();

        void add_model(const char *, const char *);

        void add_physics(const char * = "default_physics", int = 0, const char * = "ode",
                         float = 0.001, float = 1, float = 1000);

        void add_road(const char *, float);

        void add_road_point(pugi::xml_node &, std::vector<Point> &);

        void add_spherical_coords(float, float, float= .0f, float= .0f);

        void add_world();

        pugi::xml_node include_model(const char *);

        pugi::xml_document doc;
        pugi::xml_node world;
        std::vector<float> latitude;
        std::vector<float> longtitude;
        std::map<uint64_t, int> id_map;

    };

}

#endif //RBPF_SDF_H
