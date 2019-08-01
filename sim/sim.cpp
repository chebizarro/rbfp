// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <nlohmann/json.hpp>
#include <rxcpp/rx.hpp>
#include <fstream>

using json = nlohmann::json;
using namespace rxcpp;
using namespace rxcpp::rxo;
using namespace rxcpp::rxs;


int main(int argc, char*argv[]) {

    // Load JSON Config file
    std::ifstream i("file.json");
    json config;
    i >> config;


    schedulers::run_loop rl;

    auto mainthread = observe_on_run_loop(rl);

    composite_subscription lifetime;


    // main loop
    while(lifetime.is_subscribed()) {

        while (!rl.empty() && rl.peek().when < rl.now()) {
            rl.dispatch();
        }

    }

    return 0;


}