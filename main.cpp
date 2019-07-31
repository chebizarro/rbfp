#include <iostream>
#include <thread>

#include <rxcpp/rx.hpp>
using namespace rxcpp;
using namespace rxcpp::rxo;
using namespace rxcpp::rxs;

#include "State.h"

schedulers::run_loop rl;

using Reducer = std::function<State(State&)>;

int main() {

    auto mainthread = observe_on_run_loop(rl);

    composite_subscription lifetime;

    rxsub::replay<State::Activity, decltype(mainthread)> activity_change(1, mainthread, lifetime);

    auto activity_updates = activity_change.get_observable();
    auto send_activity_update = activity_change.get_subscriber();

    auto update_activity = [send_activity_update](State::Activity s){
        if (send_activity_update.is_subscribed()){
            send_activity_update.on_next(s);
        }
    };

    update_activity(State::Activity::walking);

    std::vector<observable<Reducer>> reducers;

    reducers.push_back(
        activity_updates |
        filter([](State::Activity s) { return s == State::Activity::walking; }) |
        rxo::map([&](bool) -> observable<Reducer> {
            return [](State& s) { return std::move(s) };
        })
    );


    auto actions = iterate(reducers) |  merge(mainthread);

    auto models = actions |
            scan(State{}, [=](State& m, Reducer& f){
                try {
                    auto r = f(m);
                    r.data->timestamp = mainthread.now();
                    return r;
                } catch (const std::exception& e) {
                    std::cerr << e.what() << std::endl;
                    return std::move(m);
                }
            }) |
            // only view state updates every 200ms
            sample_with_time(std::chrono::milliseconds(200), mainthread) |
            publish() |
            ref_count() |
            as_dynamic();


    // main loop
    while(lifetime.is_subscribed()) {

        while (!rl.empty() && rl.peek().when < rl.now()) {
            rl.dispatch();
        }

    }

    return 0;
}