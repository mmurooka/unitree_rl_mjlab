// Exercise the real Navigation service without DDS, ROS or motor commands.
#include "NavigationGoalService.h"
#include <iostream>
#include <chrono>
int main(int argc, char** argv) {
    YAML::Node cfg; cfg["endpoint"] = argv[1];
    NavigationGoalService service(cfg);
    service.activate_velocity();
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    while(std::chrono::steady_clock::now()<end) {
        NavigationGoalService::Goal goal;
        bool got=false;
        if (service.transition_ready(NavigationGoalService::Mode::Velocity)) got=service.start_navigation(goal);
        else got=service.consume_navigation_goal(goal);
        if (got) std::cout << goal.clock_domain << ' ' << goal.observation_stamp << ' '
                           << goal.x << ' ' << goal.y << ' ' << goal.yaw << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
