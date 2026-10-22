// Exercise the real Navigation service without DDS, ROS or motor commands.
#include "NavigationGoalService.h"
#include <iostream>
#include <chrono>
int main(int argc, char** argv) {
    YAML::Node cfg; cfg["endpoint"] = argv[1];
    NavigationGoalService service(cfg);
    service.activate_velocity();
    service.set_preview_tangent_scale(1.0f/3.0f);
    service.update_view_root({{"stamp",12.5},{"source","simulator"},{"position",{1,2,3}}});
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    while(std::chrono::steady_clock::now()<end) {
        NavigationGoalService::Goal goal;
        bool got=false;
        if (service.transition_ready(NavigationGoalService::Mode::Velocity)) got=service.start_navigation(goal);
        else got=service.consume_navigation_goal(goal);
        if (got) {
            service.update_view_path({{"goal_id", std::to_string(goal.id)}, {"poses", {{0,0,0},{goal.x,goal.y,goal.yaw}}}});
            // Keep the goal live long enough to exercise RUNNING and supersession.
            if (goal.x == 9) service.leave_navigation();
            else if (goal.x == 8) service.finish_goal(goal.id, false, "test rejection");
            else if (goal.x != 7) service.finish_goal(goal.id, true);
            // A delayed report for an older goal must never finish this one.
            service.finish_goal(goal.id - 1, true);
            std::cout << goal.clock_domain << ' ' << goal.observation_stamp << ' '
                           << goal.x << ' ' << goal.y << ' ' << goal.yaw << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
