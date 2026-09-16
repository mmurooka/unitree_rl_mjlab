#pragma once

#include "OnlineMotionService.h"
#include <atomic>
#include <mutex>

class State_OnlineMimic : public FSMState
{
public:
    State_OnlineMimic(int state_mode, std::string state_string);
    ~State_OnlineMimic();
    void enter() override;
    void run() override;
    void exit() override;

private:
    using Loader = OnlineMotionService::Loader;
    bool start_motion(const std::shared_ptr<OnlineMotionService::Request>& request);
    void control();
    void publish_action();
    std::shared_ptr<OnlineMotionService> service_;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> tracking_;
    std::shared_ptr<Loader> playing_;
    size_t tick_ = 0;
    bool holding_ = false;
    std::atomic<bool> running_{false}, bad_orientation_{false}, return_to_velocity_{false};
    std::thread control_thread_;
    std::mutex action_mutex_;
    std::vector<float> target_, kp_, kd_;
};

REGISTER_FSM(State_OnlineMimic)
