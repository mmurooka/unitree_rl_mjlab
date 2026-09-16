#pragma once

#include "State_Mimic.h"
#include <atomic>
#include <future>
#include <mutex>
#include <zmq.hpp>

class State_OnlineMimic : public FSMState
{
public:
    State_OnlineMimic(int state_mode, std::string state_string);
    ~State_OnlineMimic();
    void enter() override;
    void run() override;
    void exit() override;

private:
    using Loader = State_Mimic::MotionLoader_;
    struct Request {
        std::shared_ptr<Loader> motion;
        std::promise<std::string> result;
    };
    enum class Status { Inactive, Ready, Busy };
    void receive();
    void control();
    void publish_action(isaaclab::ManagerBasedRLEnv* env);
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> tracking_, standing_;
    std::atomic<Status> status_{Status::Inactive};
    std::atomic<bool> running_{false}, receiving_{true}, bad_orientation_{false};
    std::thread control_thread_, receiver_thread_;
    zmq::context_t context_{1};
    zmq::socket_t socket_{context_, zmq::socket_type::rep};
    std::mutex request_mutex_, action_mutex_;
    std::shared_ptr<Request> pending_;
    std::vector<float> target_, kp_, kd_;
    float threshold_, hold_seconds_;
};

REGISTER_FSM(State_OnlineMimic)
