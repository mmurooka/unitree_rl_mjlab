#pragma once

#include <atomic>
#include <future>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

// Receives robot-relative Navigation goals independently of policy states.
class NavigationGoalService
{
public:
    struct Goal {
        float x{0.0f};
        float y{0.0f};
        float yaw{0.0f};
        uint64_t id{0};
        double observation_stamp{-1.0};
        std::string clock_domain;
    };
    enum class Mode { Inactive, Velocity, Navigation, Mimic };

    explicit NavigationGoalService(const YAML::Node& cfg);
    ~NavigationGoalService();

    void activate_velocity();
    void leave_velocity();
    void leave_navigation();
    void activate_mimic();
    void set_mimic_busy();
    void set_mimic_ready();
    void leave_mimic();
    void deactivate();

    bool transition_ready(Mode mode);
    bool start_navigation(Goal& goal);
    bool consume_navigation_goal(Goal& goal);
    // Terminal results are latched for the latest accepted goal.
    void finish_goal(uint64_t id, bool reached, const std::string& reason = "");

private:
    struct Request {
        Goal goal;
        std::promise<std::string> result;
        bool claimed{false};
    };

    void activate_source(Mode mode, bool accepting);
    void leave_source(Mode mode, const std::string& name);
    void cancel_locked(const std::string& reason);
    std::string status_locked() const;
    void accept_locked(Goal& goal);
    void receive();

    std::mutex mutex_;
    Mode mode_{Mode::Inactive};
    uint64_t next_id_;
    uint64_t active_id_{0};
    std::string progress_{"IDLE"};
    std::string progress_reason_;
    bool accepting_{false};
    bool transitioning_{false};
    std::shared_ptr<Request> request_;
    std::atomic<bool> receiving_{true};
    zmq::context_t context_{1};
    zmq::socket_t socket_{context_, zmq::socket_type::rep};
    std::thread thread_;
};

extern std::shared_ptr<NavigationGoalService> navigation_goal_service;
