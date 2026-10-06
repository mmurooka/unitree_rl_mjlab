#pragma once

#include "State_Mimic.h"
#include <atomic>
#include <future>
#include <mutex>
#include <zmq.hpp>

// One receiver shared by Velocity, Navigation, and OnlineMimic; it never runs
// a policy.
class OnlineMotionService
{
public:
    using Loader = State_Mimic::MotionLoader_;
    enum class Mode { Inactive, Velocity, Navigation, Mimic };
    struct Request {
        std::shared_ptr<Loader> motion;
        std::promise<std::string> result;
        bool claimed = false;
    };
    explicit OnlineMotionService(const YAML::Node& cfg);
    ~OnlineMotionService();
    void activate_velocity();
    void leave_velocity();
    void activate_navigation();
    void leave_navigation();
    std::shared_ptr<Request> activate_mimic(Mode& source_mode);
    void deactivate();
    std::shared_ptr<Request> claim(Mode mode);
    bool prepare_transition(const std::shared_ptr<Request>& request);
    bool transition_ready(Mode mode, const Eigen::VectorXf& actual);
    bool started(const std::shared_ptr<Request>& request);
    void reject(const std::shared_ptr<Request>& request, const std::string& reason);
    void finished(const Eigen::VectorXf& final_joint_pose);
    bool consume_completed_navigation_motion(Eigen::VectorXf& final_joint_pose);
    std::string start_error(const Request& request, const Eigen::VectorXf& actual) const;

private:
    void activate_source(Mode mode);
    void leave_source(Mode mode, const std::string& name);
    void receive();
    void cancel_locked(const std::string& reason);
    std::string status_locked() const;
    float threshold_;
    std::mutex mutex_;
    Mode mode_ = Mode::Inactive;
    Mode transition_source_ = Mode::Inactive;
    Mode mimic_source_ = Mode::Inactive;
    bool completed_navigation_motion_ = false;
    Eigen::VectorXf completed_navigation_joint_pose_;
    bool accepting_ = false, transitioning_ = false;
    std::shared_ptr<Request> request_;
    std::atomic<bool> receiving_{true};
    zmq::context_t context_{1};
    zmq::socket_t socket_{context_, zmq::socket_type::rep};
    std::thread thread_;
};

extern std::shared_ptr<OnlineMotionService> online_motion_service;
