#include "NavigationGoalService.h"

#include <cmath>
#include <sstream>

std::shared_ptr<NavigationGoalService> navigation_goal_service;

NavigationGoalService::NavigationGoalService(const YAML::Node& cfg)
{
    socket_.set(zmq::sockopt::linger, 0);
    socket_.set(zmq::sockopt::rcvtimeo, 100);
    socket_.set(zmq::sockopt::sndtimeo, 100);
    socket_.set(zmq::sockopt::maxmsgsize, int64_t(256));
    socket_.bind(cfg["endpoint"].as<std::string>(
        "ipc:///tmp/task_prompt_rl_navigation.sock"));
    thread_ = std::thread(&NavigationGoalService::receive, this);
}

NavigationGoalService::~NavigationGoalService()
{
    receiving_ = false;
    deactivate();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void NavigationGoalService::cancel_locked(const std::string& reason)
{
    if (request_) {
        request_->result.set_value(reason);
        request_.reset();
    }
    transitioning_ = false;
}

void NavigationGoalService::activate_source(Mode mode, bool accepting)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_locked("ERROR controller entered another goal-receiving mode");
    mode_ = mode;
    accepting_ = accepting;
}

void NavigationGoalService::leave_source(Mode mode, const std::string& name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != mode) {
        return;
    }
    // Keep a claimed goal across the source exit / Navigation entry boundary.
    if (transitioning_) {
        return;
    }
    cancel_locked("ERROR controller left " + name);
    mode_ = Mode::Inactive;
    accepting_ = false;
}

void NavigationGoalService::activate_velocity()
{
    activate_source(Mode::Velocity, true);
}

void NavigationGoalService::leave_velocity()
{
    leave_source(Mode::Velocity, "Velocity");
}

void NavigationGoalService::leave_navigation()
{
    leave_source(Mode::Navigation, "Navigation");
}

void NavigationGoalService::activate_mimic()
{
    activate_source(Mode::Mimic, false);
}

void NavigationGoalService::set_mimic_busy()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != Mode::Mimic) {
        return;
    }
    cancel_locked("BUSY");
    accepting_ = false;
}

void NavigationGoalService::set_mimic_ready()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::Mimic && !request_) {
        accepting_ = true;
    }
}

void NavigationGoalService::leave_mimic()
{
    leave_source(Mode::Mimic, "OnlineMimic");
}

void NavigationGoalService::deactivate()
{
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_locked("ERROR navigation goal reception is inactive");
    mode_ = Mode::Inactive;
    accepting_ = false;
}

bool NavigationGoalService::transition_ready(Mode mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != mode || !accepting_ || !request_ || request_->claimed) {
        return false;
    }
    request_->claimed = true;
    transitioning_ = true;
    accepting_ = false;
    return true;
}

bool NavigationGoalService::start_navigation(Goal& goal)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const bool has_goal = transitioning_ && request_ && request_->claimed;
    if (has_goal) {
        goal = request_->goal;
        request_->result.set_value("STARTED");
        request_.reset();
    } else {
        cancel_locked("ERROR Navigation entered without the requested goal");
    }
    transitioning_ = false;
    mode_ = Mode::Navigation;
    accepting_ = true;
    return has_goal;
}

bool NavigationGoalService::consume_navigation_goal(Goal& goal)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != Mode::Navigation || !accepting_ || !request_) {
        return false;
    }
    goal = request_->goal;
    request_->result.set_value("STARTED");
    request_.reset();
    return true;
}

std::string NavigationGoalService::status_locked() const
{
    if (mode_ == Mode::Inactive) {
        return "ERROR navigation goal reception is inactive";
    }
    return accepting_ && !request_ ? "READY" : "BUSY";
}

void NavigationGoalService::receive()
{
    while (receiving_) {
        zmq::message_t message;
        if (!socket_.recv(message, zmq::recv_flags::none)) {
            continue;
        }

        std::string reply;
        const std::string command = message.to_string();
        if (command == "STATUS") {
            std::lock_guard<std::mutex> lock(mutex_);
            reply = status_locked();
        } else {
            std::istringstream stream(command);
            std::string verb;
            std::string extra;
            Goal goal;
            const bool valid =
                bool(stream >> verb >> goal.x >> goal.y >> goal.yaw) &&
                !(stream >> extra) &&
                verb == "GOAL" &&
                std::isfinite(goal.x) &&
                std::isfinite(goal.y) &&
                std::isfinite(goal.yaw);
            if (!valid) {
                reply = "ERROR expected GOAL x y yaw with finite values";
            } else {
                std::shared_ptr<Request> request;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    reply = status_locked();
                    if (reply == "READY") {
                        request = std::make_shared<Request>();
                        request->goal = goal;
                        request_ = request;
                    }
                }
                if (request) {
                    reply = request->result.get_future().get();
                }
            }
        }
        socket_.send(zmq::buffer(reply), zmq::send_flags::none);
    }
}
