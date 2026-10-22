#include "NavigationGoalService.h"
#include "../../../../common/navigation_curve.h"

#include <cmath>
#include <chrono>
#include <sstream>

std::shared_ptr<NavigationGoalService> navigation_goal_service;

void NavigationGoalService::set_preview_tangent_scale(float scale)
{
    if (!std::isfinite(scale) || scale <= 0) throw std::runtime_error("Invalid preview tangent scale");
    std::lock_guard<std::mutex> lock(mutex_);
    preview_tangent_scale_ = scale;
}

void NavigationGoalService::update_view_root(nlohmann::json root)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (view_root_.is_null() || view_root_["stamp"] != root["stamp"]) {
        view_root_ = std::move(root);
        view_root_received_ = std::chrono::steady_clock::now();
    }
}

void NavigationGoalService::update_view_path(nlohmann::json path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    view_path_ = std::move(path);
}

nlohmann::json NavigationGoalService::view_root()
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto root = view_root_;
    if (!root.is_null()) root["age_s"] = std::chrono::duration<double>(
        std::chrono::steady_clock::now()-view_root_received_).count();
    return root;
}

NavigationGoalService::NavigationGoalService(const YAML::Node& cfg)
    : next_id_(std::chrono::steady_clock::now().time_since_epoch().count())
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
    if (progress_ == "RUNNING") {
        progress_ = "CANCELLED";
        progress_reason_ = reason;
    }
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
        accept_locked(goal);
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
    accept_locked(goal);
    return true;
}

void NavigationGoalService::accept_locked(Goal& goal)
{
    goal = request_->goal;
    goal.id = ++next_id_;
    active_id_ = goal.id;
    view_path_ = nullptr;
    progress_ = "RUNNING";
    progress_reason_.clear();
    request_->result.set_value("STARTED " + std::to_string(goal.id));
    request_.reset();
}

void NavigationGoalService::finish_goal(uint64_t id, bool reached, const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (id != active_id_ || progress_ != "RUNNING") return;
    progress_ = reached ? "REACHED" : "ERROR";
    progress_reason_ = reason;
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
        if (command.rfind("PLAN ", 0) == 0) {
            float x, y, yaw;
            std::string extra;
            std::istringstream input(command.substr(5));
            if (!(input >> x >> y >> yaw) || (input >> extra) ||
                !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
                reply = "ERROR expected PLAN x y yaw with finite values";
            } else {
                std::lock_guard<std::mutex> lock(mutex_);
                if (preview_tangent_scale_ <= 0) reply = "ERROR Navigation planner is not initialized";
                else {
                    nlohmann::json poses=nlohmann::json::array();
                    for (int i=0;i<=50;++i) {
                        const auto [p,heading]=g1::navigation_curve_point(
                            Eigen::Vector2f(x,y),yaw,preview_tangent_scale_,i/50.0f);
                        poses.push_back({p.x(),p.y(),heading});
                    }
                    reply=nlohmann::json{{"schema",1},{"poses",poses},
                        {"tangent_scale",preview_tangent_scale_},{"reference_frame","root_yaw"}}.dump();
                }
            }
        } else if (command == "VIEW") {
            std::lock_guard<std::mutex> lock(mutex_);
            auto root = view_root_;
            if (!root.is_null()) root["age_s"] = std::chrono::duration<double>(
                std::chrono::steady_clock::now()-view_root_received_).count();
            reply = nlohmann::json{{"schema", 1}, {"root", root}, {"path", view_path_},
                {"goal_id", std::to_string(active_id_)}, {"progress", progress_},
                {"reason", progress_reason_}, {"status", status_locked()}}.dump();
        } else if (command == "STATUS") {
            std::lock_guard<std::mutex> lock(mutex_);
            reply = status_locked();
        } else if (command == "PROGRESS") {
            std::lock_guard<std::mutex> lock(mutex_);
            reply = progress_ + " " + std::to_string(active_id_);
            if (!progress_reason_.empty()) reply += " " + progress_reason_;
        } else {
            std::istringstream stream(command);
            std::string verb;
            std::string extra;
            Goal goal;
            bool valid = false;
            stream >> verb;
            if (verb == "GOAL" || verb == "GOAL_AT") {
                if (verb == "GOAL_AT") {
                    stream >> goal.clock_domain >> goal.observation_stamp;
                    if ((goal.clock_domain != "simulation" && goal.clock_domain != "ros") ||
                        !std::isfinite(goal.observation_stamp) || goal.observation_stamp < 0)
                        stream.setstate(std::ios::failbit);
                }
                valid =
                    bool(stream >> goal.x >> goal.y >> goal.yaw) &&
                    !(stream >> extra) &&
                    std::isfinite(goal.x) &&
                    std::isfinite(goal.y) &&
                    std::isfinite(goal.yaw);
            }
            if (!valid) {
                reply = "ERROR expected GOAL x y yaw, GOAL_AT clock stamp x y yaw "
                        "with finite values";
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
