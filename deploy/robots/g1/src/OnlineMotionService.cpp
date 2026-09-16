#include "OnlineMotionService.h"

std::shared_ptr<OnlineMotionService> online_motion_service;

OnlineMotionService::OnlineMotionService(const YAML::Node& cfg)
{
    threshold_ = cfg["start_joint_threshold_degrees"].as<float>(30.0f) *
                 3.14159265358979323846f / 180.0f;
    if (!std::isfinite(threshold_) || threshold_ <= 0) {
        throw std::runtime_error("Invalid online motion start joint threshold");
    }
    socket_.set(zmq::sockopt::linger, 0);
    socket_.set(zmq::sockopt::rcvtimeo, 100);
    socket_.set(zmq::sockopt::sndtimeo, 100);
    socket_.set(zmq::sockopt::maxmsgsize, int64_t(8192));
    socket_.bind(cfg["endpoint"].as<std::string>("ipc:///tmp/task_prompt_rl_deploy.sock"));
    thread_ = std::thread(&OnlineMotionService::receive, this);
}

OnlineMotionService::~OnlineMotionService()
{
    receiving_ = false;
    deactivate();
    if (thread_.joinable()) thread_.join();
}

void OnlineMotionService::cancel_locked(const std::string& reason)
{
    if (request_) {
        request_->result.set_value(reason);
        request_.reset();
    }
    transitioning_ = false;
}

void OnlineMotionService::activate_velocity()
{
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_locked("ERROR controller returned to Velocity");
    mode_ = Mode::Velocity;
    accepting_ = true;
}

void OnlineMotionService::leave_velocity()
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Preserve a validated request across the normal exit()/enter() transition.
    if (transitioning_) return;
    cancel_locked("ERROR controller left Velocity");
    mode_ = Mode::Inactive;
    accepting_ = false;
}

std::shared_ptr<OnlineMotionService::Request> OnlineMotionService::activate_mimic()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transitioning_ || !request_) return nullptr;
    transitioning_ = false;
    mode_ = Mode::Mimic;
    accepting_ = false;
    return request_;
}

void OnlineMotionService::deactivate()
{
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_locked("ERROR online motion reception is inactive");
    mode_ = Mode::Inactive;
    accepting_ = false;
}

std::shared_ptr<OnlineMotionService::Request> OnlineMotionService::claim(Mode mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != mode || !request_ || !request_->motion || request_->claimed) return nullptr;
    request_->claimed = true;
    return request_;
}

bool OnlineMotionService::prepare_transition(const std::shared_ptr<Request>& request)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != Mode::Velocity || request_ != request) return false;
    transitioning_ = true;
    return true;
}

bool OnlineMotionService::started(const std::shared_ptr<Request>& request)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != Mode::Mimic || request_ != request) return false;
    request_->result.set_value("STARTED");
    request_.reset();
    accepting_ = false;
    return true;
}

void OnlineMotionService::reject(const std::shared_ptr<Request>& request, const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (request_ != request) return;
    cancel_locked(reason);
    accepting_ = mode_ != Mode::Inactive;
}

void OnlineMotionService::finished()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::Mimic && !request_) accepting_ = true;
}

std::string OnlineMotionService::start_error(const Request& request, const Eigen::VectorXf& actual) const
{
    const auto reference = request.motion->joint_pos();
    if (actual.size() != reference.size() || !actual.allFinite()) {
        return "ERROR invalid measured joint positions";
    }
    Eigen::Index joint = 0;
    const float difference = (actual - reference).cwiseAbs().maxCoeff(&joint);
    if (difference > threshold_) {
        return fmt::format("ERROR start joint {} differs by {:.2f} deg (limit {:.2f} deg)",
            joint, difference * 180.0f / 3.14159265358979323846f,
            threshold_ * 180.0f / 3.14159265358979323846f);
    }
    return {};
}

std::string OnlineMotionService::status_locked() const
{
    if (mode_ == Mode::Inactive) return "ERROR online motion reception is inactive";
    return accepting_ && !request_ ? "READY" : "BUSY";
}

void OnlineMotionService::receive()
{
    while (receiving_) {
        zmq::message_t message;
        if (!socket_.recv(message, zmq::recv_flags::none)) continue;
        std::string reply;
        const std::string command = message.to_string();
        if (command == "STATUS") {
            std::lock_guard<std::mutex> lock(mutex_);
            reply = status_locked();
        } else if (command.rfind("LOAD\n", 0) == 0) {
            std::shared_ptr<Request> request;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                reply = status_locked();
                if (reply == "READY") {
                    request = std::make_shared<Request>();
                    request_ = request;
                }
            }
            if (request) {
                auto result = request->result.get_future();
                try {
                    auto motion = std::make_shared<Loader>(command.substr(5));
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        // A state exit during file loading cancels, rather than revives, the request.
                        if (request_ == request) request->motion = std::move(motion);
                    }
                } catch (const std::exception& error) {
                    reject(request, std::string("ERROR ") + error.what());
                }
                // The current state acknowledges start/rejection; exit cancels outstanding work.
                reply = result.get();
            }
        } else {
            reply = "ERROR expected STATUS or LOAD";
        }
        socket_.send(zmq::buffer(reply), zmq::send_flags::none);
    }
}
