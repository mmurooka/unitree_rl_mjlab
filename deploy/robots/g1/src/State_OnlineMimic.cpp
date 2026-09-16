#include "State_OnlineMimic.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include <chrono>

State_OnlineMimic::State_OnlineMimic(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string), service_(online_motion_service)
{
    if (!service_) throw std::runtime_error("Online motion service has not been initialized");
    const auto cfg = param::config["FSM"][state_string];
    State_Mimic::motion = std::make_shared<Loader>();
    auto path = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    tracking_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(path / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(lowstate));
    if (std::abs(tracking_->step_dt - 0.02f) > 1e-6f || tracking_->robot->data.joint_ids_map.size() != 29) {
        throw std::runtime_error("Online G1 policy must use 29 joints and step_dt=0.02");
    }
    for (size_t i = 0; i < 29; ++i) {
        if (tracking_->robot->data.joint_ids_map[i] != i) {
            throw std::runtime_error("Online G1 policy requires the existing motor joint order");
        }
    }
    tracking_->alg = std::make_unique<isaaclab::OrtRunner>(path / "exported" / "policy.onnx");
    registered_checks.emplace_back([this] { return bad_orientation_.load(); },
                                   FSMStringMap.right.at("Passive"));
    registered_checks.emplace_back([this] { return return_to_velocity_.load(); },
                                   FSMStringMap.right.at("Velocity"));
}

State_OnlineMimic::~State_OnlineMimic()
{
    exit();
}

bool State_OnlineMimic::start_motion(const std::shared_ptr<OnlineMotionService::Request>& request)
{
    tracking_->robot->update();
    const auto error = service_->start_error(*request, tracking_->robot->data.joint_pos);
    if (!error.empty()) {
        service_->reject(request, error);
        return false;
    }
    playing_ = request->motion;
    State_Mimic::motion = playing_;
    playing_->reset(tracking_->robot->data);
    align_mimic_heading(tracking_.get());
    tracking_->reset();
    tick_ = 0;
    holding_ = false;
    if (!service_->started(request)) return false;
    spdlog::info("Online motion STARTED: {} frames", playing_->num_frames);
    return true;
}

void State_OnlineMimic::enter()
{
    bad_orientation_ = false;
    return_to_velocity_ = false;
    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        target_.clear();  // Do not publish stale targets if entry is rejected.
    }
    auto request = service_->activate_mimic();
    try {
        // Check again after Velocity's policy thread has stopped.
        if (!request || !start_motion(request)) {
            service_->deactivate();
            return_to_velocity_ = true;
            return;
        }
    } catch (const std::exception& error) {
        service_->deactivate();
        spdlog::error("Online motion entry failed: {}", error.what());
        bad_orientation_ = true;
        return;
    }
    running_ = true;
    control_thread_ = std::thread([this] {
        try {
            control();
        } catch (const std::exception& error) {
            spdlog::error("Online motion control failed: {}", error.what());
            service_->deactivate();
            running_ = false;
            bad_orientation_ = true;
        }
    });
}

void State_OnlineMimic::exit()
{
    service_->deactivate();
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
    playing_.reset();
}

void State_OnlineMimic::publish_action()
{
    const auto action = tracking_->action_manager->processed_actions();
    std::lock_guard<std::mutex> lock(action_mutex_);
    target_.assign(action.data(), action.data() + action.size());
    kp_ = tracking_->robot->data.joint_stiffness;
    kd_ = tracking_->robot->data.joint_damping;
}

void State_OnlineMimic::run()
{
    std::lock_guard<std::mutex> lock(action_mutex_);
    for (size_t i = 0; i < target_.size(); ++i) {
        auto& motor = lowcmd->msg_.motor_cmd()[i];
        motor.q() = target_[i];
        motor.kp() = kp_[i];
        motor.kd() = kd_[i];
        motor.dq() = 0;
        motor.tau() = 0;
    }
}

void State_OnlineMimic::control()
{
    using Clock = std::chrono::steady_clock;
    auto next = Clock::now();
    while (running_) {
        next += std::chrono::milliseconds(20);
        if (auto request = service_->claim(OnlineMotionService::Mode::Mimic)) {
            start_motion(request);  // A rejected request leaves the held motion unchanged.
        }
        // Each frame lasts 20 ms. Thereafter stay on the final reference indefinitely.
        playing_->frame = static_cast<int>(std::min<size_t>(tick_, playing_->num_frames - 1));
        if (tick_ >= static_cast<size_t>(playing_->num_frames) && !holding_) {
            playing_->dof_velocities.back().setZero();
            holding_ = true;
        }
        tracking_->step();
        bad_orientation_ = isaaclab::mdp::bad_orientation(tracking_.get(), 1.0);
        publish_action();
        if (holding_ && tick_ == static_cast<size_t>(playing_->num_frames)) {
            service_->finished();
            ++tick_;  // Only announce READY once; keep a new request's BUSY state intact.
            spdlog::info("Online motion FINISHED; holding final reference, READY");
        } else if (!holding_) {
            ++tick_;
        }
        std::this_thread::sleep_until(next);
        if (Clock::now() > next + std::chrono::milliseconds(20)) next = Clock::now();
    }
}
