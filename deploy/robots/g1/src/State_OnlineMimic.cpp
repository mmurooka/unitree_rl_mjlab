#include "State_OnlineMimic.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include <chrono>

State_OnlineMimic::State_OnlineMimic(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string),
      service_(online_motion_service),
      navigation_service_(navigation_goal_service)
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
    // Reuse a second inference context so rejecting a new motion leaves the
    // held policy's observation/action history intact.
    candidate_tracking_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(path / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(lowstate));
    candidate_tracking_->alg = std::make_unique<isaaclab::OrtRunner>(path / "exported" / "policy.onnx");
    registered_checks.emplace_back([this] { return bad_orientation_.load(); },
                                   FSMStringMap.right.at("Passive"));
    registered_checks.emplace_back([this] { return return_to_velocity_.load(); },
                                   FSMStringMap.right.at("Velocity"));
    if (FSMStringMap.right.count("Navigation")) {
        registered_checks.emplace_back(
            [this] { return return_to_navigation_.load(); },
            FSMStringMap.right.at("Navigation")
        );
        if (navigation_service_) {
            const auto navigation_service = navigation_service_;
            registered_checks.emplace_back(
                [navigation_service] {
                    return navigation_service->transition_ready(
                        NavigationGoalService::Mode::Mimic
                    );
                },
                FSMStringMap.right.at("Navigation")
            );
        }
    }
}

State_OnlineMimic::~State_OnlineMimic()
{
    exit();
}

bool State_OnlineMimic::start_motion(const std::shared_ptr<OnlineMotionService::Request>& request)
{
    std::vector<float> before;
    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        before = target_;
    }
    try {
        return prepare_motion(request, before) && commit_motion(request, before);
    } catch (const std::exception& error) {
        service_->reject(request, std::string("ERROR candidate policy inference: ") + error.what());
        return false;
    }
}

bool State_OnlineMimic::approve_enter()
{
    prepared_request_.reset();
    const auto request = service_->transition_request();
    if (!request) return false;
    prepared_before_.clear();
    // FSM run()/publish() just completed. Only this thread writes lowcmd;
    // the source policy may continue inference while we check the candidate.
    for (size_t i=0;i<29;++i) prepared_before_.push_back(lowcmd->msg_.motor_cmd()[i].q());
    try {
        if (!prepare_motion(request, prepared_before_)) return false;
    } catch (const std::exception& error) {
        service_->reject(request, std::string("ERROR candidate policy inference: ") + error.what());
        return false;
    }
    prepared_request_ = request;
    return true;
}

bool State_OnlineMimic::prepare_motion(
    const std::shared_ptr<OnlineMotionService::Request>& request,
    const std::vector<float>& before)
{
    if (lowstate->isTimeout()) {
        service_->reject(request, "ERROR measured state unavailable for candidate policy inference");
        return false;
    }
    const auto previous_motion = State_Mimic::motion;
    try {
        State_Mimic::motion = request->motion;
        candidate_tracking_->robot->update();
        request->motion->reset(candidate_tracking_->robot->data);
        align_mimic_heading(candidate_tracking_.get());
        candidate_tracking_->reset();
        candidate_tracking_->step();
    } catch (...) {
        State_Mimic::motion = previous_motion;
        throw;
    }
    State_Mimic::motion = previous_motion;
    const auto after = candidate_tracking_->action_manager->processed_actions();
    const auto error = service_->command_start_error(before, after);
    if (!error.empty()) {
        service_->reject(request, error);
        return false;
    }
    return true;
}

bool State_OnlineMimic::commit_motion(
    const std::shared_ptr<OnlineMotionService::Request>& request,
    const std::vector<float>& before)
{
    const auto after = candidate_tracking_->action_manager->processed_actions();
    if (!service_->started(request)) return false;
    float command_change = 0.0f;
    for (size_t i=0;i<before.size();++i) command_change = std::max(command_change, std::abs(before[i]-after[i]));
    spdlog::info("OnlineMimic command-to-command start change: {:.2f} deg", command_change*180.0f/3.14159265f);
    if (navigation_service_) navigation_service_->set_mimic_busy();
    std::swap(tracking_, candidate_tracking_);
    playing_ = request->motion;
    State_Mimic::motion = playing_;
    tick_ = 0;
    holding_ = false;
    prepared_action_ = true;
    spdlog::info("Online motion STARTED: {} frames", playing_->num_frames);
    return true;
}

void State_OnlineMimic::request_return_to_source()
{
    if (source_mode_ == OnlineMotionService::Mode::Navigation &&
        FSMStringMap.right.count("Navigation")) {
        return_to_navigation_ = true;
    } else {
        return_to_velocity_ = true;
    }
}

void State_OnlineMimic::enter()
{
    bad_orientation_ = false;
    return_to_velocity_ = false;
    return_to_navigation_ = false;
    source_mode_ = OnlineMotionService::Mode::Velocity;
    if (navigation_service_) {
        navigation_service_->activate_mimic();
    }
    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        target_.clear();  // Do not publish stale targets if entry is rejected.
    }
    auto request = service_->activate_mimic(source_mode_);
    try {
        // No source run()/publication occurs between approve_enter() and here.
        // Reuse the checked candidate, rather than inferring a different first command.
        const bool prepared = request && request == prepared_request_;
        prepared_request_.reset();
        if (!prepared || !commit_motion(request, prepared_before_)) {
            service_->deactivate();
            request_return_to_source();
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
    if (navigation_service_) {
        navigation_service_->leave_mimic();
    }
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
        if (prepared_action_) prepared_action_ = false;
        else tracking_->step();
        bad_orientation_ = isaaclab::mdp::bad_orientation(tracking_.get(), 1.0);
        publish_action();
        service_->update_view_frame(playing_->frame);
        if (holding_ && tick_ == static_cast<size_t>(playing_->num_frames)) {
            service_->finished();
            if (navigation_service_) {
                navigation_service_->set_mimic_ready();
            }
            ++tick_;  // Only announce READY once; keep a new request's BUSY state intact.
            spdlog::info("Online motion FINISHED; holding final reference, READY");
        } else if (!holding_) {
            ++tick_;
        }
        std::this_thread::sleep_until(next);
        if (Clock::now() > next + std::chrono::milliseconds(20)) next = Clock::now();
    }
}
