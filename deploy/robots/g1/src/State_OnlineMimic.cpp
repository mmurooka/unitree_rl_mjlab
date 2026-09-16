#include "State_OnlineMimic.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include <chrono>

State_OnlineMimic::State_OnlineMimic(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    const auto cfg = param::config["FSM"][state_string];
    threshold_ = cfg["start_joint_threshold_degrees"].as<float>(30.0f) *
                 3.14159265358979323846f / 180.0f;
    hold_seconds_ = cfg["end_hold_seconds"].as<float>(1.0f);
    if (!std::isfinite(threshold_) || threshold_ <= 0 ||
        !std::isfinite(hold_seconds_) || hold_seconds_ < 0) {
        throw std::runtime_error("Invalid online motion threshold/hold time");
    }
    State_Mimic::motion = std::make_shared<Loader>();
    auto make_env = [&](const std::string& directory, bool standing) {
        auto path = param::parser_policy_dir(directory);
        auto policy = YAML::LoadFile(path / "params" / "deploy.yaml");
        if (standing) {
            // Clamp every joystick velocity command to zero, including gait_phase.
            for (const auto* key : {"lin_vel_x", "lin_vel_y", "ang_vel_z"}) {
                policy["commands"]["base_velocity"]["ranges"][key] = std::vector<float>{0, 0};
            }
        }
        auto env = std::make_unique<isaaclab::ManagerBasedRLEnv>(policy,
            std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(lowstate));
        if (std::abs(env->step_dt - 0.02f) > 1e-6f || env->robot->data.joint_ids_map.size() != 29) {
            throw std::runtime_error("Online G1 policies must use 29 joints and step_dt=0.02");
        }
        // Current G1 MotionPrompt converter emits motor order. Keep this contract local.
        for (size_t i = 0; i < 29; ++i) {
            if (env->robot->data.joint_ids_map[i] != i) {
                throw std::runtime_error("Online G1 policy requires the existing motor joint order");
            }
        }
        env->alg = std::make_unique<isaaclab::OrtRunner>(path / "exported" / "policy.onnx");
        return env;
    };
    tracking_ = make_env(cfg["policy_dir"].as<std::string>(), false);
    standing_ = make_env(cfg["stand_policy_dir"].as<std::string>(), true);
    registered_checks.emplace_back([this] { return bad_orientation_.load(); },
                                   FSMStringMap.right.at("Passive"));
    socket_.set(zmq::sockopt::linger, 0);
    socket_.set(zmq::sockopt::rcvtimeo, 100);
    socket_.set(zmq::sockopt::sndtimeo, 100);
    socket_.set(zmq::sockopt::maxmsgsize, int64_t(8192));
    socket_.bind(cfg["endpoint"].as<std::string>("ipc:///tmp/task_prompt_rl_deploy.sock"));
    receiver_thread_ = std::thread(&State_OnlineMimic::receive, this);
}

State_OnlineMimic::~State_OnlineMimic()
{
    exit();
    receiving_ = false;
    if (receiver_thread_.joinable()) receiver_thread_.join();
}

void State_OnlineMimic::enter()
{
    bad_orientation_ = false;
    standing_->reset();
    standing_->step();
    publish_action(standing_.get());
    running_ = true;
    status_ = Status::Ready;
    control_thread_ = std::thread([this] {
        try {
            control();
        } catch (const std::exception& error) {
            spdlog::error("Online motion control failed: {}", error.what());
            status_ = Status::Inactive;
            running_ = false;
            bad_orientation_ = true;  // Let the FSM move to Passive.
        }
    });
    spdlog::info("Online motion READY (zero velocity)");
}

void State_OnlineMimic::exit()
{
    status_ = Status::Inactive;
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (pending_) {
        pending_->result.set_value("ERROR online state exited");
        pending_.reset();
    }
}

void State_OnlineMimic::publish_action(isaaclab::ManagerBasedRLEnv* env)
{
    const auto action = env->action_manager->processed_actions();
    std::lock_guard<std::mutex> lock(action_mutex_);
    target_.assign(action.data(), action.data() + action.size());
    kp_ = env->robot->data.joint_stiffness;
    kd_ = env->robot->data.joint_damping;
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
    std::shared_ptr<Loader> playing;
    size_t tick = 0;
    while (running_) {
        next += std::chrono::milliseconds(20);
        std::shared_ptr<Request> request;
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            request.swap(pending_);
        }
        if (request) {
            tracking_->robot->update();
            const auto actual = tracking_->robot->data.joint_pos;
            const auto reference = request->motion->joint_pos();
            Eigen::Index joint = 0;
            const float difference = (actual - reference).cwiseAbs().maxCoeff(&joint);
            if (!actual.allFinite() || difference > threshold_) {
                request->result.set_value(fmt::format(
                    "ERROR start joint {} differs by {:.2f} deg (limit {:.2f} deg)",
                    joint, difference * 180.0f / 3.14159265358979323846f,
                    threshold_ * 180.0f / 3.14159265358979323846f));
                auto busy = Status::Busy;
                status_.compare_exchange_strong(busy, Status::Ready);
            } else {
                playing = request->motion;
                State_Mimic::motion = playing;
                playing->reset(tracking_->robot->data);
                align_mimic_heading(tracking_.get());
                tracking_->reset();
                tick = 0;
                request->result.set_value("STARTED");
                spdlog::info("Online motion STARTED: {} frames", playing->num_frames);
            }
        }
        auto* env = standing_.get();
        if (playing) {
            // Every received frame runs for one 20 ms tick; then hold for n seconds.
            const auto hold_ticks = static_cast<size_t>(std::ceil(hold_seconds_ / 0.02f));
            if (tick >= static_cast<size_t>(playing->num_frames) + hold_ticks) {
                playing.reset();
                standing_->reset();
                auto busy = Status::Busy;
                status_.compare_exchange_strong(busy, Status::Ready);
                spdlog::info("Online motion FINISHED; zero velocity READY");
            } else {
                playing->frame = std::min<int>(tick, playing->num_frames - 1);
                if (tick >= static_cast<size_t>(playing->num_frames)) {
                    playing->dof_velocities.back().setZero();
                }
                env = tracking_.get();
                ++tick;
            }
        }
        env->step();
        bad_orientation_ = isaaclab::mdp::bad_orientation(env, 1.0);
        publish_action(env);
        std::this_thread::sleep_until(next);
        if (Clock::now() > next + std::chrono::milliseconds(20)) next = Clock::now();
    }
}

void State_OnlineMimic::receive()
{
    while (receiving_) {
        zmq::message_t message;
        if (!socket_.recv(message, zmq::recv_flags::none)) continue;
        std::string reply;
        const std::string command = message.to_string();
        if (command == "STATUS") {
            const auto status = status_.load();
            reply = status == Status::Ready ? "READY" :
                    status == Status::Busy ? "BUSY" : "ERROR online state is inactive";
        } else if (command.rfind("LOAD\n", 0) == 0) {
            auto expected = Status::Ready;
            if (!status_.compare_exchange_strong(expected, Status::Busy)) {
                reply = expected == Status::Busy ? "BUSY" : "ERROR online state is inactive";
            } else {
                try {
                    auto request = std::make_shared<Request>();
                    request->motion = std::make_shared<Loader>(command.substr(5));
                    auto result = request->result.get_future();
                    {
                        std::lock_guard<std::mutex> lock(request_mutex_);
                        if (!running_) throw std::runtime_error("online state exited");
                        pending_ = request;
                    }
                    // Resolved by the control loop or exit(), never by another socket thread.
                    while (result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
                        if (!running_) throw std::runtime_error("online control stopped");
                    }
                    reply = result.get();
                } catch (const std::exception& error) {
                    reply = std::string("ERROR ") + error.what();
                    auto busy = Status::Busy;
                    status_.compare_exchange_strong(busy, Status::Ready);
                }
            }
        } else {
            reply = "ERROR expected STATUS or LOAD";
        }
        socket_.send(zmq::buffer(reply), zmq::send_flags::none);
    }
}
