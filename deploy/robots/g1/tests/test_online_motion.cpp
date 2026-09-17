// Isolated DDS domain/topic; no LowCmd publisher. Exercises the real policies
// and Velocity transition predicate, driving FSM lifecycle hooks explicitly.
// Usage: test_online_motion <G1 project directory> <short converted NPZ> [policy directory]
#include "State_OnlineMimic.h"
#include <chrono>
#include <iostream>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void check(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv)
{
    if (argc != 3 && argc != 4) return 2;
    using Clock = std::chrono::steady_clock;
    unitree::robot::ChannelFactory::Instance()->Init(232, "lo");
    FSMState::lowstate = std::make_shared<LowState_t>("rt/test_online_lowstate");
    auto fixture = std::make_shared<State_Mimic::MotionLoader_>(argv[2]);
    auto set_joints = [&](float offset) {
        std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
        auto& msg = FSMState::lowstate->msg_;
        msg.imu_state().quaternion() = {1, 0, 0, 0};
        for (size_t i = 0; i < 29; ++i) {
            msg.motor_state()[i].q() = fixture->dof_positions[0][i] + (i == 0 ? offset : 0);
            msg.motor_state()[i].dq() = 0;
        }
    };
    set_joints(0);
    param::proj_dir = argv[1];
    const std::string endpoint = "ipc:///tmp/tprl-online-smoke-" + std::to_string(getpid()) + ".sock";
    auto cfg = param::config["FSM"]["OnlineMimic"];
    cfg["policy_dir"] = argc == 4 ? argv[3] : "config/policy/mimic/dance1_subject2";
    cfg["endpoint"] = endpoint;
    cfg["start_joint_threshold_degrees"] = 30;
    param::config["FSM"]["Velocity"]["policy_dir"] = "config/policy/velocity/v0";
    FSMStringMap.insert({1, "Passive"});
    FSMStringMap.insert({3, "Velocity"});
    FSMStringMap.insert({6, "OnlineMimic"});
    {
        State_RLBase offline_velocity(3, "Velocity");
        for (const auto& transition : offline_velocity.registered_checks) {
            check(transition.second != 6, "ordinary Velocity works without online service");
        }
    }
    online_motion_service = std::make_shared<OnlineMotionService>(cfg);
    State_RLBase velocity(3, "Velocity");
    auto motion_ready = [&] {
        for (const auto& transition : velocity.registered_checks) {
            if (transition.second == FSMStringMap.right.at("OnlineMimic")) {
                return transition.first();
            }
        }
        throw std::runtime_error("Velocity has no online motion transition");
    };
    State_OnlineMimic state(6, "OnlineMimic");
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::req);
    socket.set(zmq::sockopt::rcvtimeo, 5000);
    socket.set(zmq::sockopt::linger, 0);
    socket.connect(endpoint);
    auto read_reply = [&] {
        zmq::message_t reply;
        check(bool(socket.recv(reply)), "test response timeout");
        return reply.to_string();
    };
    auto request = [&](const std::string& command) {
        socket.send(zmq::buffer(command), zmq::send_flags::none);
        return read_reply();
    };
    const std::string load = std::string("LOAD\n") + argv[2];
    int transitions = 0;
    auto load_from_velocity = [&](bool move_during_transition = false) {
        socket.send(zmq::buffer(load), zmq::send_flags::none);
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        while (!(socket.get(zmq::sockopt::events) & ZMQ_POLLIN)) {
            if (motion_ready()) {
                ++transitions;
                online_motion_service->leave_velocity();
                if (move_during_transition) set_joints(0.6f);
                state.enter();
            }
            check(Clock::now() < deadline, "Velocity transition timed out");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return read_reply();
    };
    auto wait_ready = [&] {
        const auto start = Clock::now();
        while (request("STATUS") == "BUSY") {
            check(Clock::now() - start < std::chrono::seconds(5), "playback finish timeout");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        check(request("STATUS") == "READY", "final hold is receptive");
    };
    check(request("STATUS").find("inactive") != std::string::npos, "inactive check");
    // Avoid constructing a motor publisher just to execute the base Velocity enter().
    online_motion_service->activate_velocity();
    set_joints(31 * 3.14159265358979323846f / 180);
    check(load_from_velocity().find("ERROR start joint 0") == 0, "31 degree rejection");
    check(transitions == 0 && request("STATUS") == "READY", "reject stays in Velocity");
    set_joints(0);
    check(load_from_velocity(true).find("ERROR start joint 0") == 0, "recheck after Velocity exits");
    check(state.registered_checks.back().first(), "changed pose requests return to Velocity");
    state.exit();
    online_motion_service->activate_velocity();
    set_joints(0);
    const auto start = Clock::now();
    check(load_from_velocity() == "STARTED", "valid request enters OnlineMimic");
    check(request(load) == "BUSY", "no replacement during playback");
    wait_ready();
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    check(elapsed < fixture->duration + 0.75, "no additional one-second acceptance delay");
    // Rejecting a new start pose while holding must preserve the current reference.
    set_joints(0.6f);
    check(request(load).find("ERROR start joint 0") == 0, "held-pose start check");
    check(request("STATUS") == "READY", "rejected request keeps holding and receptive");
    set_joints(0);
    check(request(load) == "STARTED", "next motion starts within OnlineMimic");
    wait_ready();
    check(transitions == 2, "no FSM transition for the second accepted motion");
    // Holding outlives the former one-second limit and does not switch policy.
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    check(request("STATUS") == "READY", "long hold remains receptive");
    state.exit();  // Join before inspecting mutable loader fields.
    const auto loader = State_Mimic::motion;
    check(loader->frame == loader->num_frames - 1, "final frame held");
    check(loader->joint_pos().isApprox(fixture->dof_positions.back()), "final pose held");
    check(loader->joint_vel().isZero(), "held reference velocity is zero");
    const auto phase = isaaclab::observations_map().at("motion_phase")(nullptr, YAML::Node());
    check(phase.size() == 1 && phase[0] == float(loader->num_frames - 1) / loader->num_frames,
          "phase keeps the final single-float convention");
    check(request(load).find("inactive") != std::string::npos, "inactive rejects playback");
    // Outstanding requests must not survive leaving Velocity for another state.
    online_motion_service->activate_velocity();
    socket.send(zmq::buffer(load), zmq::send_flags::none);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    online_motion_service->leave_velocity();
    check(read_reply().find("ERROR") == 0, "state exit cancels pending request");
    online_motion_service->activate_velocity();
    check(!motion_ready() && request("STATUS") == "READY", "no stale motion after reentry");
    online_motion_service->deactivate();
    std::cout << "PASS: Velocity entry/rejection, BUSY, immediate readiness, continuous Mimic hold, replacement, cancellation\n";
}
