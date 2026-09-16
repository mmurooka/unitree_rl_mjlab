// Uses a separate DDS domain and test-only lowstate topic. Never creates LowCmd.
// Usage: test_online_motion <G1 project directory> <short converted NPZ>
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
    if (argc != 3) return 2;
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
    auto cfg = param::config["FSM"]["OnlineTest"];
    cfg["policy_dir"] = "config/policy/mimic/dance1_subject2";
    cfg["stand_policy_dir"] = "config/policy/velocity/v0";
    cfg["endpoint"] = endpoint;
    cfg["start_joint_threshold_degrees"] = 30;
    cfg["end_hold_seconds"] = 1;
    FSMStringMap.insert({1, "Passive"});
    State_OnlineMimic state(99, "OnlineTest");
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::req);
    socket.set(zmq::sockopt::rcvtimeo, 5000);
    socket.set(zmq::sockopt::linger, 0);
    socket.connect(endpoint);
    auto request = [&](const std::string& command) {
        socket.send(zmq::buffer(command), zmq::send_flags::none);
        zmq::message_t reply;
        check(bool(socket.recv(reply)), "test response timeout");
        return reply.to_string();
    };
    check(request("STATUS").find("inactive") != std::string::npos, "inactive check");
    state.enter();
    check(request("STATUS") == "READY", "ready check");
    const std::string load = std::string("LOAD\n") + argv[2];
    set_joints(31 * 3.14159265358979323846f / 180);
    check(request(load).find("ERROR start joint 0") == 0, "31 degree rejection");
    check(request("STATUS") == "READY", "rejected request leaves ready");
    set_joints(0);
    const auto start = std::chrono::steady_clock::now();
    check(request(load) == "STARTED", "matching initial pose starts");
    check(request(load) == "BUSY", "no replacement while playing");
    std::this_thread::sleep_for(std::chrono::milliseconds(int(fixture->duration * 1000) + 100));
    check(request(load) == "BUSY", "no replacement during final hold");
    while (request("STATUS") == "BUSY") {
        check(std::chrono::steady_clock::now() - start < std::chrono::seconds(10), "finish timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    check(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >=
          fixture->duration + 0.98, "one second final hold");
    const auto loader = State_Mimic::motion;
    check(loader->frame == loader->num_frames - 1, "final frame held");
    check(loader->joint_vel().isZero(), "held velocity is zero");
    const auto phase = isaaclab::observations_map().at("motion_phase")(nullptr, YAML::Node());
    check(phase.size() == 1 && phase[0] == float(loader->num_frames - 1) / loader->num_frames,
          "one float phase, unchanged final convention");
    state.exit();
    check(request(load).find("inactive") != std::string::npos, "inactive rejects playback");
    std::cout << "PASS: start threshold, playback, BUSY, one-second hold, phase, exit\n";
}
