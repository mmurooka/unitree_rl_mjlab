// Isolated domain and test-only topics. No motor command is published.
// Exercises real policy admission, rejection, playback and service lifecycle.
#include "State_OnlineMimic.h"
#include <chrono>
#include <iostream>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void check(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    if (argc != 4) return 2;
    using Clock = std::chrono::steady_clock;
    unitree::robot::ChannelFactory::Instance()->Init(232, "lo");
    FSMState::lowstate = std::make_shared<LowState_t>("rt/test_online_lowstate");
    // Synthetic fixed measured state; no robot or simulator is connected.
    FSMState::lowstate->set_timeout_ms(60000);
    FSMState::lowcmd = std::make_unique<LowCmd_t>("rt/test_online_unpublished_lowcmd");
    auto fixture = std::make_shared<State_Mimic::MotionLoader_>(argv[2]);
    auto& msg = FSMState::lowstate->msg_;
    msg.imu_state().quaternion() = {1, 0, 0, 0};
    for (size_t i=0;i<29;++i) {
        msg.motor_state()[i].q() = fixture->dof_positions[0][i];
        msg.motor_state()[i].dq() = 0;
    }
    param::proj_dir = argv[1];
    const std::string endpoint = "ipc:///tmp/tprl-admission-" + std::to_string(getpid()) + ".sock";
    auto cfg = param::config["FSM"]["OnlineMimic"];
    cfg["policy_dir"] = argv[3];
    cfg["endpoint"] = endpoint;
    // Test-only generous limit makes the good fixture independent of policy
    // version; a 1000-rad previous command below must still be rejected.
    cfg["start_joint_threshold_degrees"] = 180;
    FSMStringMap.insert({1, "Passive"});
    FSMStringMap.insert({3, "Velocity"});
    FSMStringMap.insert({4, "Navigation"});
    FSMStringMap.insert({6, "OnlineMimic"});
    online_motion_service = std::make_shared<OnlineMotionService>(cfg);
    State_OnlineMimic state(6, "OnlineMimic");
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::req);
    socket.set(zmq::sockopt::rcvtimeo, 5000);
    socket.set(zmq::sockopt::linger, 0);
    socket.connect(endpoint);
    auto receive = [&] {
        zmq::message_t reply;
        check(bool(socket.recv(reply)), "response timeout");
        return reply.to_string();
    };
    auto request = [&](const std::string& command) {
        socket.send(zmq::buffer(command), zmq::send_flags::none);
        return receive();
    };
    const std::string load = std::string("LOAD\n") + argv[2];
    const auto inactive_view = nlohmann::json::parse(request("VIEW"));
    check(inactive_view["schema"] == 1 && inactive_view["playback"].is_null(), "VIEW never queues playback");
    check(inactive_view["measured"]["q"].size() == 29, "VIEW exposes measured SDK joints while inactive");
    auto claim = [&](OnlineMotionService::Mode mode) {
        const auto deadline=Clock::now()+std::chrono::seconds(5);
        while (!online_motion_service->transition_ready(mode)) {
            check(Clock::now()<deadline, "claim timeout");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    for (const auto mode : {OnlineMotionService::Mode::Velocity, OnlineMotionService::Mode::Navigation}) {
        if (mode==OnlineMotionService::Mode::Velocity) online_motion_service->activate_velocity();
        else online_motion_service->activate_navigation();
        FSMState::lowcmd->msg_.motor_cmd()[0].q() = 1000;
        socket.send(zmq::buffer(load), zmq::send_flags::none);
        claim(mode);
        check(!state.approve_enter(), "discontinuous entry must be rejected before source exit");
        check(receive().find("ERROR command-to-command") == 0, "command continuity rejection");
        check(request("STATUS")=="READY", "source remains ready without reentry");
        check(!online_motion_service->transition_request(), "no stale rejected transition");
        // Same live source can accept another request; rejected admission did
        // not call source exit()/enter(), reset its policy or replace its goal.
        FSMState::lowcmd->msg_.motor_cmd()[0].q() = fixture->dof_positions[0][0];
        socket.send(zmq::buffer(load), zmq::send_flags::none);
        claim(mode);
        check(state.approve_enter(), "valid candidate admitted");
        if (mode==OnlineMotionService::Mode::Velocity) online_motion_service->leave_velocity();
        else online_motion_service->leave_navigation();
        state.enter();
        check(receive()=="STARTED", "prepared candidate starts after source exit");
        const auto playing_view = nlohmann::json::parse(request("VIEW"));
        check(playing_view["playback"]["path"] == argv[2], "VIEW identifies exact converted reference");
        check(playing_view["playback"]["num_frames"] == fixture->num_frames, "VIEW reports policy frame count");
        check(request(load)=="BUSY", "playing motion is not replaced");
        const auto deadline=Clock::now()+std::chrono::seconds(15);
        while (request("STATUS")=="BUSY") {
            check(Clock::now()<deadline, "playback timeout");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        check(request("STATUS")=="READY", "final reference holds and accepts requests");
        const auto held_view = nlohmann::json::parse(request("VIEW"));
        check(held_view["playback"]["id"] == playing_view["playback"]["id"], "playback ID survives hold");
        check(held_view["playback"]["frame"] == fixture->num_frames-1, "VIEW follows actual frame rather than wall-time animation");
        state.exit();
        check(State_Mimic::motion->frame == fixture->num_frames-1, "final reference frame held");
        check(State_Mimic::motion->joint_vel().isZero(), "held reference velocity is zero");
    }
    check(request("STATUS").find("inactive")!=std::string::npos, "exit disables reception");
    std::cout << "PASS: pre-exit command admission, retained source, prepared playback and final hold\n";
}
