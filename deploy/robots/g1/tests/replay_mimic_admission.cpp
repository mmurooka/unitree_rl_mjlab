// Offline inference sensitivity analysis. Opens no DDS/ZMQ and publishes nothing.
#include <unordered_map>
#include <vector>
#include <cstring>
#include <fstream>
#include <cmath>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "isaaclab/algorithms/algorithms.h"

int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "usage: replay_mimic_admission POLICY_DIR CONTROLLER_LOG\n"; return 2; }
    try {
        const std::string dir=argv[1];
        const auto cfg=YAML::LoadFile(dir+"/params/deploy.yaml");
        const auto action=cfg["actions"]["JointPositionAction"];
        const auto scale=action["scale"].as<std::vector<float>>();
        const auto offset=action["offset"].as<std::vector<float>>();
        if (scale.size()!=29 || offset.size()!=29 || !action["clip"].IsNull())
            throw std::runtime_error("Expected unclipped 29-joint affine position action");
        size_t index=0, last_index=0; bool found=false;
        for (auto term:cfg["observations"]) {
            if (term.second["history_length"].as<int>(1)!=1 || !term.second["clip"].IsNull())
                throw std::runtime_error("Expected history=1, unclipped observations");
            if (term.first.as<std::string>()=="last_action") {last_index=index; found=true;}
            auto scales=term.second["scale"].as<std::vector<float>>();
            for (float s:scales) if (s!=1) throw std::runtime_error("Expected unit observation scales");
            index+=scales.size();
        }
        if (!found) throw std::runtime_error("Missing last_action observation");
        isaaclab::OrtRunner runner(dir+"/exported/policy.onnx");
        std::ifstream file(argv[2]); if (!file) throw std::runtime_error("Cannot open log");
        std::string line; int count=0;
        while (std::getline(file,line)) {
            const std::string tag="OnlineMimic admission trace: ";
            auto start=line.find(tag); if(start==std::string::npos) continue;
            auto trace=nlohmann::json::parse(line.substr(start+tag.size()));
            auto obs=trace.at("observations").get<std::unordered_map<std::string,std::vector<float>>>();
            auto before=trace.at("before_commands").get<std::vector<float>>();
            auto recorded=trace.at("candidate_commands").get<std::vector<float>>();
            if (before.size()!=29 || recorded.size()!=29 || obs.at("obs").size()!=index)
                throw std::runtime_error("Unexpected trace shape");
            auto infer=[&](auto input) {
                auto result=runner.act(input);
                for(size_t i=0;i<29;++i) result[i]=result[i]*scale[i]+offset[i];
                return result;
            };
            auto baseline=infer(obs);
            float replay_error=0;
            for(size_t i=0;i<29;++i) {
                replay_error=std::max(replay_error,std::abs(baseline[i]-recorded[i]));
                if (!std::isfinite(scale[i]) || scale[i]==0) throw std::runtime_error("Invalid action scale");
                obs.at("obs").at(last_index+i)=(before[i]-offset[i])/scale[i];
            }
            if (replay_error>1e-4f) throw std::runtime_error("Recorded output does not match supplied policy");
            auto seeded=infer(obs);
            for(size_t i=0;i<29;++i) obs.at("obs").at(last_index+i)=0;
            auto zero_reset=infer(obs);
            auto maximum=[&](const auto& output) {
                float largest=0; size_t joint=0;
                for(size_t i=0;i<29;++i) if(std::abs(output[i]-before[i])>largest) {
                    largest=std::abs(output[i]-before[i]); joint=i;
                }
                return nlohmann::json{{"max_change_deg",largest*180/3.14159265f},{"joint",joint}};
            };
            nlohmann::json result={{"path",trace["path"]},{"recorded_result",trace["result"]},
                {"baseline",maximum(baseline)}, {"previous_command_last_action",maximum(seeded)}, {"zero_reset_last_action",maximum(zero_reset)},
                {"replay_error_rad",replay_error}, {"scope","offline sensitivity only; no physical validation"}};
            std::cout << result.dump() << '\n'; ++count;
        }
        if (!count) throw std::runtime_error("No admission traces found");
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
