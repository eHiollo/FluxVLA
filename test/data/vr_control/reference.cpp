// Offline harness only: link the original a10_anchor_protocol.cpp, never robot SDK.
#include "a10_anchor_protocol.hpp"
#include "io/json.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

int main() {
    using namespace a10_tcp;
    nlohmann::json input;
    std::cin >> input;
    nlohmann::json output;
    EeAnchorShadow shadow;
    for (const auto& row : input["anchors"]) {
        auto actual = row["actual_pm"].get<std::array<double, 16>>();
        EeAnchorCommand command;
        std::string error;
        if (!parse_ee_anchor_line(row["command"].dump(), command, &error))
            throw std::runtime_error(error);
        const auto update = shadow.update(command, actual.data());
        const char* names[] = {"inactive", "anchored", "updated", "duplicate", "stale"};
        output["anchors"].push_back({{"packet_status", names[static_cast<int>(update)]},
            {"active", shadow.active()}, {"robot_anchor_pm", shadow.robot_anchor_pm()},
            {"target_pm", shadow.user_target_pm()}});
    }
    for (const auto& test : input["governors"]) {
        EeAnchorReferenceGovernor governor;
        AnchorGovernorConfig config;
        config.fault_after_frozen_cycles = test["fault_cycles"];
        governor.set_config(config);
        nlohmann::json rows = nlohmann::json::array();
        for (const auto& row : test["rows"]) {
            auto actual = row["actual_pm"].get<std::array<double, 16>>();
            const std::string operation = row["operation"];
            if (operation == "engage") governor.engage(actual.data());
            else if (operation == "release") governor.release(actual.data());
            else if (operation == "fault") governor.force_fault(actual.data());
            else {
                auto target = row["target_pm"].get<std::array<double, 16>>();
                governor.step(target.data(), actual.data(), row["dt_s"], row.value("hold", false));
            }
            rows.push_back({{"state", anchor_control_state_name(governor.state())},
                {"reference_pm", governor.reference_pm()},
                {"tracking_error_m", governor.tracking_error_m()},
                {"tracking_error_rad", governor.tracking_error_rad()}});
        }
        output["governors"].push_back(rows);
    }
    // Exact operations used by vr_vel::clamp_vec3/slew_vec3. No robot dependencies.
    for (const auto& row : input["velocities"]) {
        auto target = row["target"].get<std::array<double, 3>>();
        auto current = row["current"].get<std::array<double, 3>>();
        const double maximum = row["maximum"], dv = row["maximum_delta"];
        const double norm = std::sqrt(target[0]*target[0] + target[1]*target[1] + target[2]*target[2]);
        if (maximum <= 0) target.fill(0.);
        else if (norm > maximum) for (auto& value : target) value *= maximum / norm;
        for (int i = 0; i < 3; ++i) current[i] = slew_toward(current[i], target[i], dv);
        output["velocities"].push_back({{"clamped", target}, {"slewed", current}});
    }
    std::cout << output.dump(2) << std::endl;
}
