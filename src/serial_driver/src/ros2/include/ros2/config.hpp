#pragma once
#include "rm_decision/config.hpp"
#include "serial/config.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
namespace ros2 {
struct Ros2config {
    io::SerialConfig serial_config;
    bt::DecisionConfig decision_config;
    bool is_decision = true;
};
inline bool load_config(const std::string& filepath, Ros2config& cfg) {
    try {
        YAML::Node root = YAML::LoadFile(filepath);
        YAML::Node node = root["serial_node"]; // 最外层节点
        if (!node) {
            spdlog::error("missing 'serial_node' section");
            return false;
        }

        // ----- serial 段 -----
        YAML::Node s = node["serial"];
        cfg.serial_config.serial_name = s["serial_name"].as<std::string>();
        cfg.serial_config.baud_rate = s["baud_rate"].as<int>();
        cfg.serial_config.max_try = s["max_try"].as<int>();
        cfg.serial_config.socket_send_name = s["socket_send_name"].as<std::string>();
        cfg.serial_config.socket_receive_name = s["socket_receive_name"].as<std::string>();
        auto sm = s["sending_method"].as<std::string>();
        if (sm == "scoket") {
            cfg.serial_config.sending_method = io::SendingMethod::SOCKET;
        } else if (sm == "serial") {
            cfg.serial_config.sending_method = io::SendingMethod::SERIAL;
        } else {
            spdlog::error("[config]:加载config时 sending_method错误");
        }
        // ----- decision 段 -----
        YAML::Node d = node["decision"];
        cfg.is_decision = d["is_decision"].as<bool>();
        cfg.decision_config.tree_xml_file = d["tree_xml_file"].as<std::string>();
        cfg.decision_config.tree_node_model_export_path = d["tree_node_model_export_path"].as<std::string>();
        cfg.decision_config.map_tf_name = d["map_tf_name"].as<std::string>();

        return true;
    } catch (const YAML::Exception& e) {
        spdlog::error("YAML error: {}" , e.what() );
        return false;
    }
}
}