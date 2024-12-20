#include <iostream>
#include <cstdlib>
#include <thread>
#include "Watchdog.hpp"

WatchdogSystem::WatchdogSystem(const std::string& config_path)
{
    loadConfig(config_path);
}

void WatchdogSystem::loadConfig(const std::string& config_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_path);
        check_interval_ = config["check_interval"].as<double>();

        auto nodes = config["nodes"];
        for (const auto& node : nodes)
        {
            NodeConfig node_config;
            node_config.name = node.second["name"].as<std::string>();
            node_config.package = node.second["package"].as<std::string>();
            node_config.executable = node.second["executable"].as<std::string>();
            node_config.restart_attempts = node.second["restart_attempts"].as<int>();

            // Load dependencies if they exist
            if (node.second["dependencies"])
            {
                node_config.dependencies =
                    node.second["dependencies"].as<std::vector<std::string>>();
            }

            // Load topics
            auto topics = node.second["topics"];
            for (const auto& topic : topics)
            {
                TopicConfig topic_config;
                topic_config.name = topic["name"].as<std::string>();
                topic_config.msg_type = topic["msg_type"].as<std::string>();
                topic_config.timeout = topic["timeout"].as<double>();
                topic_config.required = topic["required"].as<bool>();
                node_config.topics.push_back(topic_config);
            }

            node_configs_[node_config.name] = node_config;
        }
    } catch (const YAML::Exception& e)
    {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        throw;
    }
}

bool WatchdogSystem::isNodeRunning(const std::string& node_name) const
{
    std::string cmd = "ros2 node list 2>/dev/null | grep " + node_name + " >/dev/null";
    int32_t result = std::system(cmd.c_str());
    return (result == 0);
}

WatchdogSystem::NodeStatus WatchdogSystem::checkNode(const std::string& node_name)
{
    WatchdogSystem::NodeStatus status = static_cast<WatchdogSystem::NodeStatus>(WatchdogSystem::NodeStatusFlag::NONE);

    auto it = node_configs_.find(node_name);
    if (it == node_configs_.end())
    {
        return WatchdogSystem::NodeStatusFlag::NONE;
    }

    auto& node_config = it->second;
    bool is_running = isNodeRunning(node_name);

    if (!is_running)
    {
        node_config.is_running = false;
        if (canStartNode(node_name))
        {
            startNode(node_name);
        }
        return WatchdogSystem::NodeStatusFlag::NOT_RUNNING;
    }

    if (!checkTopics(node_config))
    {
        if (node_config.current_attempts < node_config.restart_attempts)
        {
            node_config.current_attempts++;
            restartNode(node_name);
            return WatchdogSystem::NodeStatusFlag::TOPIC_UNHEALTHY;
        }
        else
        {
            // std::cerr << "Node " << node_name << " has exceeded the maximum number of restart attempts" << std::endl;
            return WatchdogSystem::NodeStatusFlag::RESTART_ATTEMPTS_EXCEEDED;
        }
    }

    node_config.is_running = true;
    node_config.current_attempts = 0;
    return WatchdogSystem::NodeStatusFlag::NODE_HEALTHY;

    // // Check dependencies first
    // if (!checkDependencies(node_config.dependencies))
    // {

    //     return WatchdogSystem::NodeStatusFlag::DEPENDENCY_UNHEALTHY;
    // }
}

bool WatchdogSystem::checkTopics(const NodeConfig& node_config)
{
    auto now = std::chrono::system_clock::now();

    for (const auto& topic : node_config.topics)
    {
        auto it = topic_timestamps_.find(topic.name);
        if (it == topic_timestamps_.end())
        {
            if (topic.required) return false;
            continue;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
            (now - it->second).count() / 1000.0;

        if (elapsed > topic.timeout && topic.required)
        {
            return false;
        }
    }
    return true;
}

void WatchdogSystem::updateTopicTimestamp(const std::string& topic_name)
{
    topic_timestamps_[topic_name] = std::chrono::system_clock::now();
}

bool WatchdogSystem::canStartNode(const std::string& node_name) const
{
    auto it = node_configs_.find(node_name);
    if (it == node_configs_.end()) return false;
    return checkDependencies(it->second.dependencies);
}

bool WatchdogSystem::checkDependencies(
    const std::vector<std::string>& dependencies) const
    {
    for (const auto& dep : dependencies)
    {
        auto it = node_configs_.find(dep);
        if (it == node_configs_.end() || !it->second.is_running)
        {
            return false;
        }
    }
    return true;
}

void WatchdogSystem::startNode(const std::string& node_name)
{
    auto& node_config = node_configs_[node_name];

    if (isNodeRunning(node_name))
    {
        node_config.is_running = true;
        return;
    }

    std::string cmd = "ros2 launch " + node_config.package + " " +
                     node_config.executable + " &";
    int32_t result = std::system(cmd.c_str());

    if (result == 0)
    {
        node_config.is_running = true;
        node_config.current_attempts = 0;
    }
}

void WatchdogSystem::restartNode(const std::string& node_name)
{
    auto& node_config = node_configs_[node_name];

    if (isNodeRunning(node_name))
    {
        std::string kill_cmd = "pkill -f " + node_name;
        std::system(kill_cmd.c_str());
    }

    node_config.current_attempts++;
    node_config.is_running = false;

    startNode(node_name);
}
