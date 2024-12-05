#ifndef WATCHDOG__WATCHDOG_HPP
#define WATCHDOG__WATCHDOG_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include "yaml-cpp/yaml.h"

/**
 * @brief Configuration for a topic
 */
struct TopicConfig
{
    std::string name; ///< Name of the topic
    std::string msg_type; ///< Message type
    double timeout; ///< Timeout in seconds
    bool required; ///< True if the topic is required
    std::chrono::system_clock::time_point last_update; ///< Last update time
};

/**
 * @brief Configuration for a node
 */
struct NodeConfig
{
    std::string name; ///< Name of the node
    std::string package; ///< Package name
    std::string executable; ///< Executable name
    int32_t restart_attempts; ///< Number of restart attempts
    std::vector<std::string> dependencies; ///< List of dependencies
    std::vector<TopicConfig> topics; ///< List of topics
    int32_t current_attempts{0}; ///< Current number of restart attempts
    bool is_running{false}; ///< True if the node is running
};

/**
 * @brief System for monitoring and restarting ROS 2 nodes
 */
class WatchdogSystem
{
public:
    /**
     * @brief Construct a new Watchdog System object
     * @param config_path Path to the configuration file
     */
    WatchdogSystem(const std::string& config_path);

    /**
     * @brief Load the configuration file
     * @param config_path Path to the configuration file
     */
    void loadConfig(const std::string& config_path);

    /**
     * @brief Check if a node is running
     * @param node_name Name of the node
     * @return True if the node is running, false otherwise
     */
    bool checkNode(const std::string& node_name);

    /**
     * @brief Update the timestamp for a topic
     * @param topic_name Name of the topic
     */
    void updateTopicTimestamp(const std::string& topic_name);

    /**
     * @brief Check if a node can be started
     * @param node_name Name of the node
     * @return True if the node can be started, false otherwise
     */
    bool canStartNode(const std::string& node_name) const;

    /**
     * @brief Restart a node
     * @param node_name Name of the node
     */
    void restartNode(const std::string& node_name);

    /**
     * @brief Get the check interval
     * @return Check interval in seconds
     */
    double getCheckInterval() const { return check_interval_; }

    /**
     * @brief Get the node configurations
     * @return Map of node configurations
     */
    const std::map<std::string, NodeConfig>& getNodeConfigs() const
    {
        return node_configs_;
    }

    /**
     * @brief Check if a node is running
     * @param node_name Name of the node
     * @return True if the node is running, false otherwise
     */
    bool isNodeRunning(const std::string& node_name) const;

    /**
     * @brief Start a node
     * @param node_name Name of the node
     */
    void startNode(const std::string& node_name);

private:
    std::map<std::string, NodeConfig> node_configs_; ///< Map of node configurations
    std::map<std::string, std::chrono::system_clock::time_point> topic_timestamps_; ///< Map of topic timestamps
    double check_interval_; ///< Check interval in seconds

    /**
     * @brief Check if the dependencies for a node are running
     * @param dependencies List of dependencies
     * @return True if the dependencies are running, false otherwise
     */
    bool checkTopics(const NodeConfig& node_config);

    /**
     * @brief Check if the dependencies for a node are running
     * @param dependencies List of dependencies
     * @return True if the dependencies are running, false otherwise
     */
    bool checkDependencies(const std::vector<std::string>& dependencies) const;
};


#endif // WATCHDOG__WATCHDOG_HPP
