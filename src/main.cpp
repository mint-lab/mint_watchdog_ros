/**
 * @file main.cpp
 * @author DONGWOOK HEO (hdwook3918 (at) gmail.com)
 * @brief This is the main file for the mint_watchdog node.
 * @version 0.1
 * @date 2024-12-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <cstdint>
#include "WatchdogNode.hpp"

int32_t main(int32_t argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WatchdogNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}