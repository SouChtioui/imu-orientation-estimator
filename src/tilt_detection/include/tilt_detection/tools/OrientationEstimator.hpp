#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tilt_detection/tools/IMUComplementaryFilter.hpp"

namespace tilt_detection::tools
{

using rclcpp::Time;

class OrientationEstimator
{
public:
    OrientationEstimator() = default;
    ~OrientationEstimator() = default;

    geometry_msgs::msg::Quaternion estimateOrientation(const sensor_msgs::msg::Imu& raw_data);

private:
    void checkTimeJump();
    void checkMsgTimestamp(const Time& timestamp);

    ComplementaryFilter imu_filter_;
    Time time_prev_;
    Time last_ros_time_;
    bool initialized_filter_ = false;
};

} // namespace tilt_detection::tools
