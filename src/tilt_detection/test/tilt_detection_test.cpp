#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tilt_detection/tools/OrientationEstimator.hpp"
#include "tilt_detection_msgs/msg/tilt_status.hpp"

namespace tilt_detection
{

namespace
{

bool areEqual(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2)
{
    return q1.x == q2.x && q1.y == q2.y && q1.z == q2.z && q1.w == q2.w;
}

void randomFillImuData(sensor_msgs::msg::Imu& data)
{
    std::uniform_real_distribution<double> uniform_ang_vel(M_PI, 2 * M_PI);
    std::uniform_real_distribution<double> uniform_lin_acc(0.0, 5);
    std::default_random_engine re;
    data.angular_velocity.x = uniform_ang_vel(re);
    data.angular_velocity.y = uniform_ang_vel(re);
    data.angular_velocity.z = uniform_ang_vel(re);

    data.linear_acceleration.x = uniform_lin_acc(re);
    data.linear_acceleration.y = uniform_lin_acc(re);
    data.linear_acceleration.z = uniform_lin_acc(re);
}

constexpr double Dt = 10. * 1e6; // 10 ms

} // namespace

TEST(tilt_detection, time_freeze)
{
    auto orientation_estimator = tools::OrientationEstimator();

    auto data = sensor_msgs::msg::Imu();
    data.header.stamp = rclcpp::Time();

    for(auto i = 0u; i < 10; ++i)
    {
        randomFillImuData(data); // Update data
        ASSERT_TRUE(areEqual(orientation_estimator.estimateOrientation(data), geometry_msgs::msg::Quaternion()));
    }
}

TEST(tilt_detection, time_backward)
{
    auto orientation_estimator = tools::OrientationEstimator();

    auto data = sensor_msgs::msg::Imu();
    auto t0 = rclcpp::Time() + rclcpp::Duration(0, 20 * Dt);
    for(auto i = 1u; i < 10; ++i)
    {
        randomFillImuData(data);                              // Update data
        data.header.stamp = t0 - rclcpp::Duration(0, i * Dt); // backward update timestamp
        ASSERT_TRUE(areEqual(orientation_estimator.estimateOrientation(data), geometry_msgs::msg::Quaternion()));
    }
}

TEST(tilt_detection, time_forward)
{
    auto orientation_estimator = tools::OrientationEstimator();

    auto data = sensor_msgs::msg::Imu();
    randomFillImuData(data);
    auto t0 = rclcpp::Time();
    data.header.stamp = t0;
    auto prev_orientation = geometry_msgs::msg::Quaternion();
    auto orientation = orientation_estimator.estimateOrientation(data);
    ASSERT_TRUE(areEqual(orientation, prev_orientation)); // init filter

    for(auto i = 1u; i < 10; ++i)
    {
        prev_orientation = orientation;
        randomFillImuData(data);                              // Update data
        data.header.stamp = t0 + rclcpp::Duration(0, i * Dt); // forward update timestamp
        orientation = orientation_estimator.estimateOrientation(data);
        ASSERT_FALSE(areEqual(orientation, prev_orientation));
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace tilt_detection
