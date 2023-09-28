#include "tilt_detection/tools/OrientationEstimator.hpp"

namespace tilt_detection::tools
{

geometry_msgs::msg::Quaternion OrientationEstimator::estimateOrientation(const sensor_msgs::msg::Imu& raw_data)
{
    checkTimeJump();

    const Time& time = raw_data.header.stamp;

    if(initialized_filter_)
    {
        checkMsgTimestamp(time);
    }

    // Initialize.
    if(!initialized_filter_)
    {
        time_prev_ = time;
        initialized_filter_ = true;
        return geometry_msgs::msg::Quaternion();
    }

    const auto& lin_acc = raw_data.linear_acceleration;
    const auto& ang_vel = raw_data.angular_velocity;

    const auto dt = (time - time_prev_).seconds();
    time_prev_ = time;

    // Update the filter.
    imu_filter_.update(lin_acc.x, lin_acc.y, lin_acc.z, ang_vel.x, ang_vel.y, ang_vel.z, dt);

    auto q = geometry_msgs::msg::Quaternion();
    imu_filter_.getOrientation(q.w, q.x, q.y, q.z);
    return q;
}

void OrientationEstimator::checkTimeJump()
{
    const auto now = Time();
    if(last_ros_time_ <= now)
    {
        last_ros_time_ = now;
        return;
    }

    // Reset
    imu_filter_.reset();
    last_ros_time_ = Time();
    initialized_filter_ = false;
}

void OrientationEstimator::checkMsgTimestamp(const Time& timestamp)
{
    // Invalid timestamp
    if((timestamp - time_prev_).nanoseconds() <= 0)
    {
        // Reset
        imu_filter_.reset();
        last_ros_time_ = Time();
        initialized_filter_ = false;
    }
}

} // namespace tilt_detection::tools
