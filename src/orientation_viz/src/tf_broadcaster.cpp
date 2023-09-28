#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

using TransformStamped = geometry_msgs::msg::TransformStamped;
using Quaternion = geometry_msgs::msg::Quaternion;

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster()
        : Node("imu_tf_broadcaster")
    {
        RCLCPP_INFO(get_logger(), "imu tf broadcaster is up");
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        subscription_ = create_subscription<Quaternion>("/imu/orientation", 10, std::bind(&TFBroadcaster::data_ready_callback, this, _1));
    }

private:
    void data_ready_callback(const Quaternion& orientation)
    {
        auto tf = TransformStamped();
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "imu";
        tf.transform.rotation = orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Subscription<Quaternion>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
