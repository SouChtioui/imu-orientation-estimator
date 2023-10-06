#include <chrono>
#include <cmath>
#include <functional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tilt_detection/tools/OrientationEstimator.hpp"
#include "tilt_detection_msgs/msg/tilt_status.hpp"

using std::placeholders::_1;

using ImuMsg = sensor_msgs::msg::Imu;
using TiltStatusMsg = tilt_detection_msgs::msg::TiltStatus;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue = diagnostic_msgs::msg::KeyValue;
using OrientationEstimator = tilt_detection::tools::OrientationEstimator;
using Quaternion = geometry_msgs::msg::Quaternion;

namespace
{
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double TILT_THRESHOLD = 15 * DEG_TO_RAD;
} // namespace

class TiltDetector : public rclcpp::Node
{
public:
    TiltDetector()
        : Node("tilt_detection_node")
    {
        RCLCPP_INFO(get_logger(), "Tilt detection started");

        declare_parameter("publish_diagnostics", rclcpp::PARAMETER_BOOL);
        declare_parameter("publish_orientation", rclcpp::PARAMETER_BOOL);

        publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();
        publish_orientation_ = this->get_parameter("publish_orientation").as_bool();
        timer_ = this->create_wall_timer(std::chrono_literals::operator""ms(100), std::bind(&TiltDetector::timer_callback, this));

        subscription_ = create_subscription<ImuMsg>("/imu/data_raw", 10, std::bind(&TiltDetector::data_ready_callback, this, _1));

        tilt_status_publisher_ = create_publisher<TiltStatusMsg>("/tilt/status", 10);
        if(publish_diagnostics_)
        {
            diagnostic_publisher_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
        }
        if(publish_orientation_)
        {
            orientation_publisher_ = create_publisher<Quaternion>("/imu/orientation", 10);
        }
    }

private:
    void data_ready_callback(const ImuMsg& data_raw)
    {
        static const tf2::Vector3 Z_AXIS = {0.0, 0.0, -1.0};

        const auto q = orientation_estimator_.estimateOrientation(data_raw);

        const tf2::Quaternion imu_q(q.x, q.y, q.z, q.w);
        const tf2::Matrix3x3 imu_rot(imu_q);
        const auto imu_z_axis = imu_rot.getRow(2);
        current_tilt_angle_ = std::fabs(imu_z_axis.angle(Z_AXIS));
        auto tilt_status = TiltStatusMsg();
        tilt_status.tilted = current_tilt_angle_ > TILT_THRESHOLD;
        tilt_status_publisher_->publish(tilt_status);

        if(publish_orientation_)
        {
            orientation_publisher_->publish(q);
        }
    }

    void timer_callback() const
    {
        if(publish_diagnostics_)
        {
            DiagnosticStatus emu_status;
            emu_status.name = "EMU";
            emu_status.level = DiagnosticStatus::OK;
            emu_status.message = "EMU up and running";

            KeyValue emergency_stop_diag;
            emergency_stop_diag.key = "Emergency stop button";
            emergency_stop_diag.value = "Released";

            KeyValue tilt_angle_diag;
            tilt_angle_diag.key = "Tilt angle in degrees";
            tilt_angle_diag.value = std::to_string(current_tilt_angle_ * RAD_TO_DEG);

            emu_status.values.push_back(emergency_stop_diag);
            emu_status.values.push_back(tilt_angle_diag);

            auto diagnostics = DiagnosticArray();
            diagnostics.status.push_back(emu_status);

            diagnostic_publisher_->publish(diagnostics);
        }
    }

    rclcpp::Subscription<ImuMsg>::SharedPtr subscription_;
    rclcpp::Publisher<TiltStatusMsg>::SharedPtr tilt_status_publisher_;
    rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostic_publisher_;
    rclcpp::Publisher<Quaternion>::SharedPtr orientation_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    OrientationEstimator orientation_estimator_;
    double current_tilt_angle_ = 0.;
    bool publish_diagnostics_ = false;
    bool publish_orientation_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TiltDetector>());
    rclcpp::shutdown();
    return 0;
}
