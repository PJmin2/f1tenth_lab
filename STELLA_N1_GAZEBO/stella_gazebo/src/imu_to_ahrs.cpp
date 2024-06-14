#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUToAHRSNode : public rclcpp::Node
{
    public:
    IMUToAHRSNode()
    : Node("imu_to_ahrs")
    {
        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&IMUToAHRSNode::imu_callback, this, std::placeholders::_1));

        yaw_publisher_ = create_publisher<std_msgs::msg::Float64>("imu/yaw", 10);
    }

    private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double yaw = calculate_yaw(msg->orientation);
        auto yaw_msg = std::make_unique<std_msgs::msg::Float64>();
        yaw_msg->data = yaw;
        yaw_publisher_->publish(std::move(yaw_msg));
    }

    double calculate_yaw(const geometry_msgs::msg::Quaternion& orientation)
    {
        double x = orientation.x;
        double y = orientation.y;
        double z = orientation.z;
        double w = orientation.w;

        //roll and pitch
        /*  
        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(t0, t1);
        double roll_degrees = roll * (180.0 / M_PI);
        double t2 = +2.0 * (w * y - z * x);
        t2 = +1.0 < t2 ? +1.0 : t2;
        t2 = -1.0 > t2 ? -1.0 : t2;
        double pitch = std::asin(t2);
        double pitch_degrees = pitch * (180.0 / M_PI);
        */

        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(t3, t4);
        double yaw_degrees = yaw * (180.0 / M_PI);

        return yaw_degrees;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUToAHRSNode>());
    rclcpp::shutdown();
    return 0;
}