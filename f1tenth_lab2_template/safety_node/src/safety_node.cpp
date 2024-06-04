#include <sstream>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;

class Safety : public rclcpp::Node {

public:

    Safety() : Node("safety_node")
    {

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10); 

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));

    }

private:

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    float speed = 0.0;
    float ttc = 0.0;
    ackermann_msgs::msg::AckermannDriveStamped msg;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

        const vector<float> &ranges = scan_msg->ranges;

	for (int i = 0; i < ranges.size(); ++i)
	{
	    float speed_along_ray = speed * cosf(angle_min + angle_increment * i);

	    if (speed_along_ray <= 0.0)
	    {
		continue;
	    }

	    ttc = ranges.at(i) / speed_along_ray;

	    if (ttc < 2.0)
	    {
	        msg.drive.speed = 0.0;
		RCLCPP_INFO(this->get_logger(), "AEB Engaged");
		publisher_->publish(msg);
		break;
	    }
	}
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
