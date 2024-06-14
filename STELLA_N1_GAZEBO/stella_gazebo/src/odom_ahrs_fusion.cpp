#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define convertor_d2r (M_PI / 180.0)

inline double radian2meter(double radian)
{
    double wheel_diameter = 0.18;

    double gazebo_noise = 0.09;

    double meter= (wheel_diameter - gazebo_noise) * radian;

    return meter;
}

class OdometryFuser : public rclcpp::Node {
public:
    OdometryFuser() : Node("odom_ahrs_fusion"){
        // Initialize the transform broadcaster
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to IMU yaw topic
        imu_yaw_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/imu/yaw", 10, std::bind(&OdometryFuser::imuYawCallback, this, std::placeholders::_1));

        // Subscribe /cmd_vel data
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OdometryFuser::velCallback, this, std::placeholders::_1));

        // Subscribe to joint_states topic
        joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&OdometryFuser::jointStatesCallback, this, std::placeholders::_1));

        // Publisher for fused odometry
        fused_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", 10);

        tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

        // Initialize last time
        last_time_ = this->now();
    }

private:
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states_msg) {
        // Search for wheel joint positions in joint_states message
        auto it_left_wheel = std::find(joint_states_msg->name.begin(), joint_states_msg->name.end(), "wheel_l_joint");
        auto it_right_wheel = std::find(joint_states_msg->name.begin(), joint_states_msg->name.end(), "wheel_r_joint");

        if (it_left_wheel != joint_states_msg->name.end() && it_right_wheel != joint_states_msg->name.end()) {
            // Get wheel joint positions
            size_t left_index = std::distance(joint_states_msg->name.begin(), it_left_wheel);
            size_t right_index = std::distance(joint_states_msg->name.begin(), it_right_wheel);

            double left_wheel_position = joint_states_msg->position[left_index];
            double right_wheel_position = joint_states_msg->position[right_index];

            // Calculate wheel travel distance
            double delta_left_wheel = radian2meter(left_wheel_position - last_left_wheel_position_);
            double delta_right_wheel = radian2meter(right_wheel_position - last_right_wheel_position_);

            last_left_wheel_position_ = left_wheel_position;
            last_right_wheel_position_ = right_wheel_position;

            // Calculate delta_s and delta_th
            double delta_s = (delta_left_wheel + delta_right_wheel) / 2.0;
            double delta_th = ahrs_yaw_;

            double goal_linear_velocity = goal_linear_velocity_;
            double goal_angular_velocity = goal_angular_velocity_;

            // Update fused odometry using the fused delta_s and delta_th
            updateFusedOdometry(delta_s, delta_th, goal_linear_velocity, goal_angular_velocity);
        }
    }

    void imuYawCallback(const std_msgs::msg::Float64::SharedPtr imu_yaw_msg) {
        double ahrs_yaw_d = imu_yaw_msg->data;
        ahrs_yaw_ = (ahrs_yaw_d * convertor_d2r);
    }

    void velCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
        goal_linear_velocity_ = cmd_vel_msg->linear.x;
        goal_angular_velocity_ = cmd_vel_msg->angular.z;
    }

    void updateFusedOdometry(double delta_s, double delta_th, double goal_linear_velocity_, double goal_angular_velocity_) {
        // Update pose using fused delta_s and delta_th
        double delta_x = delta_s * cos(delta_th);
        double delta_y = delta_s * sin(delta_th);

        x_ += delta_x;
        y_ += delta_y;

        // Create odometry message
        nav_msgs::msg::Odometry odom;

        // Convert delta_th to quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, delta_th);

        // Update orientation in odometry message
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();

        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = goal_linear_velocity_;
        odom.twist.twist.angular.z = goal_angular_velocity_;

        // Publish odometry message
        fused_odom_publisher_->publish(odom);

        // Broadcast the transform from "odom" to "base_footprint"
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = this->now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";

        // Set translation
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        // Set orientation
        odom_tf.transform.rotation = tf2::toMsg(quaternion);

        // Wait for the transform to become available
        try {
            tf_buffer_ptr_->canTransform("odom", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.1));
            tf_broadcaster_->sendTransform(odom_tf);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to get transform from odom to base_footprint: %s", ex.what());
        }

        // Update last_time_
        last_time_ = this->now();
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_yaw_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;

    double x_ = 0.0;
    double y_ = 0.0;
    double ahrs_yaw_ = 0.0;
    double goal_linear_velocity_ = 0.0;
    double goal_angular_velocity_ = 0.0;
    double last_left_wheel_position_ = 0.0;
    double last_right_wheel_position_ = 0.0;
    rclcpp::Time last_time_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto odometry_fuser_node = std::make_shared<OdometryFuser>();
    rclcpp::spin(odometry_fuser_node);

    rclcpp::shutdown();
    return 0;
}
