#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "csv_reader.h"
#include "pure_pursuit.h"
#include "types.h"

using namespace std;

const string &pose_topic = "/pf/viz/inferred_pose";
const string &drive_topic = "/cmd_vel";
const string &waypoint_viz_topic = "/waypoint_markers";

class PurePursuit : public rclcpp::Node {

public:

    PurePursuit() : Node("pure_pursuit_node")
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 5, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(drive_topic, 1);
        waypoint_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(waypoint_viz_topic, queue_size);

        this->declare_parameter("lookahead_distance");
        this->declare_parameter("high_speed");
        this->declare_parameter("medium_speed");
        this->declare_parameter("low_speed");
        this->declare_parameter("n_way_points");
        this->declare_parameter("kp");
        this->declare_parameter("ki");
        this->declare_parameter("kd");

        this->get_parameter("lookahead_distance", lookahead_distance_);
        this->get_parameter("high_speed", high_speed_);
        this->get_parameter("medium_speed", medium_speed_);
        this->get_parameter("low_speed", low_speed_);
        this->get_parameter("n_way_points", n_way_points_);
        this->get_parameter("kp", kp_);
	this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);

	f110::CSVReader reader("/home/min/colcon_ws/src/pure_pursuit/sensor_data/waypoint_gazebo.csv");
        RCLCPP_INFO(this->get_logger(), "%d", n_way_points_);
	way_point_data_ = reader.getData(n_way_points_);
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node Initialized");
        visualize_waypoint_data(way_point_data_, "map", 0.0, 1.0, 0.0, 1.0, 0.1, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Way Points Published as Markers.");
        rclcpp::sleep_for(std::chrono::seconds(1));

    }

    void visualize_waypoint_data(const std::vector<f110::WayPoint>& way_point, const std::string& frame_id,
            double r, double g, double b, double transparency = 1.0, double scale_x=0.1, double scale_y=0.0,
            double scale_z=0.0)
    {
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = frame_id;
        line_strip.header.stamp = this->get_clock()->now();
        line_strip.ns = "line";
        line_strip.id = unique_marker_id_;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.pose.orientation.x = 0.0;
        line_strip.pose.orientation.y = 0.0;
        line_strip.pose.orientation.z = 0.0;
        line_strip.pose.orientation.w = 1.0;
	line_strip.scale.x = scale_x;
        line_strip.scale.y = scale_y;
	line_strip.scale.z = scale_z;
	line_strip.color.a = transparency;
	line_strip.color.r = r;
	line_strip.color.g = g;
	line_strip.color.b = b;

	for (size_t i = 0; i < way_point_data_.size(); i += 10)
        {
	    geometry_msgs::msg::Point p;
	    p.x = way_point_data_[i].x;
	    p.y = way_point_data_[i].y;
	    p.z = 0;

	    line_strip.points.push_back(p);
	}

	waypoint_viz_pub_->publish(line_strip);
	queue_size = 1;
	visualized_ = true;
    }

    void add_way_point_visualization(const f110::WayPoint& way_point, const std::string& frame_id,
            double r, double g, double b, double transparency = 1.0, double scale_x=0.1, double scale_y=0.1,
            double scale_z=0.1)
    {
        visualization_msgs::msg::Marker way_point_marker;
        way_point_marker.header.frame_id = frame_id;
        way_point_marker.header.stamp = this->get_clock()->now();
        way_point_marker.ns = "point";
        way_point_marker.id = unique_marker_id_;
        way_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        way_point_marker.action = visualization_msgs::msg::Marker::ADD;
        way_point_marker.pose.position.x = way_point.x;
        way_point_marker.pose.position.y = way_point.y;
        way_point_marker.pose.position.z = 0;
        way_point_marker.pose.orientation.x = 0.0;
        way_point_marker.pose.orientation.y = 0.0;
        way_point_marker.pose.orientation.z = 0.0;
        way_point_marker.pose.orientation.w = 1.0;
        way_point_marker.scale.x = scale_x;
        way_point_marker.scale.y = scale_y;
        way_point_marker.scale.z = scale_z;
        way_point_marker.color.a = transparency;
        way_point_marker.color.r = r;
        way_point_marker.color.g = g;
        way_point_marker.color.b = b;

        if (visualized_) {waypoint_viz_pub_->publish(way_point_marker);}
    }

    float get_velocity(float angle)
    {

        if (angle < M_PI/18)
        {
            return high_speed_;
        }

        if (angle < M_PI/9)
        {
            return medium_speed_;
        }

        return low_speed_;

    }

    void pid_control(float error)
    {

        double angle = 0.0;

        rclcpp::Time previous_reading_time = current_reading_time;
        current_reading_time = this->get_clock()->now();

        double dt = (current_reading_time - previous_reading_time).seconds();
        //RCLCPP_INFO(this->get_logger(), "dt = %lf", dt);
        double integral = prev_error * dt;
        double derivative = (error - prev_error) / dt;

        angle = kp_ * error + ki_ * integral + kd_ * derivative;
        RCLCPP_INFO(this->get_logger(), "angle = %lf", angle);
        prev_error = error;

        if (abs(error) < 0.1)
        {
            angle = 0.0;
        }

        float velocity = get_velocity(abs(angle));

        drive_msg.linear.x = velocity;
        drive_msg.angular.z = angle;
        drive_pub_->publish(drive_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr pose_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "last_best_index_ = %d", last_best_index_);

	const auto current_way_point = f110::WayPoint(pose_msg);

        // Find the best waypoint to track (at lookahead distance)
        const auto goal_way_point_index = f110::get_best_track_point_index(way_point_data_, current_way_point, lookahead_distance_, last_best_index_);

	geometry_msgs::msg::PoseStamped goal_way_point;
        goal_way_point.pose.position.x = way_point_data_[goal_way_point_index].x;
        goal_way_point.pose.position.y = way_point_data_[goal_way_point_index].y;
        goal_way_point.pose.position.z = 0;
        goal_way_point.pose.orientation.x = 0;
        goal_way_point.pose.orientation.y = 0;
        goal_way_point.pose.orientation.z = 0;
        goal_way_point.pose.orientation.w = 1;

        add_way_point_visualization(goal_way_point, "map", 0.0, 0.0, 1.0, 1.0, 0.2, 0.2, 0.2);

	double x_diff = goal_way_point.pose.position.x - current_way_point.x;
	double y_diff = goal_way_point.pose.position.y - current_way_point.y;
        double yaw = current_way_point.heading;

	double rotation_x_diff = x_diff*cos(yaw) + y_diff*sin(yaw);
	double rotation_y_diff = x_diff*sin(yaw) - y_diff*cos(yaw);

        double steering_angle = -atan2(rotation_y_diff, rotation_x_diff);

        // Calculate curvature/steering angle
        RCLCPP_INFO(this->get_logger(), "steering_angle = %lf, yaw = %lf", steering_angle, yaw * (180/3.14159));
        RCLCPP_INFO(this->get_logger(), "atan2( %lf, %lf ) = %lf", rotation_y_diff, rotation_x_diff, steering_angle);
        RCLCPP_INFO(this->get_logger(), "goal_way_point.x =  %lf, current_way_point.x = %lf", goal_way_point.pose.position.x, current_way_point.x);

        pid_control(steering_angle);
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_viz_pub_;

    // Publish drive message
    geometry_msgs::msg::Twist drive_msg;

    double lookahead_distance_;
    double high_speed_;
    double medium_speed_;
    double low_speed_;
    double kp_;
    double ki_;
    double kd_;

    double prev_error = 0.0;
    double error = 0.0;

    rclcpp::Time current_reading_time = this->get_clock()->now();

    int n_way_points_;

    std::vector<f110::WayPoint> way_point_data_;

    bool visualized_ = false;
    size_t queue_size = 100;
    size_t unique_marker_id_ = 0;
    size_t last_best_index_ = 0;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
