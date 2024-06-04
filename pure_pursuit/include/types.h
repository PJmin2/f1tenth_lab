//
// Created by yash on 10/20/19.
//

#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.h>

namespace f110
{

struct WayPoint
{
    WayPoint() = default;
    WayPoint(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg, double current_speed = 1.0)
    {
        x = pose_msg->pose.position.x;
        y = pose_msg->pose.position.y;

        tf2::Quaternion quaternion{pose_msg->pose.orientation.x,
                                  pose_msg->pose.orientation.y,
                                  pose_msg->pose.orientation.z,
                                  pose_msg->pose.orientation.w};
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        heading = yaw;

        speed = current_speed;
    }

    WayPoint(const geometry_msgs::msg::PoseStamped &pose_msg, double current_speed = 1.0)
    {
        x = pose_msg.pose.position.x;
        y = pose_msg.pose.position.y;

        tf2::Quaternion quaternion{pose_msg.pose.orientation.x,
                                  pose_msg.pose.orientation.y,
                                  pose_msg.pose.orientation.z,
                                  pose_msg.pose.orientation.w};
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        heading = yaw;

        speed = current_speed;
    }

    friend std::ostream& operator << (std::ostream& os, const WayPoint& way_point);

    std::string GetString() const
    {
        std::stringstream stream;
        stream << "x: " << x << "\ty: " << y << "\theading: " << heading << "\tspeed: " <<  speed << "\n";
    }

    double x;
    double y;
    double heading;
    double speed;
};

std::ostream& operator << (std::ostream& os, const WayPoint& way_point)
{
    return os << "x: " << way_point.x << "\ty: " << way_point.y << "\ty: " << std::endl;
}

}
