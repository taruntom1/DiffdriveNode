#include "encoderodometry.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

EncoderOdometry::EncoderOdometry(const encoder_odometry_config_t &config)
    : last_left_angle_(0.0), last_right_angle_(0.0), last_timestamp_(-1.0),
      x_(0.0), y_(0.0), theta_(0.0), v_x_(0.0), omega_z_(0.0)
{
    wheel_radius_ = config.wheel_radius;
    wheel_base_ = config.wheel_base;
    pose_covariance_ = config.pose_covariance;
    twist_covariance_ = config.twist_covariance;
}

void EncoderOdometry::update(std::vector<timestamped_angle_t> &timestamped_angles)
{
    float &left_angle = timestamped_angles[0].angle;
    float &right_angle = timestamped_angles[1].angle;
    uint64_t &timestamp_left = timestamped_angles[0].timestamp;
    uint64_t &timestamp_right = timestamped_angles[1].timestamp;
    uint64_t timestamp = (timestamp_left + timestamp_right) / 2;

    if (last_timestamp_ < 0.0)
    {
        last_left_angle_ = left_angle;
        last_right_angle_ = right_angle;
        last_timestamp_ = timestamp;
        return;
    }

    double dt = timestamp - last_timestamp_;
    if (dt <= 0.0)
        return;

    double d_left = left_angle - last_left_angle_;
    double d_right = right_angle - last_right_angle_;

    double s_left = d_left * wheel_radius_;
    double s_right = d_right * wheel_radius_;

    double ds = (s_left + s_right) / 2.0;
    double dtheta = (s_right - s_left) / wheel_base_;

    x_ += ds * std::cos(theta_ + dtheta / 2.0);
    y_ += ds * std::sin(theta_ + dtheta / 2.0);
    theta_ += dtheta;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    v_x_ = ds / dt;
    omega_z_ = dtheta / dt;

    last_left_angle_ = left_angle;
    last_right_angle_ = right_angle;
    last_timestamp_ = timestamp;
}

double EncoderOdometry::getX() const { return x_; }
double EncoderOdometry::getY() const { return y_; }
double EncoderOdometry::getTheta() const { return theta_; }

double EncoderOdometry::getLinearVelocity() const { return v_x_; }
double EncoderOdometry::getAngularVelocity() const { return omega_z_; }

void EncoderOdometry::resetPose(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

nav_msgs::msg::Odometry EncoderOdometry::getOdometryMsg() const
{

    nav_msgs::msg::Odometry odom;

    odom.header.frame_id = "";
    odom.child_frame_id = "";
    odom.header.stamp = rclcpp::Time(last_timestamp_ + time_delta_ns_);

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.pose.covariance = std::move(pose_covariance_);

    odom.twist.twist.linear.x = v_x_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.z = omega_z_;

    odom.twist.covariance = std::move(twist_covariance_);

    return odom;
}

void EncoderOdometry::updateTimeDelta(int64_t delta_ns)
{
    time_delta_ns_ = delta_ns;
}
