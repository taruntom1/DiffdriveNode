#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <array>
#include <vector>
#include "structs.h"

struct differential_drive_odometry_config_t
{
    double wheel_radius;
    double wheel_base;
    std::array<double, 36> pose_covariance;
    std::array<double, 36> twist_covariance;
};

class DifferentialDriveOdometry
{
public:
    DifferentialDriveOdometry(const differential_drive_odometry_config_t &config);

    void update(std::vector<timestamped_angle_t> &timestamped_angles);

    double getX() const;
    double getY() const;
    double getTheta() const;

    double getLinearVelocity() const;
    double getAngularVelocity() const;

    void resetPose(double x, double y, double theta);

    nav_msgs::msg::Odometry getOdometryMsg() const;

private:
    double wheel_radius_;
    double wheel_base_;

    std::array<double, 36> pose_covariance_;
    std::array<double, 36> twist_covariance_;

    double last_left_angle_;
    double last_right_angle_;
    double last_timestamp_;

    double x_, y_, theta_;
    double v_x_, omega_z_;
};
