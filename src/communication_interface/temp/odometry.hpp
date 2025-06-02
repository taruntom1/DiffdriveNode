#pragma once

#include "differential_drive_odometry.hpp"
#include <memory>
#include <string>
#include <QObject>
#include "structs.h"
#include <vector>

class Odometry
{
    Q_OBJECT
public:
    Odometry();

    void resetPose(double x, double y, double theta);

private:
    // Differential drive odometry object
    differential_drive_odometry_config_t diff_drive_config;
    DifferentialDriveOdometry diff_drive_odometry_;

    // Sync time for ros and MCU
    int64_t sync_sys_time_ns;
    int64_t sync_mcu_time_ns;

public slots:
    void updateDifferentialDrive(std::vector<timestamped_angle_t> timestamped_angles);
    void updateSyncTime(int64_t sys_time_ns, int64_t mcu_time_ns);

signals:
    void diffDriveOdometryUpdated(nav_msgs::msg::Odometry odom_msg);
};
