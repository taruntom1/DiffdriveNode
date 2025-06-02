#pragma once

#include <memory>
#include <string>
#include <vector>
#include <QObject>

#include "structs.h"
#include "differential_drive_odometry.h"
#include "communicationinterface.h"
#include "rosworker.h"

class Odometry : public QObject
{
    Q_OBJECT
public:
    explicit Odometry(CommunicationInterface *communication_interface = nullptr, RosWorker *ros = nullptr, QObject *parent = nullptr);

    void resetPose(double x, double y, double theta);

private:
    CommunicationInterface *communication_interface_;
    RosWorker *ros_;
    // Differential drive odometry object
    differential_drive_odometry_config_t diff_drive_config;
    DifferentialDriveOdometry diff_drive_odometry_;

    // Sync time for ros and MCU
    int64_t sync_sys_time_ns;
    int64_t sync_mcu_time_ns;

    void makeConnections();

public slots:
    void updateDifferentialDrive(std::vector<timestamped_angle_t> timestamped_angles);
    void updateSyncTime(int64_t sys_time_ns, int64_t mcu_time_ns);

signals:
    void publishOdometry(nav_msgs::msg::Odometry odom_msg);
};
