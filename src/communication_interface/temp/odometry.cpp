#include "odometry.hpp"

Odometry::Odometry()
    : diff_drive_odometry_(diff_drive_config)
{
}

void Odometry::updateDifferentialDrive(std::vector<timestamped_angle_t> timestamped_angles)
{
    diff_drive_odometry_.update(timestamped_angles);
    emit diffDriveOdometryUpdated(diff_drive_odometry_.getOdometryMsg());
}

void Odometry::resetPose(double x, double y, double theta)
{
    diff_drive_odometry_.resetPose(x, y, theta);
}

void Odometry::updateSyncTime(int64_t sys_time_ns, int64_t mcu_time_ns)
{
    sync_sys_time_ns = sys_time_ns;
    sync_mcu_time_ns = mcu_time_ns;
}
