#include "odometry.h"

Odometry::Odometry(odometry_config_t config, QObject *parent)
    : QObject(parent),
      encoderOdometry_(config.encoder_odometry_config)
{
}

void Odometry::updateEncoderOdometry(std::vector<timestamped_angle_t> timestamped_angles)
{
    encoderOdometry_.update(timestamped_angles);
    emit publishEncoderOdometry(encoderOdometry_.getOdometryMsg());
}

void Odometry::resetPose(double x, double y, double theta)
{
    encoderOdometry_.resetPose(x, y, theta);
}

void Odometry::updateTimeDelta(int64_t delta_ns)
{
    encoderOdometry_.updateTimeDelta(delta_ns);
}