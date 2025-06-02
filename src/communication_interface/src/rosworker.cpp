#include "rosworker.h"

RosWorker::RosWorker(QObject *parent)
    : QThread(parent),
      node_(std::make_shared<rclcpp::Node>("MotorController"))
{
}

std::shared_ptr<rclcpp::Node> RosWorker::getNode() const
{
    return node_;
}

void RosWorker::run()
{
    rclcpp::spin(node_);
}
