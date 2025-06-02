#include "rosworker.h"

RosWorker::RosWorker(QObject *parent)
    : QThread(parent),
      node_(std::make_shared<rclcpp::Node>("MotorController"))
{
    makeOdoPublisher();
}

std::shared_ptr<rclcpp::Node> RosWorker::getNode() const
{
    return node_;
}

void RosWorker::run()
{
    rclcpp::spin(node_);
}

void RosWorker::makeOdoPublisher()
{
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

void RosWorker::publishOdometry(nav_msgs::msg::Odometry odom_msg)
{
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    if (odom_pub_ && rclcpp::ok())
    {
        odom_pub_->publish(odom_msg);
    }
}
