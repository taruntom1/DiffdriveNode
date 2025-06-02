#pragma once

#include <QThread>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RosWorker : public QThread
{
    Q_OBJECT

public:
    explicit RosWorker(QObject *parent = nullptr);
    std::shared_ptr<rclcpp::Node> getNode() const;

signals:
    void messageReceived(const QString &msg);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

