#pragma once
#include <QObject>
#include <QEventLoop>
#include <QTimer>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "structs.h"
#include "structserialiser.h"
#include "communicationinterface.h"
#include "timesyncclient.h"
class ControllerManager : public QObject
{
    Q_OBJECT

public:
    explicit ControllerManager(CommunicationInterface *commInterface = nullptr, TimeSyncClient *timeSyncClient = nullptr, QObject *parent = nullptr);
    ~ControllerManager();

private:
    CommunicationInterface *commInterface;
    TimeSyncClient *timeSyncClient;
    rclcpp::Logger logger = rclcpp::get_logger("Controller Manager");

    QString port_name;
    int baud_rate;
    controller_data_t controller_data;

    void manageQtConnections();

    bool readConfigurationJson(); // add baud rate reading
    bool connectController();
    void disconnectController();
    void setTimeSync(bool status);
    bool setControllerProperties();
    bool setWheelData();

    bool retryOperation(const std::function<bool()> &operation, int maxRetries, int delayMs);

public slots:

    // void setControlMode();
    // void setOdoBroadcast();
    // void setPIDConstants();
    
    void timeSync(int64_t sync_sys_time_ns, int64_t sync_mcu_time_ns, int64_t delta);


signals:
    void connectToController(const QString &port_name, int baud_rate);
    void disconnectFromController();
    void sendControllerProperties(const controller_properties_t properties);
    void sendWheelData(int id, const wheel_data_t wheel_data);
    void sendControlMode(int motor_id, ControlMode mode);
    void sendOdometryBroadcastStatus(int motor_id, odo_broadcast_flags_t flags);
    void sendPIDConstants(int motor_id, int type, pid_constants_t pid_constants);

    void startTimesync();
    void stopTimesync();
};