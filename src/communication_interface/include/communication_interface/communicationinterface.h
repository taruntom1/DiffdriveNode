#pragma once

#include <QObject>
#include <QElapsedTimer>
#include <QSerialPortInfo>

#include <vector>
#include <cstring>

#include "serialhandler.h"
#include "structs.h"
#include "commands.h"

class CommunicationInterface : public QObject
{
    Q_OBJECT

public:
    explicit CommunicationInterface(QObject *parent = nullptr, SerialHandler *serialHandler = nullptr);

public slots:
    void listAvailablePorts(QList<QSerialPortInfo> *ports);
    void toggleConnect(const QString &port_name, int baud_rate);
    void connectSerial(const QString &port_name, int baud_rate);
    void disconnectSerial();
    void commandReceived(char command);

    void ping();

    void sendControllerProperties(controller_properties_t properties);
    void sendWheelData(int id, const wheel_data_t wheel_data);
    void sendPIDConstants(int motor_id, int pid_type, pid_constants_t pid_constants);
    void sendOdoBroadcastStatus(int motor_id, odo_broadcast_flags_t flags);

    void sendControlMode(int motor_id, ControlMode mode);

    void sendSetpoints(ControlMode mode, std::vector<float> setpoints);

signals:
    void connectionStatusChange(int status, const QString &error_string);
    void propertySetUpdate(bool status, const QString &log_string);

    void controllerPropertiesReceived();
    void motorDataReceived(int motor_id);

    void odometryDataReceived();
    void encoderOdometryReceived(std::vector<timestamped_angle_t> timestamped_angles);

    void timeSyncReplyReceived();

private:
    SerialHandler *serialHandler;

    int num_wheels = 0;
    bool connected_flag = false;

    void stopAllBroadcast();
    void restoreAllBroadcast();
    void sendProperty(Command command, const std::vector<uint8_t> &data, const QString &property_name);

    void receiveOdoAnglesTimestamped();

    template <typename T>
    void sendSetpointsHelper(Command command, std::vector<T> &setpoints);
};

template <typename T>
void CommunicationInterface::sendSetpointsHelper(Command command, std::vector<T> &setpoints)
{
    std::vector<uint8_t> data(sizeof(T) * setpoints.size() + 4);

    constexpr uint8_t header[] = {0xAA, 0xAA, 0xAA};
    std::memcpy(data.data(), header, 3);

    data[3] = static_cast<uint8_t>(command);

    std::memcpy(data.data() + 4, setpoints.data(), setpoints.size() * sizeof(T));
    serialHandler->sendData(data);
}
