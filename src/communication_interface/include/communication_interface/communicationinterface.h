#pragma once

#include <QObject>
#include <vector>
#include "serialhandler.h"
#include "structs.h"
#include "commands.h"
#include <QElapsedTimer>

class CommunicationInterface : public QObject
{
    Q_OBJECT

private:
    SerialHandler *serialHandler;
    controller_data_t *controllerData;
    int num_wheels = 0;

    bool connected_flag = false;

    void stopAllBroadcast();
    void restoreAllBroadcast();
    void sendProperty(Command command, const std::vector<uint8_t> &data, const QString &property_name);

    template <typename T>
    void sendSetpointsHelper(Command command, const std::function<T(const wheel_data_t &)> &accessor);

    template <typename T>
    void receiveOdoData(std::function<void(wheel_data_t &, T)> setter);

    void receiveOdoAngles();
    void receiveOdoSpeeds();
    void receiveOdoPwms();
    void receiveOdoAnglesTimestamped();
    void receiveControllerProperties();
    void receiveMotorData();

public:
    explicit CommunicationInterface(QObject *parent = nullptr, SerialHandler *serialHandler = nullptr, controller_data_t *controllerData = nullptr);

public slots:
    void listAvaliablePorts(QList<QSerialPortInfo> *ports);
    void toggleConnect(const QString &port_name, int baud_rate);
    void connectSerial(const QString &port_name, int baud_rate);
    void disconnectSerial();
    void commandReceived(char command);

    void ping();

    void sendControllerProperties(controller_properties_t properties);
    void sendWheelData(int id, const wheel_data_t wheel_data);
    void sendPIDConstants(int motor_id, int pid_type);
    void sendOdoBroadcastStatus(int motor_id, odo_broadcast_flags_t flags);

    void getControllerProperties();
    void getMotorData(int id);

    void sendControlMode(int motor_id, ControlMode mode);

    // void sendSetpoints(ControlMode mode);
    // template <typename T>
    // void sendSetpoints(ControlMode mode, std::vector<T> setpoints);

/*     void sendAngleSetpoint();
    void sendPWMSetpoint();
    void sendSpeedSetpoint(); */

signals:
    void connectionStatusChange(int status, const QString &error_string);
    void propertySetUpdate(bool status, const QString &log_string);

    void controllerPropertiesReceived();
    void motorDataReceived(int motor_id);

    void odometryDataReceived();
    void encoderOdometryReceived(std::vector<timestamped_angle_t> timestamped_angles);

    void timeSyncReplyReceived();
};

/* template <typename T>
void CommunicationInterface::sendSetpointsHelper(Command command, const std::function<T(const wheel_data_t &)> &accessor)
{
    std::vector<uint8_t> data(sizeof(T) * controllerData->controllerProperties.numMotors + 4);
    constexpr uint8_t header = {0xAA, 0xAA, 0xAA};
    std::memcpy(data.data(), header, 3);
    data[3] = static_cast<char>(command);
    int i = 0;
    for (const auto &wheel : controllerData->wheelData)
    {
        T value = accessor(wheel);
        std::memcpy(data.data() + i + 4, &value, sizeof(T));
        i += sizeof(T);
    }
    serialHandler->sendData(data);
}
 */
/* template <typename T>
void CommunicationInterface::sendSetpoints(ControlMode mode, std::vector<T> setpoints)
{
    std::vector<uint8_t> data(sizeof(T) * setpoints.size() + 4);
    constexpr uint8_t header = {0xAA, 0xAA, 0xAA};
    std::memcpy(data.data(), header, 3);
    Command command;
    switch (mode)
    {
    case ControlMode::PWM_DIRECT_CONTROL:
        command = Command::SET_MOTOR_PWMS;
        break;
    case ControlMode::POSITION_CONTROL:
        command = Command::SET_MOTOR_ANGLE_SETPOINTS;
        break;
    case ControlMode::SPEED_CONTROL:
        command = Command::SET_MOTOR_SPEED_SETPOINTS;
        break;
    default:
        return;
    }

    data[3] = static_cast<char>(command);

    memcpy(data.data() + 4, setpoints.data(), setpoints.size() * sizeof(T));
    serialHandler->sendData(data);
} */

template <typename T>
void CommunicationInterface::receiveOdoData(
    std::function<void(wheel_data_t &, T)> setter)
{
    size_t size = sizeof(T) * controllerData->wheelData.size();
    std::vector<uint8_t> data = serialHandler->readData(size);

    int i = 0;
    for (auto &wheel : controllerData->wheelData)
    {
        T value = *reinterpret_cast<T *>(&data[i]);
        setter(wheel, value);
        i += sizeof(T);
    }

    emit odometryDataReceived();
    serialHandler->setAutoReadCommandFlag(true);
}
