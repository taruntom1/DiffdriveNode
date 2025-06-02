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
public:
    explicit CommunicationInterface(QObject *parent = nullptr, SerialHandler *serialHandler = nullptr);

public slots:
    void listAvaliablePorts(QList<QSerialPortInfo> *ports);
    void toggleConnect(const QString &port_name, int baud_rate);
    void connectSerial(const QString &port_name, int baud_rate);
    void disconnectSerial();
    void commandReceived(char command);

    void ping();

    void sendControllerProperties(controller_properties_t properties);
    void sendWheelData(int id, const wheel_data_t wheel_data);
    void sendPIDConstants(int motor_id, int pid_type, pid_constants_t pid_constants);
    void sendOdoBroadcastStatus(int motor_id, odo_broadcast_flags_t flags);

    void getControllerProperties();
    void getMotorData(int id);

    void sendControlMode(int motor_id, ControlMode mode);

    void sendSetpoints(ControlMode mode, std::vector<float> setpoints);
    // void sendSetpoints(ControlMode mode, std::vector<angularvelocity_t> setpoints);
    // void sendSetpoints(ControlMode mode, std::vector<pwmvalue_t> setpoints);
private:
    SerialHandler *serialHandler;

    int num_wheels = 0;

    bool connected_flag = false;

    bool checkOperationStatus();

    void stopAllBroadcast();
    void restoreAllBroadcast();
    void sendProperty(Command command, const std::vector<uint8_t> &data, const QString &property_name);

    template <typename T>
    void sendSetpointsHelper(Command command, std::vector<T> &setpoints);

    template <typename T>
    void receiveOdoData(std::function<void(wheel_data_t &, T)> setter);

    void receiveOdoAnglesTimestamped();
    void receiveOdoAngles();
    void receiveOdoSpeeds();
    void receiveOdoPwms();
    void receiveControllerProperties();
    void receiveMotorData();

signals:
    void connectionStatusChange(int status, const QString &error_string);
    void propertySetUpdate(bool status, const QString &log_string);

    void controllerPropertiesReceived();
    void motorDataReceived(int motor_id);

    void odometryDataReceived();
    void encoderOdometryReceived(std::vector<timestamped_angle_t> timestamped_angles);

    void timeSyncReplyReceived();
};

template <typename T>
void CommunicationInterface::sendSetpointsHelper(Command command, std::vector<T> &setpoints)
{
    std::vector<uint8_t> data(sizeof(T) * setpoints.size() + 4);
    constexpr uint8_t header[] = {0xAA, 0xAA, 0xAA};
    std::memcpy(data.data(), header, 3);

    data[3] = static_cast<char>(command);

    memcpy(data.data() + 4, setpoints.data(), setpoints.size() * sizeof(T));
    serialHandler->sendData(data);
}

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
