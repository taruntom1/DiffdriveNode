#include "communicationinterface.h"

CommunicationInterface::CommunicationInterface(QObject *parent, SerialHandler *serialHandler, controller_data_t *controllerData)
    : QObject{parent}, serialHandler(serialHandler), controllerData(controllerData)
{
    connect(serialHandler, &SerialHandler::commandReceived, this, &CommunicationInterface::commandReceived, Qt::DirectConnection);
}

void CommunicationInterface::listAvaliablePorts(QList<QSerialPortInfo> *ports)
{
    ports->append(serialHandler->getAvailablePorts());
}

void CommunicationInterface::toggleConnect(const QString &port_name, int baud_rate)
{
    QString error_string;
    int status;
    if (serialHandler->checkConnection())
    {
        serialHandler->disconnectSerial();
        status = -1;
    }
    else
    {
        status = (serialHandler->connectSerial(port_name, baud_rate, &error_string));
    }
    emit connectionStatusChange(status, error_string);
}

void CommunicationInterface::connectSerial(const QString &port_name, int baud_rate)
{
    QString error_string;
    int status;
    status = serialHandler->connectSerial(port_name, baud_rate, &error_string);
    emit connectionStatusChange(status, error_string);
}

void CommunicationInterface::disconnectSerial()
{
    serialHandler->disconnectSerial();
}

void CommunicationInterface::ping()
{
    serialHandler->setAutoReadCommandFlag(false);
    stopAllBroadcast();

    // emit propertySetUpdate("Ping Send");
    qint64 elapsed_time = serialHandler->ping(Command::PING);

    // emit propertySetUpdate("Ping received in " + QString::number(elapsed_time) + " ms");
    serialHandler->setAutoReadCommandFlag(true);
    restoreAllBroadcast();
}

void CommunicationInterface::stopAllBroadcast()
{
    serialHandler->sendCommand(Command::STOP_ALL_BROADCAST);
    /*      emit propertySetUpdate("All broadcast stopped");
     else
         emit propertySetUpdate("Failed to stop all broadcast"); */
}

void CommunicationInterface::restoreAllBroadcast()
{
    serialHandler->sendCommand(Command::RESTORE_ALL_BROADCAST);
    // emit propertySetUpdate("All broadcast restored");
    // else
    // emit propertySetUpdate("Failed to restore all broadcast");
}

void CommunicationInterface::sendProperty(Command command, const std::vector<uint8_t> &data, const QString &property_name)
{
    serialHandler->setAutoReadCommandFlag(false);
    stopAllBroadcast();

    if (serialHandler->sendCommand(command))
    {
        if (serialHandler->sendData(data))
        {
            if (serialHandler->readCommand(command))
            {
                if (command == Command::READ_SUCCESS)
                    emit propertySetUpdate(true, property_name + " sent, and set successfully");
                else if (command == Command::READ_FAILURE)
                    emit propertySetUpdate(false, property_name + " sent, but failed to set ");
                else
                    emit propertySetUpdate(false, property_name + " sent, but unexpected value recieved as aknowledgement");
            }
            else
                emit propertySetUpdate(false, "aknowledgement not recived");
        }
        else
            emit propertySetUpdate(false, "Failed to send" + property_name);
    }
    else
        emit propertySetUpdate(false, "Failed to send" + property_name + " command");

    serialHandler->setAutoReadCommandFlag(true);
    restoreAllBroadcast();
    serialHandler->handleReadyRead();
}

void CommunicationInterface::sendControllerProperties(controller_properties_t properties)
{
    num_wheels = properties.numMotors;
    sendProperty(Command::SET_CONTROLLER_PROPERTIES, properties.to_bytes(), "Controller Properties");
}

void CommunicationInterface::sendWheelData(int id, const wheel_data_t wheel_data)
{
    std::vector<uint8_t> data;
    data.reserve(wheel_data.size + 1);
    data.push_back(static_cast<uint8_t>(id)); // Motor ID
    auto motor_data_vec = wheel_data.to_bytes();
    data.insert(data.end(), motor_data_vec.begin(), motor_data_vec.end()); // Motor Data

    sendProperty(Command::SET_MOTOR_DATA, data, "Motor Data (Motor " + QString::number(id + 1) + ")");
}

void CommunicationInterface::sendPIDConstants(int motor_id, int pid_type)
{
    std::vector<uint8_t> data;
    data.reserve(sizeof(pid_constants_t) + 2);
    data.push_back(static_cast<uint8_t>(motor_id)); // Motor ID
    data.push_back(static_cast<uint8_t>(pid_type)); // PID Type

    if (pid_type == 0)
    {
        auto pid_data = controllerData->wheelData.at(motor_id).anglePIDConstants.to_bytes();
        std::copy(pid_data.begin(), pid_data.end(), std::back_inserter(data));
    }
    else
    {
        auto pid_data = controllerData->wheelData.at(motor_id).speedPIDConstants.to_bytes();
        std::copy(pid_data.begin(), pid_data.end(), std::back_inserter(data));
    }

    sendProperty(Command::SET_PID_CONSTANTS, data,
                 "PID Constants (Motor " + QString::number(motor_id) + (pid_type ? " Speed" : " Angle") + ")");
}

void CommunicationInterface::sendOdoBroadcastStatus(int motor_id, odo_broadcast_flags_t flags)
{
    std::vector<uint8_t> data;
    data.reserve(sizeof(odo_broadcast_flags_t) + 1);
    data.push_back(static_cast<uint8_t>(motor_id));

    auto odo_broadcast_flags = flags.to_bytes();
    std::copy(odo_broadcast_flags.begin(), odo_broadcast_flags.end(), std::back_inserter(data));

    sendProperty(Command::SET_ODO_BROADCAST_STATUS, data, "Odometry broadcast status for motor " + QString::number(motor_id));
}

/* void CommunicationInterface::sendSetpoints(ControlMode mode)
{
    if (mode == ControlMode::POSITION_CONTROL)
        sendAngleSetpoint();
    else if (mode == ControlMode::SPEED_CONTROL)
        sendSpeedSetpoint();
    else if (mode == ControlMode::PWM_DIRECT_CONTROL)
        sendPWMSetpoint();
} */

/* void CommunicationInterface::sendAngleSetpoint()
{
    sendSetpointsHelper<angle_t>(
        Command::SET_MOTOR_ANGLE_SETPOINTS,
        [](const wheel_data_t &wheel)
        { return wheel.setpoint.angle; });
}

void CommunicationInterface::sendSpeedSetpoint()
{
    sendSetpointsHelper<angularvelocity_t>(
        Command::SET_MOTOR_SPEED_SETPOINTS,
        [](const wheel_data_t &wheel)
        { return wheel.setpoint.rpm; });
}

void CommunicationInterface::sendPWMSetpoint()
{
    sendSetpointsHelper<pwmvalue_t>(
        Command::SET_MOTOR_PWMS,
        [](const wheel_data_t &wheel)
        { return wheel.pwmValue; });
} */

void CommunicationInterface::sendControlMode(int motor_id, ControlMode mode)
{
    std::vector<uint8_t> data;
    data.reserve(1 + sizeof(ControlMode));
    data.push_back(static_cast<char>(motor_id));
    data.push_back(static_cast<char>(mode));

    sendProperty(Command::SET_MOTOR_CONTROL_MODES, data, "Control Mode (Motor " + QString::number(motor_id) + ")");
}

void CommunicationInterface::getControllerProperties()
{
    serialHandler->sendCommand(Command::GET_CONTROLLER_PROPERTIES);
}

void CommunicationInterface::getMotorData(int id)
{
    serialHandler->sendCommand(Command::GET_MOTOR_DATA);
    serialHandler->sendData(reinterpret_cast<uint8_t *>(&id), sizeof(char));
}

void CommunicationInterface::receiveControllerProperties()
{
    size_t offset = 0;
    auto data = serialHandler->readData(controller_properties_t::size);
    controllerData->controllerProperties.from_bytes(data, offset);
    emit controllerPropertiesReceived();
}

void CommunicationInterface::receiveMotorData()
{
    auto data = serialHandler->readData(wheel_data_t::size + 1);
    uint8_t &motor_id = data[0];
    size_t offset = 1;
    if (data.empty())
    {
        return;
    }
    if (motor_id >= controllerData->wheelData.size())
    {
        return;
    }

    controllerData->wheelData.at(motor_id).from_bytes(data, offset);
    emit motorDataReceived(motor_id);
}

void CommunicationInterface::receiveOdoAngles()
{
    receiveOdoData<angle_t>([](wheel_data_t &wheel, angle_t value)
                            { wheel.odometryData.angle = value; });
}

void CommunicationInterface::receiveOdoSpeeds()
{
    receiveOdoData<angularvelocity_t>([](wheel_data_t &wheel, angularvelocity_t value)
                                      { wheel.odometryData.rpm = value; });
}

void CommunicationInterface::receiveOdoPwms()
{
    receiveOdoData<pwmvalue_t>([](wheel_data_t &wheel, pwmvalue_t value)
                               { wheel.pwmValue = value; });
}

void CommunicationInterface::receiveOdoAnglesTimestamped()
{
    size_t size = sizeof(timestamped_angle_t) * num_wheels;
    std::vector<uint8_t> data = serialHandler->readData(size);
    int i = 0;

    std::vector<timestamped_angle_t> timestamped_angles(num_wheels);
    memcpy(timestamped_angles.data(), data.data(), size);

    emit encoderOdometryReceived(timestamped_angles);
    serialHandler->setAutoReadCommandFlag(true);
}

void CommunicationInterface::commandReceived(char command)
{
    switch (static_cast<Command>(command))
    {
    case Command::SYNC_TIME:
        emit timeSyncReplyReceived();
        break;
    case Command::SEND_ODO_ANGLES:
        receiveOdoAngles();
        break;
    case Command::SEND_ODO_SPEEDS:
        receiveOdoSpeeds();
        break;
    case Command::SEND_ODO_PWMS:
        receiveOdoPwms();
        break;
    case Command::SEND_ODO_TIMESTAMPED_ANGLES:
        receiveOdoAnglesTimestamped();
        break;

    case Command::GET_CONTROLLER_PROPERTIES:
        receiveControllerProperties();
        break;

    case Command::GET_MOTOR_DATA:
        receiveMotorData();
        break;

    default:
        break;
    }
}
