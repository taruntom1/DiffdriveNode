#include <QCoreApplication>
#include <QThread>
#include <rclcpp/rclcpp.hpp>

#include "serialhandler.h"
#include "controllermanager.h"
#include "communicationinterface.h"
#include "timesyncclient.h"
#include "rosworker.h"

int main(int argc, char *argv[])
{
    // --- 1. Initialize ROS 2 and Qt Application ---
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    // --- 3. Start ROS Worker ---
    RosWorker *rosWorker = new RosWorker;
    rosWorker->start();

    // --- Clock for timing ---
    rclcpp::Clock clock(RCL_SYSTEM_TIME);

    // --- 4. Set up Serial Communication in a Separate Thread ---
    QThread *commThread = new QThread;
    SerialHandler *serialHandler = new SerialHandler;
    CommunicationInterface *commInterface =
        new CommunicationInterface(nullptr, serialHandler);
    TimeSyncClient *timeSyncClient = new TimeSyncClient(serialHandler, commInterface, &clock);

    serialHandler->moveToThread(commThread);
    commInterface->moveToThread(commThread);
    timeSyncClient->moveToThread(commThread);
    commThread->start();

    // --- controller manager ---
    ControllerManager *controllerManager = new ControllerManager(commInterface, timeSyncClient);

    // --- 5. Synchronize ROS and Qt Shutdowns ---
    rclcpp::on_shutdown([&app]()
                        { app.quit(); });
    QObject::connect(&app, &QCoreApplication::aboutToQuit, []()
                     {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        } });

    // --- 6. Execute Qt Event Loop ---
    int ret = app.exec();

    // --- 7. Clean Up (Delete in Reverse Order of Creation) ---
    rosWorker->quit();
    rosWorker->wait();

    commThread->quit();
    commThread->wait();

    delete commInterface;
    delete serialHandler;
    delete commThread;
    delete timeSyncClient;
    delete controllerManager;
    delete rosWorker;

    return ret;
}
