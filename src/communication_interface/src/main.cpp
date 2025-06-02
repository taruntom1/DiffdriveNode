#include <QCoreApplication>
#include <QThread>
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "serialhandler.h"
#include "controllermanager.h"
#include "odometry.h"
#include "communicationinterface.h"
#include "timesyncclient.h"
#include "rosworker.h"

int main(int argc, char *argv[])
{
    // Initialize ROS2 and Qt core application
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    // --- ROS Worker Thread ---
    RosWorker *rosWorker = new RosWorker;
    rosWorker->start();

    // --- Communication Thread and Components ---
    QThread *commThread = new QThread;

    SerialHandler *serialHandler = new SerialHandler;
    CommunicationInterface *commInterface = new CommunicationInterface(nullptr, serialHandler);

    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    TimeSyncClient *timeSyncClient = new TimeSyncClient(serialHandler, commInterface, &clock);

    // Move components to the communication thread
    serialHandler->moveToThread(commThread);
    commInterface->moveToThread(commThread);
    timeSyncClient->moveToThread(commThread);
    commThread->start();

    // --- Application Modules ---
    ControllerManager *controllerManager = new ControllerManager(commInterface, timeSyncClient);
    Odometry *odometry = new Odometry(commInterface, rosWorker);

    // --- Graceful Shutdown Handling ---
    rclcpp::on_shutdown([&app]() {
        app.quit();
    });

    QObject::connect(&app, &QCoreApplication::aboutToQuit, []() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    });

    // --- Execute Application Loop ---
    int ret = app.exec();

    // --- Cleanup ---
    rosWorker->quit();
    rosWorker->wait();

    commThread->quit();
    commThread->wait();

    delete odometry;
    delete controllerManager;
    delete timeSyncClient;
    delete commInterface;
    delete serialHandler;
    delete commThread;
    delete rosWorker;

    return ret;
}
