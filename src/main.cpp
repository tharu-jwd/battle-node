#include "system/SensorFusionSystem.h"
#include "sensors/SyntheticSensorGenerator.h"
#include "fusion/FusionEngine.h"
#include "output/CLIVisualizer.h"
#include "output/WebSocketServer.h"
#include "common/Logger.h"
#include <iostream>
#include <csignal>

using namespace SensorFusion;

std::atomic<bool> keepRunning(true);

void signalHandler(int)
{
    keepRunning = false;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    Logger::getInstance().setLogLevel(LogLevel::INFO);
    Logger::getInstance().setLogFile("sensor_fusion.log");

    Logger::getInstance().info("Initializing Battle-Node System");

    auto system = std::make_shared<SensorFusionSystem>();
    auto fusionEngine = std::make_shared<FusionEngine>();
    fusionEngine->setOutputRateHz(5.0);
    fusionEngine->setStaleEntityTimeout(std::chrono::seconds(15));

    system->setFusionEngine(fusionEngine);

    auto cliVisualizer = std::make_shared<CLIVisualizer>(true);
    cliVisualizer->setVerbose(true);
    system->addOutputInterface(cliVisualizer);

    auto wsServer = std::make_shared<WebSocketServer>(8080);
    system->addOutputInterface(wsServer);

    auto gpsSensor = std::make_shared<SyntheticSensorGenerator>(
        SensorType::GPS, 1.0, 5.0);
    gpsSensor->setDropoutProbability(0.05);

    auto radarSensor = std::make_shared<SyntheticSensorGenerator>(
        SensorType::RADAR, 5.0, 3.0);
    radarSensor->setDropoutProbability(0.10);
    radarSensor->setDelayMs(10, 50);

    auto visionSensor = std::make_shared<SyntheticSensorGenerator>(
        SensorType::VISION, 10.0, 8.0);
    visionSensor->setDropoutProbability(0.15);

    EntityTrajectory entity1;
    entity1.entityId = 101;
    entity1.type = EntityType::VEHICLE;
    entity1.initialPosition = Position3D(0.0, 0.0, 0.0);
    entity1.velocity = Velocity3D(15.0, 10.0, 0.0);

    EntityTrajectory entity2;
    entity2.entityId = 102;
    entity2.type = EntityType::AIRCRAFT;
    entity2.initialPosition = Position3D(100.0, 200.0, 50.0);
    entity2.velocity = Velocity3D(-20.0, 5.0, 2.0);

    EntityTrajectory entity3;
    entity3.entityId = 103;
    entity3.type = EntityType::VEHICLE;
    entity3.initialPosition = Position3D(-50.0, 100.0, 0.0);
    entity3.velocity = Velocity3D(8.0, -12.0, 0.0);

    gpsSensor->addEntity(entity1);
    gpsSensor->addEntity(entity2);
    gpsSensor->addEntity(entity3);

    radarSensor->addEntity(entity1);
    radarSensor->addEntity(entity2);
    radarSensor->addEntity(entity3);

    visionSensor->addEntity(entity1);
    visionSensor->addEntity(entity3);

    system->addSensor(gpsSensor);
    system->addSensor(radarSensor);
    system->addSensor(visionSensor);

    system->start();

    std::cout << "\nBattle-Node System Running...\n";
    std::cout << "Press Ctrl+C to stop\n\n";

    while (keepRunning)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "\nShutting down...\n";
    system->stop();

    Logger::getInstance().info("System shutdown complete");

    return 0;
}
