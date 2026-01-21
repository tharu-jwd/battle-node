#include "system/SensorFusionSystem.h"
#include "sensors/SyntheticSensorGenerator.h"
#include "fusion/FusionEngine.h"
#include "output/CLIVisualizer.h"
#include "common/Logger.h"
#include <iostream>

using namespace SensorFusion;

int main()
{
    Logger::getInstance().setLogLevel(LogLevel::INFO);

    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     BATTLEFIELD SENSOR FUSION - PORTFOLIO DEMONSTRATION      ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
    std::cout << "This demo showcases:\n";
    std::cout << "  • Multi-sensor data ingestion (GPS, Radar, Vision)\n";
    std::cout << "  • Real-time Kalman filter fusion\n";
    std::cout << "  • Multithreaded architecture\n";
    std::cout << "  • Simulated noisy sensor data with dropouts\n";
    std::cout << "  • Live entity tracking and state estimation\n";
    std::cout << "\n";
    std::cout << "Starting demo in 3 seconds...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto system = std::make_shared<SensorFusionSystem>();
    auto fusionEngine = std::make_shared<FusionEngine>();
    fusionEngine->setOutputRateHz(2.0);
    system->setFusionEngine(fusionEngine);

    auto visualizer = std::make_shared<CLIVisualizer>(true);
    system->addOutputInterface(visualizer);

    auto gpsSensor = std::make_shared<SyntheticSensorGenerator>(
        SensorType::GPS, 1.0, 4.0);
    gpsSensor->setDropoutProbability(0.08);

    auto radarSensor = std::make_shared<SyntheticSensorGenerator>(
        SensorType::RADAR, 4.0, 2.5);
    radarSensor->setDropoutProbability(0.12);

    EntityTrajectory tank;
    tank.entityId = 201;
    tank.type = EntityType::VEHICLE;
    tank.initialPosition = Position3D(0.0, 0.0, 0.0);
    tank.velocity = Velocity3D(12.0, 8.0, 0.0);

    EntityTrajectory helicopter;
    helicopter.entityId = 202;
    helicopter.type = EntityType::AIRCRAFT;
    helicopter.initialPosition = Position3D(150.0, 100.0, 80.0);
    helicopter.velocity = Velocity3D(-18.0, -10.0, 1.5);

    gpsSensor->addEntity(tank);
    gpsSensor->addEntity(helicopter);
    radarSensor->addEntity(tank);
    radarSensor->addEntity(helicopter);

    system->addSensor(gpsSensor);
    system->addSensor(radarSensor);

    system->start();

    std::cout << "\nRunning demo for 30 seconds...\n\n";

    for (int i = 0; i < 30; ++i)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    system->stop();

    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    DEMO COMPLETED                             ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";

    return 0;
}
