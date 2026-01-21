#ifndef SENSOR_FUSION_SYSTEM_H
#define SENSOR_FUSION_SYSTEM_H

#include "sensors/SensorInterface.h"
#include "fusion/FusionEngine.h"
#include "output/OutputInterface.h"
#include <vector>
#include <memory>

namespace SensorFusion
{

    class SensorFusionSystem
    {
    public:
        SensorFusionSystem();
        ~SensorFusionSystem();

        void addSensor(std::shared_ptr<SensorInterface> sensor);
        void addOutputInterface(std::shared_ptr<OutputInterface> output);

        void setFusionEngine(std::shared_ptr<FusionEngine> engine);

        void start();
        void stop();

        bool isRunning() const { return running_; }

    private:
        void onSensorMeasurement(const SensorMeasurementPtr &measurement);
        void onFusedState(const FusedEntityState &state);

        std::vector<std::shared_ptr<SensorInterface>> sensors_;
        std::vector<std::shared_ptr<OutputInterface>> outputs_;
        std::shared_ptr<FusionEngine> fusionEngine_;

        bool running_;
    };

} // namespace SensorFusion

#endif // SENSOR_FUSION_SYSTEM_H
