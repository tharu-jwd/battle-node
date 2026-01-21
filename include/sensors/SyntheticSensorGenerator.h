#ifndef SYNTHETIC_SENSOR_GENERATOR_H
#define SYNTHETIC_SENSOR_GENERATOR_H

#include "sensors/SensorInterface.h"
#include "common/ThreadSafeQueue.h"
#include <thread>
#include <atomic>
#include <random>
#include <vector>

namespace SensorFusion
{

    struct EntityTrajectory
    {
        uint64_t entityId;
        EntityType type;
        Position3D initialPosition;
        Velocity3D velocity;
        double heading;
    };

    class SyntheticSensorGenerator : public SensorInterface
    {
    public:
        SyntheticSensorGenerator(SensorType type, double updateRateHz, double noiseStdDev);
        ~SyntheticSensorGenerator();

        void start() override;
        void stop() override;
        void setCallback(SensorCallback callback) override;
        SensorType getType() const override { return sensorType_; }

        void addEntity(const EntityTrajectory &trajectory);
        void setDropoutProbability(double prob);
        void setDelayMs(int minMs, int maxMs);

    private:
        void generatorThread();
        SensorMeasurementPtr generateMeasurement(const EntityTrajectory &traj, TimePoint currentTime);

        SensorType sensorType_;
        double updateRateHz_;
        double noiseStdDev_;
        double dropoutProb_;
        int minDelayMs_;
        int maxDelayMs_;

        std::vector<EntityTrajectory> entities_;
        SensorCallback callback_;

        std::atomic<bool> running_;
        std::thread generatorThread_;

        std::mt19937 rng_;
        TimePoint startTime_;
    };

} // namespace SensorFusion

#endif // SYNTHETIC_SENSOR_GENERATOR_H
