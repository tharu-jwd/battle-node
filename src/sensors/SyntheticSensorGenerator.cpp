#include "sensors/SyntheticSensorGenerator.h"
#include "common/Logger.h"
#include <cmath>

namespace SensorFusion
{

    SyntheticSensorGenerator::SyntheticSensorGenerator(SensorType type, double updateRateHz, double noiseStdDev)
        : sensorType_(type),
          updateRateHz_(updateRateHz),
          noiseStdDev_(noiseStdDev),
          dropoutProb_(0.0),
          minDelayMs_(0),
          maxDelayMs_(0),
          running_(false),
          rng_(std::random_device{}())
    {
    }

    SyntheticSensorGenerator::~SyntheticSensorGenerator()
    {
        stop();
    }

    void SyntheticSensorGenerator::start()
    {
        if (running_)
            return;

        running_ = true;
        startTime_ = std::chrono::high_resolution_clock::now();
        generatorThread_ = std::thread(&SyntheticSensorGenerator::generatorThread, this);

        Logger::getInstance().info("Started " + sensorTypeToString(sensorType_) + " generator");
    }

    void SyntheticSensorGenerator::stop()
    {
        if (!running_)
            return;

        running_ = false;
        if (generatorThread_.joinable())
        {
            generatorThread_.join();
        }

        Logger::getInstance().info("Stopped " + sensorTypeToString(sensorType_) + " generator");
    }

    void SyntheticSensorGenerator::setCallback(SensorCallback callback)
    {
        callback_ = callback;
    }

    void SyntheticSensorGenerator::addEntity(const EntityTrajectory &trajectory)
    {
        entities_.push_back(trajectory);
    }

    void SyntheticSensorGenerator::setDropoutProbability(double prob)
    {
        dropoutProb_ = std::clamp(prob, 0.0, 1.0);
    }

    void SyntheticSensorGenerator::setDelayMs(int minMs, int maxMs)
    {
        minDelayMs_ = minMs;
        maxDelayMs_ = maxMs;
    }

    void SyntheticSensorGenerator::generatorThread()
    {
        auto updatePeriod = std::chrono::microseconds(static_cast<long>(1e6 / updateRateHz_));
        auto nextUpdate = std::chrono::high_resolution_clock::now();

        while (running_)
        {
            auto currentTime = std::chrono::high_resolution_clock::now();

            if (currentTime >= nextUpdate)
            {
                for (const auto &entity : entities_)
                {
                    std::uniform_real_distribution<double> dropoutDist(0.0, 1.0);
                    if (dropoutDist(rng_) < dropoutProb_)
                    {
                        continue;
                    }

                    auto measurement = generateMeasurement(entity, currentTime);

                    if (callback_)
                    {
                        if (maxDelayMs_ > minDelayMs_)
                        {
                            std::uniform_int_distribution<int> delayDist(minDelayMs_, maxDelayMs_);
                            int delay = delayDist(rng_);
                            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                        }
                        callback_(measurement);
                    }
                }

                nextUpdate += updatePeriod;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    SensorMeasurementPtr SyntheticSensorGenerator::generateMeasurement(
        const EntityTrajectory &traj, TimePoint currentTime)
    {

        auto measurement = std::make_shared<SensorMeasurement>();
        measurement->entityId = traj.entityId;
        measurement->sensorType = sensorType_;
        measurement->timestamp = currentTime;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           currentTime - startTime_)
                           .count() /
                       1000.0;

        std::normal_distribution<double> noiseDist(0.0, noiseStdDev_);

        measurement->position.x = traj.initialPosition.x + traj.velocity.vx * elapsed + noiseDist(rng_);
        measurement->position.y = traj.initialPosition.y + traj.velocity.vy * elapsed + noiseDist(rng_);
        measurement->position.z = traj.initialPosition.z + traj.velocity.vz * elapsed + noiseDist(rng_);

        double variance = noiseStdDev_ * noiseStdDev_;
        measurement->positionCovariance = Eigen::Matrix3d::Identity() * variance;

        if (sensorType_ == SensorType::RADAR || sensorType_ == SensorType::LIDAR)
        {
            measurement->hasVelocity = true;
            measurement->velocity.vx = traj.velocity.vx + noiseDist(rng_) * 0.1;
            measurement->velocity.vy = traj.velocity.vy + noiseDist(rng_) * 0.1;
            measurement->velocity.vz = traj.velocity.vz + noiseDist(rng_) * 0.1;
            measurement->velocityCovariance = Eigen::Matrix3d::Identity() * (variance * 0.01);
        }
        else
        {
            measurement->hasVelocity = false;
        }

        switch (sensorType_)
        {
        case SensorType::GPS:
            measurement->confidence = 0.95;
            break;
        case SensorType::RADAR:
            measurement->confidence = 0.85;
            break;
        case SensorType::VISION:
            measurement->confidence = 0.75;
            break;
        case SensorType::LIDAR:
            measurement->confidence = 0.90;
            break;
        default:
            measurement->confidence = 0.70;
        }

        return measurement;
    }

} // namespace SensorFusion
