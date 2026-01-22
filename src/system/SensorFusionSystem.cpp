#include "system/SensorFusionSystem.h"
#include "common/Logger.h"

namespace SensorFusion
{

    SensorFusionSystem::SensorFusionSystem() : running_(false)
    {
    }

    SensorFusionSystem::~SensorFusionSystem()
    {
        stop();
    }

    void SensorFusionSystem::addSensor(std::shared_ptr<SensorInterface> sensor)
    {
        sensors_.push_back(sensor);
    }

    void SensorFusionSystem::addOutputInterface(std::shared_ptr<OutputInterface> output)
    {
        outputs_.push_back(output);
    }

    void SensorFusionSystem::setFusionEngine(std::shared_ptr<FusionEngine> engine)
    {
        fusionEngine_ = engine;
    }

    void SensorFusionSystem::start()
    {
        if (running_)
            return;

        Logger::getInstance().info("Starting Battle-Node System...");

        if (fusionEngine_)
        {
            fusionEngine_->setOutputCallback(
                [this](const FusedEntityState &state)
                {
                    this->onFusedState(state);
                });
            fusionEngine_->start();
        }

        for (auto &output : outputs_)
        {
            output->start();
        }

        for (auto &sensor : sensors_)
        {
            sensor->setCallback(
                [this](const SensorMeasurementPtr &measurement)
                {
                    this->onSensorMeasurement(measurement);
                });
            sensor->start();
        }

        running_ = true;
        Logger::getInstance().info("Battle-Node System started successfully");
    }

    void SensorFusionSystem::stop()
    {
        if (!running_)
            return;

        Logger::getInstance().info("Stopping Battle-Node System...");

        for (auto &sensor : sensors_)
        {
            sensor->stop();
        }

        if (fusionEngine_)
        {
            fusionEngine_->stop();
        }

        for (auto &output : outputs_)
        {
            output->stop();
        }

        running_ = false;
        Logger::getInstance().info("Battle-Node System stopped");
    }

    void SensorFusionSystem::onSensorMeasurement(const SensorMeasurementPtr &measurement)
    {
        if (fusionEngine_)
        {
            fusionEngine_->ingestMeasurement(measurement);
        }
    }

    void SensorFusionSystem::onFusedState(const FusedEntityState &state)
    {
        for (auto &output : outputs_)
        {
            output->publishState(state);
        }
    }

} // namespace SensorFusion
