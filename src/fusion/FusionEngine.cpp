#include "fusion/FusionEngine.h"
#include "common/Logger.h"

namespace SensorFusion
{

    FusionEngine::FusionEngine()
        : running_(false),
          staleTimeout_(std::chrono::seconds(10)),
          outputRateHz_(10.0)
    {
    }

    FusionEngine::~FusionEngine()
    {
        stop();
    }

    void FusionEngine::start()
    {
        if (running_)
            return;

        running_ = true;
        fusionThread_ = std::thread(&FusionEngine::fusionThread, this);
        outputThread_ = std::thread(&FusionEngine::outputThread, this);

        Logger::getInstance().info("Fusion Engine started");
    }

    void FusionEngine::stop()
    {
        if (!running_)
            return;

        running_ = false;
        measurementQueue_.shutdown();

        if (fusionThread_.joinable())
        {
            fusionThread_.join();
        }
        if (outputThread_.joinable())
        {
            outputThread_.join();
        }

        Logger::getInstance().info("Fusion Engine stopped");
    }

    void FusionEngine::ingestMeasurement(const SensorMeasurementPtr &measurement)
    {
        measurementQueue_.push(measurement);
    }

    void FusionEngine::setOutputCallback(FusedStateCallback callback)
    {
        outputCallback_ = callback;
    }

    std::vector<FusedEntityState> FusionEngine::getAllEntityStates() const
    {
        std::lock_guard<std::mutex> lock(trackersMutex_);

        std::vector<FusedEntityState> states;
        states.reserve(entityTrackers_.size());

        for (const auto &[entityId, tracker] : entityTrackers_)
        {
            states.push_back(tracker->getFusedState());
        }

        return states;
    }

    void FusionEngine::setStaleEntityTimeout(Duration timeout)
    {
        staleTimeout_ = timeout;
    }

    void FusionEngine::setOutputRateHz(double rateHz)
    {
        outputRateHz_ = rateHz;
    }

    void FusionEngine::fusionThread()
    {
        while (running_)
        {
            auto measurement = measurementQueue_.pop();

            if (!measurement.has_value())
            {
                break;
            }

            auto m = measurement.value();

            std::lock_guard<std::mutex> lock(trackersMutex_);

            auto it = entityTrackers_.find(m->entityId);
            if (it == entityTrackers_.end())
            {
                auto tracker = std::make_shared<EntityTracker>(m->entityId, EntityType::VEHICLE);
                entityTrackers_[m->entityId] = tracker;

                Logger::getInstance().info("Created new tracker for entity " +
                                           std::to_string(m->entityId));
            }

            entityTrackers_[m->entityId]->processMeasurement(m);
        }
    }

    void FusionEngine::outputThread()
    {
        auto period = std::chrono::microseconds(static_cast<long>(1e6 / outputRateHz_));
        auto nextOutput = std::chrono::high_resolution_clock::now();

        while (running_)
        {
            auto currentTime = std::chrono::high_resolution_clock::now();

            if (currentTime >= nextOutput)
            {
                cleanupStaleEntities();

                auto states = getAllEntityStates();

                if (outputCallback_)
                {
                    for (const auto &state : states)
                    {
                        outputCallback_(state);
                    }
                }

                nextOutput += period;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void FusionEngine::cleanupStaleEntities()
    {
        std::lock_guard<std::mutex> lock(trackersMutex_);

        auto currentTime = std::chrono::high_resolution_clock::now();

        auto it = entityTrackers_.begin();
        while (it != entityTrackers_.end())
        {
            if (it->second->isStale(currentTime, staleTimeout_))
            {
                Logger::getInstance().info("Removed stale entity " +
                                           std::to_string(it->first));
                it = entityTrackers_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

} // namespace SensorFusion
