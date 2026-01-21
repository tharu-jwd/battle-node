#ifndef FUSION_ENGINE_H
#define FUSION_ENGINE_H

#include "fusion/EntityTracker.h"
#include "common/ThreadSafeQueue.h"
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace SensorFusion
{

    using FusedStateCallback = std::function<void(const FusedEntityState &)>;

    class FusionEngine
    {
    public:
        FusionEngine();
        ~FusionEngine();

        void start();
        void stop();

        void ingestMeasurement(const SensorMeasurementPtr &measurement);

        void setOutputCallback(FusedStateCallback callback);

        std::vector<FusedEntityState> getAllEntityStates() const;

        void setStaleEntityTimeout(Duration timeout);
        void setOutputRateHz(double rateHz);

    private:
        void fusionThread();
        void outputThread();
        void cleanupStaleEntities();

        std::map<uint64_t, EntityTrackerPtr> entityTrackers_;
        mutable std::mutex trackersMutex_;

        ThreadSafeQueue<SensorMeasurementPtr> measurementQueue_;

        FusedStateCallback outputCallback_;

        std::atomic<bool> running_;
        std::thread fusionThread_;
        std::thread outputThread_;

        Duration staleTimeout_;
        double outputRateHz_;
    };

} // namespace SensorFusion

#endif // FUSION_ENGINE_H
