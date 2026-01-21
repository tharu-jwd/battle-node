#ifndef ENTITY_TRACKER_H
#define ENTITY_TRACKER_H

#include "fusion/KalmanFilter.h"
#include "sensors/SensorData.h"
#include <memory>

namespace SensorFusion
{

    struct FusedEntityState
    {
        uint64_t entityId;
        EntityType entityType;

        Position3D position;
        Velocity3D velocity;

        Eigen::MatrixXd covariance;

        double confidence;
        TimePoint timestamp;
        TimePoint lastUpdateTime;

        std::vector<SensorType> contributingSensors;
        int measurementCount;

        std::string toString() const;
    };

    class EntityTracker
    {
    public:
        EntityTracker(uint64_t entityId, EntityType type);

        void processMeasurement(const SensorMeasurementPtr &measurement);

        FusedEntityState getFusedState() const;

        bool isStale(TimePoint currentTime, Duration maxAge) const;

        uint64_t getEntityId() const { return entityId_; }

    private:
        uint64_t entityId_;
        EntityType entityType_;

        KalmanFilter kalmanFilter_;

        TimePoint lastUpdateTime_;
        TimePoint creationTime_;

        std::vector<SensorType> recentSensors_;
        int totalMeasurements_;

        double baseConfidence_;

        void updateConfidence(double measurementConfidence);
    };

    using EntityTrackerPtr = std::shared_ptr<EntityTracker>;

} // namespace SensorFusion

#endif // ENTITY_TRACKER_H
