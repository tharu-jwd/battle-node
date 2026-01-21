#include "fusion/EntityTracker.h"
#include <sstream>
#include <iomanip>

namespace SensorFusion
{

    EntityTracker::EntityTracker(uint64_t entityId, EntityType type)
        : entityId_(entityId),
          entityType_(type),
          totalMeasurements_(0),
          baseConfidence_(0.5)
    {

        creationTime_ = std::chrono::high_resolution_clock::now();
        lastUpdateTime_ = creationTime_;
    }

    void EntityTracker::processMeasurement(const SensorMeasurementPtr &measurement)
    {
        auto currentTime = measurement->timestamp;

        if (totalMeasurements_ == 0)
        {
            Eigen::VectorXd initialState(6);
            initialState << measurement->position.x, measurement->position.y, measurement->position.z,
                0.0, 0.0, 0.0;

            if (measurement->hasVelocity)
            {
                initialState(3) = measurement->velocity.vx;
                initialState(4) = measurement->velocity.vy;
                initialState(5) = measurement->velocity.vz;
            }

            Eigen::MatrixXd initialCov = Eigen::MatrixXd::Identity(6, 6);
            initialCov.block<3, 3>(0, 0) = measurement->positionCovariance;
            if (measurement->hasVelocity)
            {
                initialCov.block<3, 3>(3, 3) = measurement->velocityCovariance;
            }
            else
            {
                initialCov.block<3, 3>(3, 3) *= 10.0;
            }

            kalmanFilter_.initialize(initialState, initialCov);
        }
        else
        {
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                          currentTime - lastUpdateTime_)
                          .count() /
                      1e6;

            if (dt > 0.0)
            {
                kalmanFilter_.predict(dt);
            }

            Eigen::VectorXd z(6);
            Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6) * 100.0;

            z << measurement->position.x, measurement->position.y, measurement->position.z,
                measurement->velocity.vx, measurement->velocity.vy, measurement->velocity.vz;

            R.block<3, 3>(0, 0) = measurement->positionCovariance;
            R.block<3, 3>(3, 3) = measurement->velocityCovariance;

            kalmanFilter_.update(z, R, measurement->hasVelocity);
        }

        updateConfidence(measurement->confidence);

        recentSensors_.push_back(measurement->sensorType);
        if (recentSensors_.size() > 10)
        {
            recentSensors_.erase(recentSensors_.begin());
        }

        lastUpdateTime_ = currentTime;
        totalMeasurements_++;
    }

    FusedEntityState EntityTracker::getFusedState() const
    {
        FusedEntityState state;
        state.entityId = entityId_;
        state.entityType = entityType_;
        state.position = kalmanFilter_.getPosition();
        state.velocity = kalmanFilter_.getVelocity();
        state.covariance = kalmanFilter_.getCovariance();
        state.confidence = baseConfidence_;
        state.timestamp = std::chrono::high_resolution_clock::now();
        state.lastUpdateTime = lastUpdateTime_;
        state.contributingSensors = recentSensors_;
        state.measurementCount = totalMeasurements_;

        return state;
    }

    bool EntityTracker::isStale(TimePoint currentTime, Duration maxAge) const
    {
        auto age = std::chrono::duration_cast<Duration>(currentTime - lastUpdateTime_);
        return age > maxAge;
    }

    void EntityTracker::updateConfidence(double measurementConfidence)
    {
        double alpha = 0.1;
        baseConfidence_ = alpha * measurementConfidence + (1.0 - alpha) * baseConfidence_;

        double measurementBonus = std::min(totalMeasurements_ / 100.0, 0.2);
        baseConfidence_ = std::min(baseConfidence_ + measurementBonus, 0.99);
    }

    std::string FusedEntityState::toString() const
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "Entity " << entityId << " [" << entityTypeToString(entityType) << "] "
           << "Pos:(" << position.x << "," << position.y << "," << position.z << ") "
           << "Vel:(" << velocity.vx << "," << velocity.vy << "," << velocity.vz << ") "
           << "Conf:" << (confidence * 100) << "% "
           << "Measurements:" << measurementCount;
        return ss.str();
    }

} // namespace SensorFusion
