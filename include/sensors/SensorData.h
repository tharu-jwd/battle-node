#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "common/Types.h"
#include <memory>

namespace SensorFusion
{

    struct SensorMeasurement
    {
        uint64_t entityId;
        SensorType sensorType;
        TimePoint timestamp;

        Position3D position;
        Velocity3D velocity;

        double confidence;
        bool hasVelocity;

        Eigen::Matrix3d positionCovariance;
        Eigen::Matrix3d velocityCovariance;

        SensorMeasurement();

        std::string toString() const;
    };

    using SensorMeasurementPtr = std::shared_ptr<SensorMeasurement>;

} // namespace SensorFusion

#endif // SENSOR_DATA_H
