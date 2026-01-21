#include "sensors/SensorData.h"
#include <sstream>
#include <iomanip>

namespace SensorFusion
{

    std::string sensorTypeToString(SensorType type)
    {
        switch (type)
        {
        case SensorType::GPS:
            return "GPS";
        case SensorType::VISION:
            return "VISION";
        case SensorType::RF:
            return "RF";
        case SensorType::RADAR:
            return "RADAR";
        case SensorType::LIDAR:
            return "LIDAR";
        default:
            return "UNKNOWN";
        }
    }

    std::string entityTypeToString(EntityType type)
    {
        switch (type)
        {
        case EntityType::VEHICLE:
            return "VEHICLE";
        case EntityType::AIRCRAFT:
            return "AIRCRAFT";
        case EntityType::PERSONNEL:
            return "PERSONNEL";
        default:
            return "UNKNOWN";
        }
    }

    SensorMeasurement::SensorMeasurement()
        : entityId(0),
          sensorType(SensorType::UNKNOWN),
          confidence(0.0),
          hasVelocity(false)
    {

        positionCovariance = Eigen::Matrix3d::Identity();
        velocityCovariance = Eigen::Matrix3d::Identity();
    }

    std::string SensorMeasurement::toString() const
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "Entity:" << entityId
           << " Sensor:" << sensorTypeToString(sensorType)
           << " Pos:(" << position.x << "," << position.y << "," << position.z << ")"
           << " Conf:" << confidence;
        return ss.str();
    }

} // namespace SensorFusion
