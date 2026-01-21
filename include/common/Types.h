#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <chrono>
#include <memory>

namespace SensorFusion
{

    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Duration = std::chrono::microseconds;

    enum class SensorType
    {
        GPS,
        VISION,
        RF,
        RADAR,
        LIDAR,
        UNKNOWN
    };

    enum class EntityType
    {
        VEHICLE,
        AIRCRAFT,
        PERSONNEL,
        UNKNOWN
    };

    struct Position3D
    {
        double x;
        double y;
        double z;

        Position3D() : x(0.0), y(0.0), z(0.0) {}
        Position3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    struct Velocity3D
    {
        double vx;
        double vy;
        double vz;

        Velocity3D() : vx(0.0), vy(0.0), vz(0.0) {}
        Velocity3D(double vx_, double vy_, double vz_) : vx(vx_), vy(vy_), vz(vz_) {}
    };

    std::string sensorTypeToString(SensorType type);
    std::string entityTypeToString(EntityType type);

} // namespace SensorFusion

#endif // TYPES_H
