#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include "sensors/SensorData.h"
#include <functional>

namespace SensorFusion
{

    using SensorCallback = std::function<void(const SensorMeasurementPtr &)>;

    class SensorInterface
    {
    public:
        virtual ~SensorInterface() = default;

        virtual void start() = 0;
        virtual void stop() = 0;
        virtual void setCallback(SensorCallback callback) = 0;
        virtual SensorType getType() const = 0;
    };

} // namespace SensorFusion

#endif // SENSOR_INTERFACE_H
