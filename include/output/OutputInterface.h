#ifndef OUTPUT_INTERFACE_H
#define OUTPUT_INTERFACE_H

#include "fusion/EntityTracker.h"
#include <vector>

namespace SensorFusion
{

    class OutputInterface
    {
    public:
        virtual ~OutputInterface() = default;

        virtual void start() = 0;
        virtual void stop() = 0;
        virtual void publishState(const FusedEntityState &state) = 0;
        virtual void publishStates(const std::vector<FusedEntityState> &states) = 0;
    };

} // namespace SensorFusion

#endif // OUTPUT_INTERFACE_H
