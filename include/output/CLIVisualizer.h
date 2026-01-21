#ifndef CLI_VISUALIZER_H
#define CLI_VISUALIZER_H

#include "output/OutputInterface.h"
#include <mutex>
#include <map>

namespace SensorFusion
{

    class CLIVisualizer : public OutputInterface
    {
    public:
        CLIVisualizer(bool enableColors = true);

        void start() override;
        void stop() override;
        void publishState(const FusedEntityState &state) override;
        void publishStates(const std::vector<FusedEntityState> &states) override;

        void setVerbose(bool verbose);
        void clearScreen();

    private:
        void displayState(const FusedEntityState &state);
        void displayHeader();
        void displaySummary(const std::vector<FusedEntityState> &states);

        std::string formatPosition(const Position3D &pos);
        std::string formatVelocity(const Velocity3D &vel);
        std::string formatConfidence(double confidence);

        bool enableColors_;
        bool verbose_;
        std::mutex displayMutex_;

        std::map<uint64_t, FusedEntityState> latestStates_;
    };

} // namespace SensorFusion

#endif // CLI_VISUALIZER_H
