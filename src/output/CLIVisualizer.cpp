#include "output/CLIVisualizer.h"
#include <iostream>
#include <iomanip>
#include <sstream>

namespace SensorFusion
{

    CLIVisualizer::CLIVisualizer(bool enableColors)
        : enableColors_(enableColors), verbose_(false)
    {
    }

    void CLIVisualizer::start()
    {
        displayHeader();
    }

    void CLIVisualizer::stop()
    {
    }

    void CLIVisualizer::publishState(const FusedEntityState &state)
    {
        std::lock_guard<std::mutex> lock(displayMutex_);
        latestStates_[state.entityId] = state;

        if (verbose_)
        {
            displayState(state);
        }
    }

    void CLIVisualizer::publishStates(const std::vector<FusedEntityState> &states)
    {
        std::lock_guard<std::mutex> lock(displayMutex_);

        for (const auto &state : states)
        {
            latestStates_[state.entityId] = state;
        }

        displaySummary(states);
    }

    void CLIVisualizer::setVerbose(bool verbose)
    {
        verbose_ = verbose;
    }

    void CLIVisualizer::clearScreen()
    {
        std::cout << "\033[2J\033[1;1H";
    }

    void CLIVisualizer::displayState(const FusedEntityState &state)
    {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "[Entity " << std::setw(3) << state.entityId << "] "
                  << std::setw(10) << entityTypeToString(state.entityType) << " | "
                  << "Pos: " << formatPosition(state.position) << " | "
                  << "Vel: " << formatVelocity(state.velocity) << " | "
                  << "Conf: " << formatConfidence(state.confidence) << " | "
                  << "Meas: " << std::setw(4) << state.measurementCount
                  << std::endl;
    }

    void CLIVisualizer::displayHeader()
    {
        std::cout << "\n";
        std::cout << "═══════════════════════════════════════════════════════════════════════════\n";
        std::cout << "                    BATTLE-NODE - REAL-TIME TRACKER                        \n";
        std::cout << "═══════════════════════════════════════════════════════════════════════════\n";
        std::cout << std::endl;
    }

    void CLIVisualizer::displaySummary(const std::vector<FusedEntityState> &states)
    {
        clearScreen();
        displayHeader();

        std::cout << "Active Entities: " << states.size() << "\n";
        std::cout << "───────────────────────────────────────────────────────────────────────────\n";
        std::cout << std::left << std::setw(6) << "ID"
                  << std::setw(12) << "Type"
                  << std::setw(30) << "Position (x,y,z)"
                  << std::setw(30) << "Velocity (vx,vy,vz)"
                  << std::setw(10) << "Conf%"
                  << std::setw(8) << "Meas"
                  << "\n";
        std::cout << "───────────────────────────────────────────────────────────────────────────\n";

        for (const auto &state : states)
        {
            std::cout << std::left << std::setw(6) << state.entityId
                      << std::setw(12) << entityTypeToString(state.entityType)
                      << std::setw(30) << formatPosition(state.position)
                      << std::setw(30) << formatVelocity(state.velocity)
                      << std::setw(10) << formatConfidence(state.confidence)
                      << std::setw(8) << state.measurementCount
                      << "\n";
        }

        std::cout << "═══════════════════════════════════════════════════════════════════════════\n";
        std::cout << std::flush;
    }

    std::string CLIVisualizer::formatPosition(const Position3D &pos)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "(" << std::setw(7) << pos.x << ", "
           << std::setw(7) << pos.y << ", "
           << std::setw(7) << pos.z << ")";
        return ss.str();
    }

    std::string CLIVisualizer::formatVelocity(const Velocity3D &vel)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "(" << std::setw(6) << vel.vx << ", "
           << std::setw(6) << vel.vy << ", "
           << std::setw(6) << vel.vz << ")";
        return ss.str();
    }

    std::string CLIVisualizer::formatConfidence(double confidence)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << (confidence * 100.0) << "%";
        return ss.str();
    }

} // namespace SensorFusion
