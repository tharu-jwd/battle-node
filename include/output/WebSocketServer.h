#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include "output/OutputInterface.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace SensorFusion
{

    class WebSocketServer : public OutputInterface
    {
    public:
        WebSocketServer(int port);
        ~WebSocketServer();

        void start() override;
        void stop() override;
        void publishState(const FusedEntityState &state) override;
        void publishStates(const std::vector<FusedEntityState> &states) override;

    private:
        std::string serializeState(const FusedEntityState &state);
        std::string serializeStates(const std::vector<FusedEntityState> &states);

        void serverThread();
        void broadcast(const std::string &message);

        int port_;
        std::atomic<bool> running_;
        std::thread serverThread_;

        std::mutex clientsMutex_;
        std::vector<std::string> messageBuffer_;
    };

} // namespace SensorFusion

#endif // WEBSOCKET_SERVER_H
