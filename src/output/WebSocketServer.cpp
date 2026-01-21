#include "output/WebSocketServer.h"
#include "common/Logger.h"
#include <sstream>
#include <iomanip>

namespace SensorFusion
{

    WebSocketServer::WebSocketServer(int port)
        : port_(port), running_(false)
    {
    }

    WebSocketServer::~WebSocketServer()
    {
        stop();
    }

    void WebSocketServer::start()
    {
        if (running_)
            return;

        running_ = true;
        serverThread_ = std::thread(&WebSocketServer::serverThread, this);

        Logger::getInstance().info("WebSocket server started on port " + std::to_string(port_));
    }

    void WebSocketServer::stop()
    {
        if (!running_)
            return;

        running_ = false;
        if (serverThread_.joinable())
        {
            serverThread_.join();
        }

        Logger::getInstance().info("WebSocket server stopped");
    }

    void WebSocketServer::publishState(const FusedEntityState &state)
    {
        std::string message = serializeState(state);
        broadcast(message);
    }

    void WebSocketServer::publishStates(const std::vector<FusedEntityState> &states)
    {
        std::string message = serializeStates(states);
        broadcast(message);
    }

    std::string WebSocketServer::serializeState(const FusedEntityState &state)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4);
        ss << "{";
        ss << "\"entityId\":" << state.entityId << ",";
        ss << "\"type\":\"" << entityTypeToString(state.entityType) << "\",";
        ss << "\"position\":{\"x\":" << state.position.x << ",\"y\":" << state.position.y
           << ",\"z\":" << state.position.z << "},";
        ss << "\"velocity\":{\"vx\":" << state.velocity.vx << ",\"vy\":" << state.velocity.vy
           << ",\"vz\":" << state.velocity.vz << "},";
        ss << "\"confidence\":" << state.confidence << ",";
        ss << "\"measurements\":" << state.measurementCount;
        ss << "}";
        return ss.str();
    }

    std::string WebSocketServer::serializeStates(const std::vector<FusedEntityState> &states)
    {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < states.size(); ++i)
        {
            ss << serializeState(states[i]);
            if (i < states.size() - 1)
                ss << ",";
        }
        ss << "]";
        return ss.str();
    }

    void WebSocketServer::serverThread()
    {
        while (running_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void WebSocketServer::broadcast(const std::string &message)
    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        messageBuffer_.push_back(message);

        if (messageBuffer_.size() > 100)
        {
            messageBuffer_.erase(messageBuffer_.begin());
        }
    }

} // namespace SensorFusion
