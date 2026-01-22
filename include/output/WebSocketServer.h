#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include "output/OutputInterface.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <deque>
#include <unordered_set>
#include <memory>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace SensorFusion
{

    class Session;

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

        void doAccept();
        void onBroadcastTimer(boost::system::error_code ec);
        void broadcastToSessions(const std::string &message);

        int port_;
        std::atomic<bool> running_;
        std::thread serverThread_;

        net::io_context ioc_;
        tcp::acceptor acceptor_{ioc_};
        std::shared_ptr<net::steady_timer> broadcastTimer_;

        std::mutex sessionsMutex_;
        std::unordered_set<std::shared_ptr<Session>> sessions_;

        std::mutex queueMutex_;
        std::deque<std::string> messageQueue_;

        std::mutex clientsMutex_;
        std::vector<std::string> messageBuffer_;

    public:
        void removeSession(std::shared_ptr<Session> session);
    };

} // namespace SensorFusion

#endif // WEBSOCKET_SERVER_H
