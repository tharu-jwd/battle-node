#include "output/WebSocketServer.h"
#include "common/Logger.h"
#include <sstream>
#include <iomanip>
#include <functional>

namespace SensorFusion
{

    class Session : public std::enable_shared_from_this<Session>
    {
    public:
        Session(tcp::socket socket, WebSocketServer *server)
            : ws_(std::move(socket)), server_(server) {}

        void run()
        {
            ws_.async_accept([self = shared_from_this()](beast::error_code ec)
                             {
                if (ec) {
                    Logger::getInstance().error("WebSocket handshake failed: " + ec.message());
                    return;
                }
                self->doRead(); });
        }

        void send(std::string message)
        {
            std::lock_guard<std::mutex> lock(writeMutex_);
            writeQueue_.push_back(std::move(message));
            if (writeQueue_.size() > 50)
            {
                writeQueue_.pop_front();
            }
            if (!writing_)
            {
                doWrite();
            }
        }

        void close()
        {
            boost::system::error_code ec;
            ws_.close(websocket::close_code::normal, ec);
        }

    private:
        void doRead()
        {
            ws_.async_read(buffer_, [self = shared_from_this()](
                                         beast::error_code ec, std::size_t bytes)
                           {
                if (ec == websocket::error::closed) {
                    self->server_->removeSession(self);
                    return;
                }
                if (ec) {
                    Logger::getInstance().warning("WebSocket read error: " + ec.message());
                    self->server_->removeSession(self);
                    return;
                }
                self->buffer_.consume(bytes);
                self->doRead(); });
        }

        void doWrite()
        {
            if (writeQueue_.empty())
            {
                writing_ = false;
                return;
            }
            writing_ = true;
            ws_.async_write(net::buffer(writeQueue_.front()),
                            [self = shared_from_this()](beast::error_code ec, std::size_t)
                            {
                                if (ec)
                                {
                                    self->server_->removeSession(self);
                                    return;
                                }
                                std::lock_guard<std::mutex> lock(self->writeMutex_);
                                self->writeQueue_.pop_front();
                                self->doWrite();
                            });
        }

        websocket::stream<tcp::socket> ws_;
        WebSocketServer *server_;
        beast::flat_buffer buffer_;
        std::mutex writeMutex_;
        std::deque<std::string> writeQueue_;
        bool writing_ = false;
    };

    WebSocketServer::WebSocketServer(int port)
        : port_(port), running_(false), acceptor_(ioc_)
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

        serverThread_ = std::thread([this]()
                                     {
            try {
                acceptor_.open(tcp::v4());
                acceptor_.set_option(net::socket_base::reuse_address(true));
                acceptor_.bind(tcp::endpoint(tcp::v4(), port_));
                acceptor_.listen();

                broadcastTimer_ = std::make_shared<net::steady_timer>(
                    ioc_, std::chrono::milliseconds(100));

                doAccept();
                broadcastTimer_->async_wait(
                    std::bind(&WebSocketServer::onBroadcastTimer, this,
                              std::placeholders::_1));

                ioc_.run();
            } catch (std::exception& e) {
                Logger::getInstance().error(
                    "WebSocket server error: " + std::string(e.what()));
            } });

        Logger::getInstance().info("WebSocket server started on port " + std::to_string(port_));
    }

    void WebSocketServer::stop()
    {
        if (!running_)
            return;

        running_ = false;

        {
            std::lock_guard<std::mutex> lock(sessionsMutex_);
            for (auto &session : sessions_)
            {
                session->close();
            }
            sessions_.clear();
        }

        ioc_.stop();
        if (serverThread_.joinable())
        {
            serverThread_.join();
        }

        Logger::getInstance().info("WebSocket server stopped");
    }

    void WebSocketServer::publishState(const FusedEntityState &state)
    {
        std::string message = serializeState(state);

        std::lock_guard<std::mutex> lock(queueMutex_);
        messageQueue_.push_back(std::move(message));

        if (messageQueue_.size() > 100)
        {
            messageQueue_.pop_front();
        }
    }

    void WebSocketServer::publishStates(const std::vector<FusedEntityState> &states)
    {
        std::string message = serializeStates(states);

        std::lock_guard<std::mutex> lock(queueMutex_);
        messageQueue_.push_back(std::move(message));

        if (messageQueue_.size() > 100)
        {
            messageQueue_.pop_front();
        }
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

    void WebSocketServer::doAccept()
    {
        acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket)
                               {
            if (!ec) {
                auto session = std::make_shared<Session>(std::move(socket), this);
                {
                    std::lock_guard<std::mutex> lock(sessionsMutex_);
                    sessions_.insert(session);
                }
                session->run();
                Logger::getInstance().info(
                    "WebSocket client connected. Total: " +
                    std::to_string(sessions_.size()));
            }

            if (running_) {
                doAccept();
            } });
    }

    void WebSocketServer::onBroadcastTimer(boost::system::error_code ec)
    {
        if (ec || !running_)
            return;

        std::vector<std::string> messages;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            messages.assign(messageQueue_.begin(), messageQueue_.end());
            messageQueue_.clear();
        }

        for (const auto &msg : messages)
        {
            broadcastToSessions(msg);
        }

        broadcastTimer_->expires_after(std::chrono::milliseconds(100));
        broadcastTimer_->async_wait(
            std::bind(&WebSocketServer::onBroadcastTimer, this,
                      std::placeholders::_1));
    }

    void WebSocketServer::broadcastToSessions(const std::string &message)
    {
        std::lock_guard<std::mutex> lock(sessionsMutex_);
        for (auto &session : sessions_)
        {
            session->send(message);
        }
    }

    void WebSocketServer::removeSession(std::shared_ptr<Session> session)
    {
        std::lock_guard<std::mutex> lock(sessionsMutex_);
        sessions_.erase(session);
        Logger::getInstance().info(
            "WebSocket client disconnected. Remaining: " +
            std::to_string(sessions_.size()));
    }

} // namespace SensorFusion
