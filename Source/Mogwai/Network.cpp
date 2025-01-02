#include "Network.h"
#define _WIN32_WINDOWS 0x0601
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

#include "Falcor.h"

#define CHECK_WS_ERROR(ec, fmt)          \
    do                                   \
    {                                    \
        if (ec)                          \
        {                                \
            logError(fmt, ec.message()); \
            return;                      \
        }                                \
    } while (false)

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using json = nlohmann::json;

namespace Mogwai
{

net::io_context ioc_;
tcp::acceptor acceptor_(ioc_, {tcp::v4(), static_cast<unsigned short>(8087)});
std::unique_ptr<websocket::stream<tcp::socket>> ws_;
beast::flat_buffer buffer_;
std::thread io_thread_;
std::queue<std::string> message_queue_;
std::mutex queue_mutex_;

NetworkServer::NetworkServer(int port)
{
    doAccept();
    io_thread_ = std::thread([this]() { ioc_.run(); });
}

void NetworkServer::setScene(ref<Scene> pScene)
{
    mpScene = pScene;
}

bool NetworkServer::receiveUpdate()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!message_queue_.empty())
    {
        std::string message = message_queue_.front();
        message_queue_.pop();

        json j = json::parse(message);
        json camera = j["camera"];
        std::vector<double> posArr = camera["position"];
        float3 position(posArr[0], posArr[1], posArr[2]);

        std::vector<double> dirArr = camera["direction"];
        float3 direction(dirArr[0], dirArr[1], dirArr[2]);

        auto pCam = mpScene->getCamera();
        pCam->setPosition(position);
        pCam->setTarget(position + direction);

        return true;
    }
    return false;
}

static void readMessage()
{
    ws_->async_read(
        buffer_,
        [](beast::error_code ec, std::size_t bytes_transferred)
        {
            if (ec == websocket::error::closed)
            {
                ws_->close(websocket::close_code::normal);
                ws_.reset();
                return;
            }
            CHECK_WS_ERROR(ec, "Error reading from WebSocket: {}");
            std::string message = beast::buffers_to_string(buffer_.data());
            buffer_.consume(buffer_.size());
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                message_queue_.push(message);
            }
            readMessage();
        }
    );
}

void NetworkServer::doAccept()
{
    acceptor_.async_accept(
        [this](beast::error_code ec, tcp::socket socket)
        {
            CHECK_WS_ERROR(ec, "Accept failed: {}");
            ws_ = std::make_unique<websocket::stream<tcp::socket>>(std::move(socket));
            ws_->async_accept(
                [this](beast::error_code ec)
                {
                    CHECK_WS_ERROR(ec, "Error ws accept: {}");

                    readMessage();
                }
            );
        }
    );
}
NetworkServer::~NetworkServer()
{
    if (ws_ && ws_->is_open())
    {
        logInfo("WebSocket close");

        ws_->close(websocket::close_code::normal);
    }
    ioc_.stop();
    if (io_thread_.joinable())
    {
        io_thread_.join();
    }
}

} // namespace Mogwai
