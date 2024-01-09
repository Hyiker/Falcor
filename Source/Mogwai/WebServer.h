#pragma once
#include <boost/asio/ip/tcp.hpp>
#include <cinttypes>
#include <memory>
#include <thread>

namespace Mogwai
{
class Renderer;
class WebServer
{
public:
    WebServer(const Renderer* renderer, uint16_t port = 8080U);
    void sendCameraUpdate() const;
    ~WebServer();
    void run();

private:
    const Renderer* mpRenderer;
    uint16_t mPort;
    std::unique_ptr<std::thread> mpWSThread;
    std::unique_ptr<boost::asio::io_context> mpIoContext;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> mpAcceptor;
};
} // namespace Mogwai
