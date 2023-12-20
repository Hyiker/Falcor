#pragma once
#include <httplib.h>
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
    ~WebServer();
    void run();

private:
    const Renderer* mpRenderer;
    uint16_t mPort;
    httplib::Server mServer;
    std::unique_ptr<std::thread> mpHttpThread, mpWSThread;
    std::unique_ptr<boost::asio::io_context> mpIoContext;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> mpAcceptor;
};
} // namespace Mogwai
