#pragma once
#include <httplib.h>
#include <cinttypes>
#include <memory>
#include <thread>

namespace Mogwai
{
class Renderer;
class HttpServer
{
public:
    HttpServer(const Renderer* renderer, uint16_t port = 8080U);
    ~HttpServer();
    void run();

private:
    const Renderer* mpRenderer;
    uint16_t mPort;
    httplib::Server mServer;

    std::unique_ptr<std::thread> mpThread;
};
} // namespace Mogwai
