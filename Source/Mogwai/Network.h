#pragma once
#include <atomic>
#include <string>
#include <thread>

#include <libwebsockets.h>
#include "Core/Enum.h"

namespace Mogwai
{
enum class NetworkRequestType : uint8_t
{
    FrameSync = 0,
    AssetsSync = 1,
};
FALCOR_ENUM_INFO(NetworkRequestType, {{NetworkRequestType::FrameSync, "FrameSync"}, {NetworkRequestType::AssetsSync, "AssetsSync"}});
class NetworkServer
{
public:
    NetworkServer(const std::string& host, int port);

    /** Dispatch the request
     */
    std::string dispatch(const char* requestPayload) const;

    ~NetworkServer();

private:
    lws_context* mpWebSocketContext = nullptr;
    std::atomic_bool mServerStop = false;
    std::unique_ptr<std::thread> mpServerThread;
};
} // namespace Mogwai
