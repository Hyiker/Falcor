#pragma once
#include <atomic>
#include <string>
#include <thread>

#include <libwebsockets.h>
#include "Core/Enum.h"
#include "Utils/Timing/Clock.h"

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
    using Clock = Falcor::Clock;

    NetworkServer(const std::string& host, int port, Clock& clock);

    /** Dispatch the request
     */
    std::string dispatch(const std::string& clientMsg) const;

    ~NetworkServer();

private:
    double getSceneAnimationTime() const;

    Falcor::Clock& mClock; ///< Global clock reference for animation sync.

    lws_context* mpWebSocketContext = nullptr;
    std::atomic_bool mServerStop = false;

    std::unique_ptr<std::thread> mpServerThread;
};
} // namespace Mogwai
