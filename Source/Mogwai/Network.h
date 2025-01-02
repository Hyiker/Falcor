#pragma once
#include <atomic>
#include <string>
#include <thread>

#include "Falcor.h"
#include "Core/Enum.h"
#include "Utils/Timing/Clock.h"

namespace Mogwai
{

using namespace Falcor;
class NetworkServer
{
public:
    NetworkServer(int port);

    void setScene(ref<Scene> pScene);

    bool receiveUpdate();

    ~NetworkServer();

private:
    void doAccept();

    ref<Scene> mpScene;
};
} // namespace Mogwai
