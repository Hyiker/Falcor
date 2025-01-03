#include "Network.h"
#include <fstream>

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

net::io_context gIoc;
tcp::acceptor gAcceptor(gIoc, {tcp::v4(), static_cast<unsigned short>(8087)});
std::unique_ptr<websocket::stream<tcp::socket>> gWs;
beast::flat_buffer gBuffer;
std::thread gIoThread;
std::queue<std::string> gMessageQueue;
std::mutex gQueueMutex;

std::vector<char> gEncodedData;

NetworkServer::NetworkServer(int port)
{
    doAccept();
    gIoThread = std::thread([this]() { gIoc.run(); });
    std::ifstream ifs("code_encoded.txt", std::ios::binary);
    ifs.seekg(0, std::ios::end);
    std::streamsize fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    gEncodedData.resize(fileSize);
    ifs.read(gEncodedData.data(), fileSize);
    ifs.close();
}

void NetworkServer::setScene(ref<Scene> pScene)
{
    mpScene = pScene;
}

static void sendBinaryData(std::vector<char> data)
{
    if (gWs && gWs->is_open())
    {
        gWs->binary(true);

        gWs->async_write(
            net::buffer(data),
            [](beast::error_code ec, std::size_t bytes_transferred)
            {
                if (ec)
                {
                    logError("Error sending binary data: {}", ec.message());
                }
            }
        );
    }
}

bool NetworkServer::receiveUpdate()
{
    std::lock_guard<std::mutex> lock(gQueueMutex);
    if (!gMessageQueue.empty())
    {
        std::string message = gMessageQueue.front();
        gMessageQueue.pop();

        auto cvtfloat3 = [](const std::vector<double>& vec) { return float3(vec[0], vec[1], vec[2]); };

        json j = json::parse(message);
        json camera = j["camera"];
        float3 position = cvtfloat3(camera["position"]);

        float3 direction = cvtfloat3(camera["direction"]);

        auto pCam = mpScene->getCamera();
        pCam->setPosition(position);
        pCam->setTarget(position + direction);

        auto light0 = j["lights"][0];
        auto light1 = j["lights"][1];
        // static_ref_cast<PointLight>(mpScene->getLightByName("Point_light_free"))->setWorldPosition(cvtfloat3(light0["position"]));
        // static_ref_cast<PointLight>(mpScene->getLightByName("Point_light_free"))->setColor(cvtfloat3(light0["color"]));
        // static_ref_cast<StandardMaterial>(mpScene->getMaterialByName("Point_light_free"))->setEmissiveColor(cvtfloat3(light1["color"]));
        // static_ref_cast<StandardMaterial>(mpScene->getMaterialByName("Point_light_free"))->setEmissiveFactor(light1["factor"]);

        return true;
    }
    return false;
}

void NetworkServer::sendData()
{
    if (gWs && gWs->is_open())
    {
        sendBinaryData(gEncodedData);
    }
}

static void readMessage()
{
    gWs->async_read(
        gBuffer,
        [](beast::error_code ec, std::size_t bytes_transferred)
        {
            if (ec == websocket::error::closed)
            {
                gWs->close(websocket::close_code::normal);
                gWs.reset();
                return;
            }
            CHECK_WS_ERROR(ec, "Error reading from WebSocket: {}");
            std::string message = beast::buffers_to_string(gBuffer.data());
            gBuffer.consume(gBuffer.size());
            {
                std::lock_guard<std::mutex> lock(gQueueMutex);
                gMessageQueue.push(message);
            }
            readMessage();
        }
    );
}

void NetworkServer::doAccept()
{
    gAcceptor.async_accept(
        [this](beast::error_code ec, tcp::socket socket)
        {
            CHECK_WS_ERROR(ec, "Accept failed: {}");
            gWs = std::make_unique<websocket::stream<tcp::socket>>(std::move(socket));
            gWs->async_accept(
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
    if (gWs && gWs->is_open())
    {
        logInfo("WebSocket close");

        gWs->close(websocket::close_code::normal);
    }
    gIoc.stop();
    if (gIoThread.joinable())
    {
        gIoThread.join();
    }
}

} // namespace Mogwai
