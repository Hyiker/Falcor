#include "WebServer.h"
#include <boost/asio/detached.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include "Mogwai.h"
#include "Scene/Scene.h"
#include "Utils/Logger.h"
namespace Mogwai
{
struct TrivialSceneData
{
    std::vector<MeshDesc> meshDesc;
    std::vector<uint32_t> meshIndexData;
    std::vector<PackedStaticVertexData> meshStaticData;
    TrivialSceneData(const Scene::SceneData& rawSceneData)
    {
        meshDesc = rawSceneData.meshDesc;
        meshIndexData = rawSceneData.meshIndexData;
        meshStaticData = rawSceneData.meshStaticData;
    }
};
using WSEventId = uint32_t;
enum class EventType : int
{
    FetchMesh,
    FetchMaterial,
    CameraUpdate,
    AnimationUpdate,

    Count
};

const std::string_view kEventTypeNames[] = {
    "FetchMesh",
    "FetchMaterial",
    "CameraUpdate",
    "AnimationUpdate",
};

struct DataPackHeader
{
    WSEventId eventId;
    EventType eventType;
    // size of the data in bytes, excluding the header
    uint32_t dataSizeInBytes;
};

struct DataPack
{
    DataPackHeader header;
    char* data;
};

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
std::vector<char> createCameraBuffer(const Renderer* renderer)
{
    if (renderer == nullptr)
    {
        Falcor::logInfo("HTTP GET /camera renderer is null");
        return {};
    }
    const auto& scene = renderer->getScene();
    if (!scene)
    {
        Falcor::logInfo("HTTP GET /camera scene is null");
        return {};
    }
    std::vector<char> buffer(sizeof(CameraData));
    auto camera = scene->getCamera();
    if (!camera)
    {
        Falcor::logInfo("HTTP GET /camera camera is null");
        return {};
    }
    memcpy(buffer.data(), &camera->getData(), sizeof(CameraData));
    return std::move(buffer);
}
std::vector<char> createMeshBuffer(const Renderer* renderer)
{
    auto scene = renderer->mpScene;
    if (!scene)
    {
        Falcor::logInfo("renderer->mpScene is null");
        return {};
    }
    TrivialSceneData sceneData(scene->rawSceneData);
    int meshSizeInBytes = 3 * sizeof(int) + sceneData.meshDesc.size() * sizeof(MeshDesc) +
                          sceneData.meshIndexData.size() * sizeof(uint32_t) +
                          sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData);
    std::vector<char> buffer(meshSizeInBytes);
    /* scene mesh */
    int size = sceneData.meshDesc.size();
    int offset = 0;
    memcpy(buffer.data() + offset, &size, sizeof(int));
    offset += sizeof(int);
    size = sceneData.meshIndexData.size();
    memcpy(buffer.data() + offset, &size, sizeof(int));
    offset += sizeof(int);
    size = sceneData.meshStaticData.size();
    memcpy(buffer.data() + offset, &size, sizeof(int));
    offset += sizeof(int);
    memcpy(buffer.data() + offset, sceneData.meshDesc.data(), sceneData.meshDesc.size() * sizeof(MeshDesc));
    offset += sceneData.meshDesc.size() * sizeof(MeshDesc);
    memcpy(buffer.data() + offset, sceneData.meshIndexData.data(), sceneData.meshIndexData.size() * sizeof(uint32_t));
    offset += sceneData.meshIndexData.size() * sizeof(uint32_t);
    memcpy(buffer.data() + offset, sceneData.meshStaticData.data(), sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData));
    for (const auto& meshDesc : sceneData.meshDesc)
    {
        Falcor::logInfo(
            "meshDesc.indexCount: {}, meshDesc.indexOffset: {}, meshDesc.vertexCount: {}, meshDesc.vertexOffset: {}",
            meshDesc.indexCount,
            meshDesc.ibOffset,
            meshDesc.vertexCount,
            meshDesc.vbOffset
        );
    }
    Falcor::logInfo(
        "meshDesc.size(): {}, meshIndexData.size(): {}, meshStaticData.size(): {}",
        sceneData.meshDesc.size(),
        sceneData.meshIndexData.size(),
        sceneData.meshStaticData.size()
    );
    return std::move(buffer);
}

std::vector<char> createMaterialBuffer(const Renderer* renderer)
{
    auto scene = renderer->mpScene;
    if (!scene)
    {
        Falcor::logInfo("renderer->mpScene is null");
        return {};
    }
    /* scene material */
    std::vector<char> materialBuffer(4); // materialCount: 4 bytes
    int materialSize = scene->getMaterials().size();
    memcpy(materialBuffer.data(), &materialSize, sizeof(int));
    int materialBufferOffset = 4;
    std::unordered_map<Falcor::Texture*, uint32_t> textureMap;
    for (const auto& material : scene->getMaterials())
    {
        int nTextures = 0;
        int headerOffset = materialBufferOffset;
        materialBufferOffset += 4; // nTextures: 4 bytes
        materialBuffer.resize(materialBufferOffset);
        for (int i = 0; i < static_cast<int>(Material::TextureSlot::Count); ++i)
        {
            if (material->hasTextureSlot(static_cast<Material::TextureSlot>(i)))
            {
                const auto texture = material->getTexture(static_cast<Material::TextureSlot>(i));
                auto slot = static_cast<Material::TextureSlot>(i);
                if (!texture || (slot != Material::TextureSlot::BaseColor && slot != Material::TextureSlot::Normal))
                {
                    continue;
                }
                Falcor::logInfo("Slot {} has texture", Falcor::to_string(static_cast<Material::TextureSlot>(i)));
                ++nTextures;
                int textureSize = 4 + 4 + 4 + 4 + 4; // slot + format + width + height + compressed size
                int textureRawDataSizeInBytes = texture->getWidth() * texture->getHeight() * getFormatBytesPerBlock(texture->getFormat());
                textureSize += textureRawDataSizeInBytes;
                materialBuffer.resize(materialBufferOffset + textureSize);
                memcpy(materialBuffer.data() + materialBufferOffset, &i, sizeof(int));
                materialBufferOffset += 4;
                int format = static_cast<int>(texture->getFormat());
                Falcor::logInfo("Texture format: {}", format);
                memcpy(materialBuffer.data() + materialBufferOffset, &format, sizeof(int));
                materialBufferOffset += 4;
                int width = texture->getWidth(), height = texture->getHeight();
                memcpy(materialBuffer.data() + materialBufferOffset, &width, sizeof(int));
                materialBufferOffset += 4;
                memcpy(materialBuffer.data() + materialBufferOffset, &height, sizeof(int));
                materialBufferOffset += 4;
                uint32_t compressedSize = texture->rawDataCompressed.size();
                memcpy(materialBuffer.data() + materialBufferOffset, &compressedSize, sizeof(uint32_t));
                materialBufferOffset += 4;
                memcpy(materialBuffer.data() + materialBufferOffset, texture->rawDataCompressed.data(), compressedSize);
                materialBufferOffset += compressedSize;
                Falcor::logInfo("Texture size(compressed): {} bytes", compressedSize);
            }
        }
        Falcor::logInfo("Material has {} textures", nTextures);
        memcpy(materialBuffer.data() + headerOffset, &nTextures, sizeof(int));
    }

    Falcor::logInfo("materialSize: {}", materialSize);
    Falcor::logInfo("materialBuffer.size(): {}", materialBuffer.size());
    Falcor::logInfo("materialBufferOffset: {}", materialBufferOffset);
    return std::move(materialBuffer);
}

std::vector<char> createMeshAndMaterialBuffer(const Renderer* renderer)
{
    auto buffer = createMeshBuffer(renderer);
    auto materialBuffer = createMaterialBuffer(renderer);
    buffer.insert(buffer.end(), std::make_move_iterator(materialBuffer.begin()), std::make_move_iterator(materialBuffer.end()));

    return std::move(buffer);
}
WebServer::WebServer(const Renderer* renderer, uint16_t port) : mpRenderer(renderer), mPort(port)
{
    mServer.Get(
        "/scene",
        [renderer](const httplib::Request& req, httplib::Response& res)
        {
            Falcor::logInfo("HTTP GET /scene start");
            if (renderer == nullptr)
            {
                Falcor::logInfo("HTTP GET /scene renderer is null");
                res.status = 500;
                return;
            }
            const auto& scene = renderer->getScene();
            if (!scene)
            {
                Falcor::logInfo("HTTP GET /scene scene is null");
                res.status = 500;
                return;
            }
            auto buffer = createMeshAndMaterialBuffer(renderer);
            res.set_content(buffer.data(), buffer.size(), "application/octet-stream");
            res.set_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type, Accept");
            res.set_header("Access-Control-Allow-Origin", "*");
            Falcor::logInfo("HTTP GET /scene size: {} bytes", buffer.size());
            Falcor::logInfo("HTTP GET /scene end");
        }
    );
    mServer.Get(
        "/camera",
        [renderer](const httplib::Request& req, httplib::Response& res)
        {
            Falcor::logInfo("HTTP GET /camera start");

            auto buffer = createCameraBuffer(renderer);
            if (buffer.empty())
            {
                res.status = 500;
                return;
            }
            res.set_content(buffer.data(), buffer.size(), "application/octet-stream");
            res.set_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type, Accept");
            res.set_header("Access-Control-Allow-Origin", "*");
            Falcor::logInfo("HTTP GET /camera size: {} bytes", buffer.size());
            Falcor::logInfo("HTTP GET /camera end");
        }
    );
}

WebServer::~WebServer()
{
    mServer.stop();
    mpWSThread->detach();
    mpHttpThread->join();
    Falcor::logInfo("Stopping HTTP server");
}
void WebServer::run()
{
    const std::string host = "0.0.0.0";
    Falcor::logInfo("Starting HTTP server on {}:{}", host, mPort);
    mpHttpThread = std::make_unique<std::thread>([this, &host]() { mServer.listen(host, mPort); });
    mServer.wait_until_ready();

    auto const wsAddress = net::ip::make_address(host);
    auto const wsPort = static_cast<unsigned short>(mPort + 1);

    mpIoContext = std::make_unique<net::io_context>(1);

    // The acceptor receives incoming connections
    mpAcceptor = std::unique_ptr<tcp::acceptor>(new tcp::acceptor{*mpIoContext, {wsAddress, wsPort}});

    // Open the acceptor
    mpWSThread = std::make_unique<std::thread>(
        [this, host]()
        {
            Falcor::logInfo("Starting WS server on {}:{}", host, mPort + 1);
            for (;;)
            {
                // This will receive the new connection
                tcp::socket socket{*mpIoContext};

                // Block until we get a connection
                mpAcceptor->accept(socket);

                // Launch the session, transferring ownership of the socket
                std::thread(
                    [this](tcp::socket socket)
                    {
                        try
                        {
                            // Construct the stream by moving in the socket
                            websocket::stream<tcp::socket> ws{std::move(socket)};
                            ws.binary(true);
                            // Set a decorator to change the Server of the handshake
                            ws.set_option(websocket::stream_base::decorator(
                                [](websocket::response_type& res)
                                { res.set(http::field::server, std::string(BOOST_BEAST_VERSION_STRING) + " websocket-server-sync"); }
                            ));
                            // enable compression
                            websocket::permessage_deflate pmd;
                            pmd.server_enable = true;
                            ws.set_option(pmd);

                            // Accept the websocket handshake
                            ws.accept();

                            for (;;)
                            {
                                // This buffer will hold the incoming message
                                beast::flat_buffer buffer;
                                ws.binary(true);

                                // Read a message
                                ws.read(buffer);
                                DataPack pack{};
                                if (buffer.size() < sizeof(DataPackHeader))
                                {
                                    Falcor::logError("WS message size is less than DataPackHeader size: {}", buffer.size());
                                    continue;
                                }
                                memcpy(&pack.header, buffer.data().data(), sizeof(DataPackHeader));
                                // if (pack.header.dataSizeInBytes > 0)
                                // {
                                //     pack.data = new char[pack.header.dataSizeInBytes];
                                //     memcpy(pack.data, buffer.data().data() + sizeof(DataPackHeader), pack.header.dataSizeInBytes);
                                // }

                                Falcor::logInfo(
                                    "WS message eventId: {}, eventType: {}, dataSizeInBytes: {}",
                                    pack.header.eventId,
                                    kEventTypeNames[static_cast<int>(pack.header.eventType)],
                                    pack.header.dataSizeInBytes
                                );
                                std::vector<char> dataBuffer;
                                switch (pack.header.eventType)
                                {
                                case EventType::FetchMesh:
                                {
                                    dataBuffer = createMeshBuffer(mpRenderer);
                                    break;
                                }
                                case EventType::FetchMaterial:
                                {
                                    dataBuffer = createMaterialBuffer(mpRenderer);
                                    break;
                                }
                                case EventType::CameraUpdate:
                                {
                                    dataBuffer = createCameraBuffer(mpRenderer);
                                    break;
                                }
                                default:
                                    Falcor::logError("Unknown event type: {}", kEventTypeNames[(int)pack.header.eventType]);
                                    break;
                                };
                                Falcor::logInfo("WS message size: {} bytes", buffer.size());
                                if (dataBuffer.empty())
                                {
                                    Falcor::logError("WS message dataBuffer is empty");
                                    continue;
                                }
                                std::vector<char> responseBuffer(sizeof(DataPackHeader));
                                pack.header.dataSizeInBytes = dataBuffer.size();
                                Falcor::logInfo("WS response dataSizeInBytes: {} bytes", pack.header.dataSizeInBytes);
                                memcpy(responseBuffer.data(), &pack.header, sizeof(DataPackHeader));
                                responseBuffer.insert(
                                    responseBuffer.begin() + sizeof(DataPackHeader),
                                    std::make_move_iterator(dataBuffer.begin()),
                                    std::make_move_iterator(dataBuffer.end())
                                );
                                Falcor::logInfo("WS response size: {} bytes", responseBuffer.size());
                                ws.write(net::buffer(responseBuffer.data(), responseBuffer.size()));
                            }
                        }
                        catch (beast::system_error const& se)
                        {
                            // This indicates that the session was closed
                            if (se.code() != websocket::error::closed)
                                logError("system_error: {}", se.what());
                        }
                        catch (std::exception const& e)
                        {
                            logError("exception: {}", e.what());
                        }
                    },
                    std::move(socket)
                )
                    .detach();
            };
        }
    );
}
} // namespace Mogwai
