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

std::vector<char> createMaterialBuffer(const Renderer* renderer, int offset)
{
    auto scene = renderer->mpScene;
    if (!scene)
    {
        Falcor::logInfo("renderer->mpScene is null");
        return {};
    }
    /* scene material */
    std::vector<char> materialBuffer(4); // materialCount: 4 bytes
    int materialTotalSize = scene->getMaterials().size();
    constexpr int materialPageCount = 10;
    int materialPageSize = std::ceil(float(materialTotalSize) / materialPageCount);
    int startIndex = offset * materialPageSize;
    int materialSize = std::min(std::max(0, materialTotalSize - startIndex), materialPageSize);
    memcpy(materialBuffer.data(), &materialSize, sizeof(int));
    int materialBufferOffset = 4;
    Falcor::logInfo(
        "material page: {}, materialPageSize: {}, materialTotalSize: {}, materialSize: {}",
        offset,
        materialPageSize,
        materialTotalSize,
        materialSize
    );
    if (offset < 0)
    {
        return std::move(materialBuffer);
    }
    for (int j = 0; j < materialSize; j++)
    {
        auto material = scene->getMaterials()[j + startIndex];
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
                uint32_t compressedSize = texture->rawDataCompressed.size();
                textureSize += compressedSize;
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

WebServer::WebServer(const Renderer* renderer, uint16_t port) : mpRenderer(renderer), mPort(port) {}
std::unique_ptr<websocket::stream<tcp::socket>> mpWs;
std::mutex websocketMutex;
std::atomic_uint32_t nFrame = 0;
void WebServer::sendCameraUpdate() const
{
    if (mpWs)
    {
        std::lock_guard<std::mutex> lock(websocketMutex);
        std::vector<char> dataBuffer;
        dataBuffer = createCameraBuffer(mpRenderer);
        std::vector<char> responseBuffer(sizeof(DataPackHeader));
        DataPack pack{};
        pack.header.eventId = nFrame++;
        pack.header.eventType = EventType::CameraUpdate;
        pack.header.dataSizeInBytes = dataBuffer.size();
        memcpy(responseBuffer.data(), &pack.header, sizeof(DataPackHeader));
        responseBuffer.insert(
            responseBuffer.begin() + sizeof(DataPackHeader),
            std::make_move_iterator(dataBuffer.begin()),
            std::make_move_iterator(dataBuffer.end())
        );
        mpWs->write(net::buffer(responseBuffer.data(), responseBuffer.size()));
    }
}

WebServer::~WebServer()
{
    mpWSThread->detach();
    Falcor::logInfo("Stopping Web server");
}
void WebServer::run()
{
    const std::string host = "0.0.0.0";
    Falcor::logInfo("Starting HTTP server on {}:{}", host, mPort);

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
                            mpWs = std::make_unique<websocket::stream<tcp::socket>>(std::move(socket));
                            mpWs->binary(true);
                            // Set a decorator to change the Server of the handshake
                            mpWs->set_option(websocket::stream_base::decorator(
                                [](websocket::response_type& res)
                                { res.set(http::field::server, std::string(BOOST_BEAST_VERSION_STRING) + " websocket-server-sync"); }
                            ));
                            // enable compression
                            websocket::permessage_deflate pmd;
                            pmd.server_enable = true;
                            mpWs->set_option(pmd);

                            // Accept the websocket handshake
                            mpWs->accept();

                            for (;;)
                            {
                                logInfo("WS server waiting for message");
                                // This buffer will hold the incoming message
                                beast::flat_buffer buffer;
                                mpWs->binary(true);

                                // Read a message
                                mpWs->read(buffer);
                                std::lock_guard<std::mutex> lock(websocketMutex);
                                DataPack pack{};
                                if (buffer.size() < sizeof(DataPackHeader))
                                {
                                    Falcor::logError("WS message size is less than DataPackHeader size: {}", buffer.size());
                                    continue;
                                }
                                memcpy(&pack.header, buffer.data().data(), sizeof(DataPackHeader));
                                const char* pData = nullptr;
                                if (pack.header.dataSizeInBytes > 0)
                                {
                                    pData = (const char*)buffer.data().data() + sizeof(DataPackHeader);
                                }

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
                                    int offset = 0;
                                    memcpy(&offset, pData, sizeof(int));
                                    Falcor::logInfo("FetchMaterial offset: {}", offset);
                                    dataBuffer = createMaterialBuffer(mpRenderer, offset);
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
                                mpWs->write(net::buffer(responseBuffer.data(), responseBuffer.size()));
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
