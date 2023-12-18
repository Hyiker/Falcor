#include "HttpServer.h"
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
HttpServer::HttpServer(const Renderer* renderer, uint16_t port) : mpRenderer(renderer), mPort(port)
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
            memcpy(
                buffer.data() + offset, sceneData.meshStaticData.data(), sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData)
            );
            for(const auto& meshDesc : sceneData.meshDesc)
            {
                Falcor::logInfo("meshDesc.indexCount: {}, meshDesc.indexOffset: {}, meshDesc.vertexCount: {}, meshDesc.vertexOffset: {}",
                    meshDesc.indexCount, meshDesc.ibOffset, meshDesc.vertexCount, meshDesc.vbOffset);
            }
            Falcor::logInfo(
                "meshDesc.size(): {}, meshIndexData.size(): {}, meshStaticData.size(): {}",
                sceneData.meshDesc.size(),
                sceneData.meshIndexData.size(),
                sceneData.meshStaticData.size()
            );
            /* scene material */
            std::vector<char> materialBuffer(4); // materialCount: 4 bytes
            int materialSize = scene->getMaterials().size();
            memcpy(materialBuffer.data(), &materialSize, sizeof(int));
            int materialBufferOffset = 4;
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
                        if(!texture)
                        {
                            continue;
                        }
                        Falcor::logInfo("Slot {} has texture", Falcor::to_string(static_cast<Material::TextureSlot>(i)));
                        ++nTextures;
                        int textureSize = 4 + 4 + 4 + 4; // slot + format + width + height
                        int textureRawDataSizeInBytes =
                            texture->getWidth() * texture->getHeight() * getFormatBytesPerBlock(texture->getFormat());
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
                        memcpy(materialBuffer.data() + materialBufferOffset, texture->rawData, textureRawDataSizeInBytes);
                        materialBufferOffset += textureRawDataSizeInBytes;
                        Falcor::logInfo("Texture size: {} bytes", textureRawDataSizeInBytes);
                    }
                }
                Falcor::logInfo("Material has {} textures", nTextures);
                memcpy(materialBuffer.data() + headerOffset, &nTextures, sizeof(int));
            }

            Falcor::logInfo("materialSize: {}", materialSize);
            Falcor::logInfo("materialBuffer.size(): {}", materialBuffer.size());
            Falcor::logInfo("materialBufferOffset: {}", materialBufferOffset);
            buffer.insert(buffer.end(), std::make_move_iterator(materialBuffer.begin()), std::make_move_iterator(materialBuffer.end()));

            res.set_content(buffer.data(), buffer.size(), "application/octet-stream");
            res.set_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type, Accept");
            res.set_header("Access-Control-Allow-Origin", "*");
            Falcor::logInfo("HTTP GET /scene size: {} bytes", offset + sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData));
            Falcor::logInfo("HTTP GET /scene end");
        }
    );
    mServer.Get(
        "/camera",
        [renderer](const httplib::Request& req, httplib::Response& res)
        {
            Falcor::logInfo("HTTP GET /camera start");
            if (renderer == nullptr)
            {
                Falcor::logInfo("HTTP GET /camera renderer is null");
                res.status = 500;
                return;
            }
            const auto& scene = renderer->getScene();
            if (!scene)
            {
                Falcor::logInfo("HTTP GET /camera scene is null");
                res.status = 500;
                return;
            }
            std::unique_ptr<char[]> buffer = std::make_unique<char[]>(sizeof(CameraData));
            auto camera = scene->getCamera();
            if (!camera)
            {
                Falcor::logInfo("HTTP GET /camera camera is null");
                res.status = 500;
                return;
            }
            memcpy(buffer.get(), &camera->getData(), sizeof(CameraData));
            res.set_content(buffer.get(), sizeof(CameraData), "application/octet-stream");
            res.set_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type, Accept");
            res.set_header("Access-Control-Allow-Origin", "*");
            Falcor::logInfo("HTTP GET /camera size: {} bytes", sizeof(CameraData));
            Falcor::logInfo("HTTP GET /camera end");
        }
    );
}

HttpServer::~HttpServer()
{
    mServer.stop();
    mpThread->join();
    Falcor::logInfo("Stopping HTTP server");
}
void HttpServer::run()
{
    const std::string host = "0.0.0.0";
    Falcor::logInfo("Starting HTTP server on {}:{}", host, mPort);
    mpThread = std::make_unique<std::thread>([this, &host]() { mServer.listen(host, mPort); });
    mServer.wait_until_ready();
}
} // namespace Mogwai
