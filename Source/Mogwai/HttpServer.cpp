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
            std::unique_ptr<char[]> buffer = std::make_unique<char[]>(
                3 * sizeof(int) + sceneData.meshDesc.size() * sizeof(MeshDesc) + sceneData.meshIndexData.size() * sizeof(uint32_t) +
                sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData)
            );
            int size = sceneData.meshDesc.size();
            int offset = 0;
            memcpy(buffer.get() + offset, &size, sizeof(int));
            offset += sizeof(int);
            size = sceneData.meshIndexData.size();
            memcpy(buffer.get() + offset, &size, sizeof(int));
            offset += sizeof(int);
            size = sceneData.meshStaticData.size();
            memcpy(buffer.get() + offset, &size, sizeof(int));
            offset += sizeof(int);
            memcpy(buffer.get() + offset, sceneData.meshDesc.data(), sceneData.meshDesc.size() * sizeof(MeshDesc));
            offset += sceneData.meshDesc.size() * sizeof(MeshDesc);
            memcpy(buffer.get() + offset, sceneData.meshIndexData.data(), sceneData.meshIndexData.size() * sizeof(uint32_t));
            offset += sceneData.meshIndexData.size() * sizeof(uint32_t);
            memcpy(
                buffer.get() + offset, sceneData.meshStaticData.data(), sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData)
            );
            res.set_content(
                buffer.get(), offset + sceneData.meshStaticData.size() * sizeof(PackedStaticVertexData), "application/octet-stream"
            );
            Falcor::logInfo(
                "meshDesc.size(): {}, meshIndexData.size(): {}, meshStaticData.size(): {}",
                sceneData.meshDesc.size(),
                sceneData.meshIndexData.size(),
                sceneData.meshStaticData.size()
            );
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
