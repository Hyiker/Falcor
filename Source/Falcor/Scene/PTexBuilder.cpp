#include "PTexBuilder.h"
#include "Utils/Image/ImageIO.h"
#include "Utils/Math/Common.h"
#include "Scene/Material/BasicMaterial.h"

namespace Falcor
{

struct : public PtexErrorHandler
{
    void reportError(const char* error) override { logError("{}", error); }
} errorHandler;

PTexBuilder::PTexBuilder(ref<Device> pDevice) : mpDevice(pDevice)
{
    int maxFiles = 100;
    size_t maxMem = 1ull << 32; // 4GB
    bool premultiply = true;

    mpPtexCache = Ptex::PtexCache::create(maxFiles, maxMem, premultiply, nullptr, &errorHandler);
}

PTexBuilder::PTexData PTexBuilder::loadPTexData(const std::string& filename, bool useSrgb, float scale)
{
    PTexData texels;
    auto encodingFunc = [useSrgb](float3 x) { return useSrgb ? float3(pow(x.r, 2.2f), pow(x.g, 2.2f), pow(x.b, 2.2f)) : x; };
    Ptex::String error;
    Ptex::PtexTexture* texture = mpPtexCache->get(filename.c_str(), error);
    if (!texture)
    {
        logError("Error loading ptex file: {}", error.c_str());
    }
    else
    {
        int nFaces = texture->numFaces();
        logDebug("{}: nFaces = {}", filename, nFaces);
        const int nc = texture->numChannels();
        texture->release();

        Ptex::PtexFilter::Options opts(Ptex::PtexFilter::FilterType::f_bspline);
        auto* filter = Ptex::PtexFilter::getFilter(texture, opts);

        if (nc != 1 && nc != 3)
        {
            logError("Only 1 or 3 channel ptex files are supported");
        }
        else
        {
            // convert the original ptex storage to a falcor texture
            texels.reserve(nFaces);
            for (int i = 0; i < nFaces; i++)
            {
                constexpr float filterWidth = 0.75f;

                int firstChan = 0;
                float3 rgb;

                filter->eval(&rgb.x, firstChan, nc, i, 0.5f, 0.5f, filterWidth, filterWidth, filterWidth, filterWidth);
                if (nc == 1)
                {
                    rgb.b = rgb.g = rgb.r;
                }
                rgb = encodingFunc(rgb) * scale;
                texels.emplace_back(rgb.x, rgb.y, rgb.z, 1.f);
            }
        }
        filter->release();
    }
    return texels;
}

int PTexBuilder::addTextureData(const ref<Material>& pMaterial, fstd::span<const PTexTexel> texels)
{
    int offset = mPTexData.size();
    mPTexData.insert(mPTexData.end(), texels.begin(), texels.end());
    mFaceIndexOffsets[pMaterial] = offset;
    return offset;
}

int PTexBuilder::findFaceIndexOffset(const ref<Material>& pMaterial) const
{
    auto it = mFaceIndexOffsets.find(pMaterial);
    if (it == mFaceIndexOffsets.end())
    {
        return -1;
    }
    return it->second;
}

void PTexBuilder::buildPTexAtlas()
{
    if (mPTexData.empty())
        return;

    int atlasHeight = std::ceil(float(mPTexData.size()) / mAtlasWidth);

    // BC6 compress by 4x4 blocks, so we need to pad the data to be a multiple of 4.
    atlasHeight = align_to(4, atlasHeight);
    mPTexData.resize(mAtlasWidth * atlasHeight, PTexTexel(0.0f));
    logInfo("Creating PTex atlas of size {}x{}", mAtlasWidth, atlasHeight);

    std::filesystem::path path = std::filesystem::temp_directory_path() / "ptexAtlas.dds";

    {
        // Create a temporary DDS file to apply BC6 compression
        auto bitmap = Bitmap::create(mAtlasWidth, atlasHeight, ResourceFormat::RGBA16Float, (const uint8_t*)mPTexData.data());
        mPTexData.clear();
        ImageIO::saveToDDS(path, *bitmap, ImageIO::CompressionMode::BC6, false);
    }

    mpAtlas = ImageIO::loadTextureFromDDS(mpDevice, path, false);
    mpAtlas->setIsPTex(true);
    logInfo("PTex atlas allocated size: {}MB", mpAtlas->getTextureSizeInBytes() / float(1024 * 1024));
}

void PTexBuilder::assignPTexToMaterials()
{
    for (auto [pMaterial, _] : mFaceIndexOffsets)
    {
        if (auto pBasicMaterial = dynamic_ref_cast<BasicMaterial>(pMaterial))
        {
            pBasicMaterial->setBaseColorTexture(mpAtlas);
        }
    }
}

void PTexBuilder::destroyPTexCache()
{
    mpPtexCache->release();
    mpPtexCache = nullptr;
}

} // namespace Falcor
