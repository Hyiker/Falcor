#pragma once

#include <fstd/span.h>
#include <Ptexture.h>

#include "Scene/Material/Material.h"

namespace Falcor
{

class FALCOR_API PTexBuilder
{
public:
    using PTexTexel = float16_t4;
    using PTexData = std::vector<PTexTexel>;

    PTexBuilder(ref<Device> pDevice);

    auto getAtlasWidth() const { return mAtlasWidth; }
    /**
     * @brief Load texture data from a PTex file.
     *
     * @param filename path to the PTex file.
     * @param useSrgb whether to use sRGB encoding.
     * @param scale scale factor to apply to the texture data.
     * @return The texture data.
     */
    PTexData loadPTexData(const std::string& filename, bool useSrgb, float scale = 1.0f);

    /**
     * @brief Add texture data to the PTex atlas.
     *
     * @param pMaterial The material which uses the texels.
     * @param texels The texture data.
     * @return The offset of the texture data in the PTex atlas.
     */
    int addTextureData(const ref<Material>& pMaterial, fstd::span<const PTexTexel> texels);

    /**
     * @brief Find the offset of the face index offset corresponding to the given material.
     *
     * @param pMaterial The material to find the face index offset for.
     * @return The offset of the face index offset in the PTex atlas,
     * -1 if the material is not found.
     */
    int findFaceIndexOffset(const ref<Material>& pMaterial) const;

    /**
     * @brief Build the PTex atlas.
     * This will consume the texture data.
     */
    void buildPTexAtlas();

    /**
     * @brief Assign the PTex atlas to the materials that use it.
     */
    void assignPTexToMaterials();

    /**
     * @brief manually destroy the mpPtexCache to save memory.
     *
     */
    void destroyPTexCache();

private:
    ref<Device> mpDevice;
    Ptex::PtexCache* mpPtexCache;

    // TODO: smartly choose the atlas width
    const int mAtlasWidth = 4096;

    PTexData mPTexData;
    std::map<ref<Material>, int> mFaceIndexOffsets;
    // TODO: use TextureArray or multiple textures if one atlas is too large
    ref<Texture> mpAtlas;
};

} // namespace Falcor
