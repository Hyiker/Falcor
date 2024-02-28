/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "TriangleMesh.h"
#include "GlobalState.h"
#include "Core/Error.h"
#include "Core/Platform/OS.h"
#include "Utils/Logger.h"
#include "Utils/Scripting/ScriptBindings.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <cmath>
#include <rply.h>
#include <functional>

namespace Falcor
{
ref<TriangleMesh> TriangleMesh::create()
{
    return ref<TriangleMesh>(new TriangleMesh());
}

ref<TriangleMesh> TriangleMesh::create(const VertexList& vertices, const IndexList& indices, bool frontFaceCW)
{
    return ref<TriangleMesh>(new TriangleMesh(vertices, indices, frontFaceCW));
}

ref<TriangleMesh> TriangleMesh::createDummy()
{
    VertexList vertices = {{{0.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f}}};
    IndexList indices = {0, 0, 0};
    return create(vertices, indices);
}

ref<TriangleMesh> TriangleMesh::createQuad(float2 size)
{
    float2 hsize = 0.5f * size;
    float3 normal{0.f, 1.f, 0.f};
    bool frontFaceCW = size.x * size.y < 0.f;

    VertexList vertices{
        {{-hsize.x, 0.f, -hsize.y}, normal, {0.f, 0.f}},
        {{hsize.x, 0.f, -hsize.y}, normal, {1.f, 0.f}},
        {{-hsize.x, 0.f, hsize.y}, normal, {0.f, 1.f}},
        {{hsize.x, 0.f, hsize.y}, normal, {1.f, 1.f}},
    };

    IndexList indices{
        2,
        1,
        0,
        1,
        2,
        3,
    };

    return create(vertices, indices, frontFaceCW);
}

ref<TriangleMesh> TriangleMesh::createDisk(float radius, uint32_t segments)
{
    std::vector<Vertex> vertices(segments + 1);
    std::vector<uint32_t> indices(segments * 3);

    float3 normal = {0.f, 1.f, 0.f};
    vertices[0] = {{0.f, 0.f, 0.f}, normal, {0.5f, 0.5f}};

    for (uint32_t i = 0; i < segments; ++i)
    {
        float phi = float(i) / float(segments) * 2.f * (float)M_PI;
        float c = std::cos(phi);
        float s = -std::sin(phi);
        vertices[i + 1] = {{c * radius, 0.f, s * radius}, normal, {0.5f + c * 0.5f, 0.5f + s * 0.5f}};

        indices[i * 3] = 0;
        indices[i * 3 + 1] = i + 1;
        indices[i * 3 + 2] = ((i + 1) % segments) + 1;
    }

    return create(vertices, indices, false);
}

ref<TriangleMesh> TriangleMesh::createCube(float3 size)
{
    const float3 positions[6][4] = {
        {{-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, -0.5f}},
        {{-0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}},
        {{-0.5f, 0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}},
        {{0.5f, 0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}},
        {{-0.5f, 0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}},
        {{0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}},
    };

    const float3 normals[6] = {
        {0.f, -1.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, -1.f},
        {0.f, 0.f, 1.f},
        {-1.f, 0.f, 0.f},
        {1.f, 0.f, 0.f},
    };

    const float2 texCoords[4] = {{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}};

    VertexList vertices;
    IndexList indices;

    float3 sign = {size.x < 0.f ? -1.f : 1.f, size.y < 0.f ? -1.f : 1.f, size.z < 0.f ? -1.f : 1.f};
    bool frontFaceCW = size.x * size.y * size.z < 0.f;

    for (size_t i = 0; i < 6; ++i)
    {
        uint32_t idx = (uint32_t)vertices.size();
        indices.emplace_back(idx);
        indices.emplace_back(idx + 2);
        indices.emplace_back(idx + 1);
        indices.emplace_back(idx);
        indices.emplace_back(idx + 3);
        indices.emplace_back(idx + 2);

        for (size_t j = 0; j < 4; ++j)
        {
            vertices.emplace_back(Vertex{positions[i][j] * size, normals[i] * sign, texCoords[j]});
        }
    }

    return create(vertices, indices, frontFaceCW);
}

ref<TriangleMesh> TriangleMesh::createSphere(float radius, uint32_t segmentsU, uint32_t segmentsV)
{
    VertexList vertices;
    IndexList indices;

    // Create vertices.
    for (uint32_t v = 0; v <= segmentsV; ++v)
    {
        for (uint32_t u = 0; u <= segmentsU; ++u)
        {
            float2 uv = float2(u / float(segmentsU), v / float(segmentsV));
            float theta = uv.x * 2.f * (float)M_PI;
            float phi = uv.y * (float)M_PI;
            float3 dir = float3(std::cos(theta) * std::sin(phi), std::cos(phi), std::sin(theta) * std::sin(phi));
            vertices.emplace_back(Vertex{dir * radius, dir, uv});
        }
    }

    // Create indices.
    for (uint32_t v = 0; v < segmentsV; ++v)
    {
        for (uint32_t u = 0; u < segmentsU; ++u)
        {
            uint32_t i0 = v * (segmentsU + 1) + u;
            uint32_t i1 = v * (segmentsU + 1) + (u + 1) % (segmentsU + 1);
            uint32_t i2 = (v + 1) * (segmentsU + 1) + u;
            uint32_t i3 = (v + 1) * (segmentsU + 1) + (u + 1) % (segmentsU + 1);

            indices.emplace_back(i0);
            indices.emplace_back(i1);
            indices.emplace_back(i2);

            indices.emplace_back(i2);
            indices.emplace_back(i1);
            indices.emplace_back(i3);
        }
    }

    return create(vertices, indices);
}

ref<TriangleMesh> TriangleMesh::createFromFile(const std::filesystem::path& path, ImportFlags importFlags)
{
    if (!std::filesystem::exists(path))
    {
        logWarning("Failed to load triangle mesh from '{}': File not found", path);
        return nullptr;
    }

    Assimp::Importer importer;

    unsigned int flags = aiProcess_FlipUVs | aiProcess_Triangulate | aiProcess_PreTransformVertices;
    flags |= is_set(importFlags, ImportFlags::GenSmoothNormals) ? aiProcess_GenSmoothNormals : aiProcess_GenNormals;
    flags |= is_set(importFlags, ImportFlags::JoinIdenticalVertices) ? aiProcess_JoinIdenticalVertices : 0;

    const aiScene* scene = nullptr;

    if (hasExtension(path, "gz"))
    {
        auto decompressed = decompressFile(path);
        scene = importer.ReadFileFromMemory(decompressed.data(), decompressed.size(), flags);
    }
    else
    {
        scene = importer.ReadFile(path.string().c_str(), flags);
    }

    if (!scene)
    {
        logWarning("Failed to load triangle mesh from '{}': {}", path, importer.GetErrorString());
        return nullptr;
    }

    VertexList vertices;
    IndexList indices;

    size_t vertexCount = 0;
    size_t indexCount = 0;

    for (size_t meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
    {
        vertexCount += scene->mMeshes[meshIdx]->mNumVertices;
        indexCount += scene->mMeshes[meshIdx]->mNumFaces * 3;
    }

    vertices.reserve(vertexCount);
    indices.reserve(indexCount);

    for (size_t meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
    {
        size_t indexBase = vertices.size();
        auto mesh = scene->mMeshes[meshIdx];
        for (size_t vertexIdx = 0; vertexIdx < mesh->mNumVertices; ++vertexIdx)
        {
            const auto& vertex = mesh->mVertices[vertexIdx];
            const auto& normal = mesh->mNormals[vertexIdx];
            const auto& texCoord = mesh->mTextureCoords[0] ? mesh->mTextureCoords[0][vertexIdx] : aiVector3D(0.f);
            vertices.emplace_back(Vertex{
                float3(vertex.x, vertex.y, vertex.z), float3(normal.x, normal.y, normal.z), float2(texCoord.x, texCoord.y)});
        }
        for (size_t faceIdx = 0; faceIdx < mesh->mNumFaces; ++faceIdx)
        {
            const auto& face = mesh->mFaces[faceIdx];
            if (face.mNumIndices != 3)
            {
                logWarning("Failed to load triangle mesh from '{}': Broken face data", path);
                return nullptr;
            }
            for (size_t i = 0; i < 3; ++i)
                indices.emplace_back((uint32_t)(indexBase + face.mIndices[i]));
        }
    }

    return create(vertices, indices);
}

ref<TriangleMesh> TriangleMesh::createFromFile(const std::filesystem::path& path, bool smoothNormals)
{
    ImportFlags flags = smoothNormals ? ImportFlags::GenSmoothNormals : ImportFlags::None;
    return createFromFile(path, flags);
}

void rply_message_callback(p_ply ply, const char* message)
{
    logWarning("rply: {}", message);
}

/* Callback to handle vertex data from RPly */
int rply_vertex_callback(p_ply_argument argument)
{
    float* buffer;
    long index, offset;

    ply_get_argument_user_data(argument, (void**)&buffer, &offset);
    ply_get_argument_element(argument, nullptr, &index);

    constexpr int vertexStride = sizeof(TriangleMesh::Vertex) / sizeof(float);

    buffer[index * vertexStride + offset] = (float)ply_get_argument_value(argument);

    return 1;
}

struct FaceCallbackContext
{
    int face[4];
    std::vector<uint32_t> triIndices, quadIndices;
};

/* Callback to handle face data from RPly */
int rply_face_callback(p_ply_argument argument)
{
    FaceCallbackContext* context;
    long flags;
    ply_get_argument_user_data(argument, (void**)&context, &flags);

    long length, value_index;
    ply_get_argument_property(argument, nullptr, &length, &value_index);

    if (length != 3 && length != 4)
    {
        logWarning(
            "plymesh: Ignoring face with {} vertices (only triangles and quads "
            "are supported!)",
            (int)length
        );
        return 1;
    }
    else if (value_index < 0)
    {
        return 1;
    }

    if (value_index >= 0)
        context->face[value_index] = (int)ply_get_argument_value(argument);

    if (value_index == length - 1)
    {
        if (length == 3)
            for (int i = 0; i < 3; ++i)
                context->triIndices.push_back(context->face[i]);
        else
        {
            FALCOR_CHECK(length == 4, "Invalid face length");

            // Note: modify order since we're specifying it as a blp...
            context->quadIndices.push_back(context->face[0]);
            context->quadIndices.push_back(context->face[1]);
            context->quadIndices.push_back(context->face[3]);
            context->quadIndices.push_back(context->face[2]);
        }
    }

    return 1;
}

int rply_faceindex_callback(p_ply_argument argument)
{
    float* buffer;
    long index, offset;

    ply_get_argument_user_data(argument, (void**)&buffer, &offset);
    ply_get_argument_element(argument, nullptr, &index);

    constexpr int vertexStride = sizeof(TriangleMesh::Vertex) / sizeof(float);

    int faceIndex = (int)ply_get_argument_value(argument);
    FALCOR_CHECK(faceIndex >= 0, "Invalid face index");
    // store face index in the texCoord.x and clear texCoord.y
    buffer[index * vertexStride + offset] = (int)ply_get_argument_value(argument);
    buffer[index * vertexStride + offset + 1] = 0.f;

    return 1;
}

template<class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct VertexHash
{
    auto operator()(const Falcor::TriangleMesh::Vertex& v) const
    {
        size_t seed = 42;
        hash_combine(seed, v.position.x);
        hash_combine(seed, v.position.y);
        hash_combine(seed, v.position.z);
        hash_combine(seed, v.normal.x);
        hash_combine(seed, v.normal.y);
        hash_combine(seed, v.normal.z);
        hash_combine(seed, v.texCoord.x);
        hash_combine(seed, v.texCoord.y);
        return seed;
    }
};

ref<TriangleMesh> TriangleMesh::createFromPlyFile(const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path))
    {
        logWarning("Failed to load triangle mesh from '{}': File not found", path);
        return nullptr;
    }

    if (!hasExtension(path, "ply"))
    {
        logWarning("Failed to load triangle mesh from '{}': Must be a '.ply' file", path);
        return nullptr;
    }

    p_ply ply = ply_open(path.string().c_str(), rply_message_callback, 0, nullptr);
    if (!ply)
    {
        logWarning("Couldn't open PLY file '{}'", path);
        return nullptr;
    }

    if (ply_read_header(ply) == 0)
    {
        logWarning("Unable to read the header of PLY file '{}'", path);
        ply_close(ply);
        return nullptr;
    }

    VertexList vertices;
    IndexList triangleIndices, quadIndices;

    p_ply_element element = nullptr;
    size_t vertexCount = 0, faceCount = 0;

    /* Inspect the structure of the PLY file */
    while ((element = ply_get_next_element(ply, element)) != nullptr)
    {
        const char* name;
        long nInstances;

        ply_get_element_info(element, &name, &nInstances);
        if (strcmp(name, "vertex") == 0)
            vertexCount = nInstances;
        else if (strcmp(name, "face") == 0)
            faceCount = nInstances;
    }

    if (vertexCount == 0 || faceCount == 0)
    {
        logWarning("{}: PLY file is invalid! No face/vertex elements found!", path);
        ply_close(ply);
        return nullptr;
    }

    vertices.resize(vertexCount);
    constexpr int positionOffset = offsetof(Vertex, position) / sizeof(float);
    if (ply_set_read_cb(ply, "vertex", "x", rply_vertex_callback, vertices.data(), positionOffset) == 0 ||
        ply_set_read_cb(ply, "vertex", "y", rply_vertex_callback, vertices.data(), positionOffset + 1) == 0 ||
        ply_set_read_cb(ply, "vertex", "z", rply_vertex_callback, vertices.data(), positionOffset + 2) == 0)
    {
        logWarning("{}: Vertex coordinate property not found!", path);
        ply_close(ply);
        return nullptr;
    }
    constexpr int normalOffset = offsetof(Vertex, normal) / sizeof(float);
    if (ply_set_read_cb(ply, "vertex", "nx", rply_vertex_callback, vertices.data(), normalOffset) == 0 ||
        ply_set_read_cb(ply, "vertex", "ny", rply_vertex_callback, vertices.data(), normalOffset + 1) == 0 ||
        ply_set_read_cb(ply, "vertex", "nz", rply_vertex_callback, vertices.data(), normalOffset + 2) == 0)
    {
        logWarning("{}: Vertex normal property not found!", path);
    }

    constexpr int texCoordOffset = offsetof(Vertex, texCoord) / sizeof(float);
    if (((ply_set_read_cb(ply, "vertex", "u", rply_vertex_callback, vertices.data(), texCoordOffset) != 0) &&
         (ply_set_read_cb(ply, "vertex", "v", rply_vertex_callback, vertices.data(), texCoordOffset + 1) != 0)) ||
        ((ply_set_read_cb(ply, "vertex", "s", rply_vertex_callback, vertices.data(), texCoordOffset) != 0) &&
         (ply_set_read_cb(ply, "vertex", "t", rply_vertex_callback, vertices.data(), texCoordOffset + 1) != 0)) ||
        ((ply_set_read_cb(ply, "vertex", "texture_u", rply_vertex_callback, vertices.data(), texCoordOffset) != 0) &&
         (ply_set_read_cb(ply, "vertex", "texture_v", rply_vertex_callback, vertices.data(), texCoordOffset + 1) != 0)) ||
        ((ply_set_read_cb(ply, "vertex", "texture_s", rply_vertex_callback, vertices.data(), texCoordOffset) != 0) &&
         (ply_set_read_cb(ply, "vertex", "texture_t", rply_vertex_callback, vertices.data(), texCoordOffset + 1) != 0)))
    {
    }

    FaceCallbackContext context;
    context.triIndices.reserve(faceCount * 3);
    context.quadIndices.reserve(faceCount * 4);
    if (ply_set_read_cb(ply, "face", "vertex_indices", rply_face_callback, &context, 0) == 0)
    {
        logWarning("{}: vertex indices not found in PLY file", path);
        ply_close(ply);
        return nullptr;
    }

    ply_set_read_cb(ply, "face", "face_indices", rply_faceindex_callback, vertices.data(), texCoordOffset);

    if (ply_read(ply) == 0)
    {
        logWarning("{}: unable to read the contents of PLY file", path);
        ply_close(ply);
        return nullptr;
    }

    triangleIndices = std::move(context.triIndices);
    quadIndices = std::move(context.quadIndices);

    ply_close(ply);

    for (int idx : triangleIndices)
        if (idx < 0 || idx >= vertices.size())
        {
            logWarning(
                "plymesh: Vertex index {} is out of bounds! "
                "Valid range is [0..{})",
                idx,
                int(vertices.size())
            );
            return nullptr;
        }
    for (int idx : quadIndices)
        if (idx < 0 || idx >= vertices.size())
        {
            logWarning(
                "plymesh: Vertex index {} is out of bounds! "
                "Valid range is [0..{})",
                idx,
                int(vertices.size())
            );
            return nullptr;
        }

    // convert quad indices to triangle indices
    for (size_t i = 0; i < quadIndices.size(); i += 4)
    {
        // 0, 1, 2
        triangleIndices.push_back(quadIndices[i]);
        triangleIndices.push_back(quadIndices[i + 1]);
        triangleIndices.push_back(quadIndices[i + 2]);
        // 0, 2, 3
        triangleIndices.push_back(quadIndices[i]);
        triangleIndices.push_back(quadIndices[i + 2]);
        triangleIndices.push_back(quadIndices[i + 3]);
    }
    quadIndices.clear();
    std::unordered_map<Vertex, uint32_t, VertexHash> uniqueVertices;

    IndexList indices;
    indices.reserve(triangleIndices.size());
    // join identical vertices
    for (auto idx : triangleIndices)
    {
        Vertex vertex = vertices[idx];
        if (uniqueVertices.count(vertex) == 0)
        {
            uniqueVertices[vertex] = (uint32_t)vertices.size();
            vertices.push_back(vertex);
        }
        indices.push_back(uniqueVertices[vertex]);
    }

    return create(vertices, triangleIndices);
}

uint32_t TriangleMesh::addVertex(float3 position, float3 normal, float2 texCoord)
{
    mVertices.emplace_back(Vertex{position, normal, texCoord});
    FALCOR_ASSERT(mVertices.size() < std::numeric_limits<uint32_t>::max());
    return (uint32_t)(mVertices.size() - 1);
}

void TriangleMesh::addTriangle(uint32_t i0, uint32_t i1, uint32_t i2)
{
    mIndices.emplace_back(i0);
    mIndices.emplace_back(i1);
    mIndices.emplace_back(i2);
}

void TriangleMesh::applyTransform(const Transform& transform)
{
    applyTransform(transform.getMatrix());
}

void TriangleMesh::applyTransform(const float4x4& transform)
{
    auto invTranspose = float3x3(transpose(inverse(transform)));

    for (auto& vertex : mVertices)
    {
        vertex.position = transformPoint(transform, vertex.position);
        vertex.normal = normalize(transformVector(invTranspose, vertex.normal));
    }

    // Check if triangle winding has flipped and adjust winding order accordingly.
    bool flippedWinding = determinant(float3x3(transform)) < 0.f;
    if (flippedWinding)
        mFrontFaceCW = !mFrontFaceCW;
}

TriangleMesh::TriangleMesh() {}

TriangleMesh::TriangleMesh(const VertexList& vertices, const IndexList& indices, bool frontFaceCW)
    : mVertices(vertices), mIndices(indices), mFrontFaceCW(frontFaceCW)
{}

FALCOR_SCRIPT_BINDING(TriangleMesh)
{
    using namespace pybind11::literals;

    pybind11::enum_<TriangleMesh::ImportFlags> flags(m, "TriangleMeshImportFlags");
    flags.value("Default", TriangleMesh::ImportFlags::Default);
    flags.value("GenSmoothNormals", TriangleMesh::ImportFlags::GenSmoothNormals);
    flags.value("JoinIdenticalVertices", TriangleMesh::ImportFlags::JoinIdenticalVertices);
    ScriptBindings::addEnumBinaryOperators(flags);

    pybind11::class_<TriangleMesh, ref<TriangleMesh>> triangleMesh(m, "TriangleMesh");

    pybind11::class_<TriangleMesh::Vertex> vertex(triangleMesh, "Vertex");
    vertex.def_readwrite("position", &TriangleMesh::Vertex::position);
    vertex.def_readwrite("normal", &TriangleMesh::Vertex::normal);
    vertex.def_readwrite("texCoord", &TriangleMesh::Vertex::texCoord);

    triangleMesh.def_property("name", &TriangleMesh::getName, &TriangleMesh::setName);
    triangleMesh.def_property("frontFaceCW", &TriangleMesh::getFrontFaceCW, &TriangleMesh::setFrontFaceCW);
    triangleMesh.def_property_readonly("vertices", &TriangleMesh::getVertices);
    triangleMesh.def_property_readonly("indices", &TriangleMesh::getIndices);
    triangleMesh.def(pybind11::init(pybind11::overload_cast<>(&TriangleMesh::create)));
    triangleMesh.def("addVertex", &TriangleMesh::addVertex, "position"_a, "normal"_a, "texCoord"_a);
    triangleMesh.def("addTriangle", &TriangleMesh::addTriangle, "i0"_a, "i1"_a, "i2"_a);
    triangleMesh.def_static("createQuad", &TriangleMesh::createQuad, "size"_a = float2(1.f));
    triangleMesh.def_static("createDisk", &TriangleMesh::createDisk, "radius"_a = 1.f, "segments"_a = 32);
    triangleMesh.def_static("createCube", &TriangleMesh::createCube, "size"_a = float3(1.f));
    triangleMesh.def_static("createSphere", &TriangleMesh::createSphere, "radius"_a = 1.f, "segmentsU"_a = 32, "segmentsV"_a = 32);
    triangleMesh.def_static(
        "createFromFile",
        [](const std::filesystem::path& path, bool smoothNormals)
        { return TriangleMesh::createFromFile(getActiveAssetResolver().resolvePath(path), smoothNormals); },
        "path"_a,
        "smoothNormals"_a = false
    ); // PYTHONDEPRECATED
    triangleMesh.def_static(
        "createFromFile",
        [](const std::filesystem::path& path, TriangleMesh::ImportFlags importFlags)
        { return TriangleMesh::createFromFile(getActiveAssetResolver().resolvePath(path), importFlags); },
        "path"_a,
        "importFlags"_a
    ); // PYTHONDEPRECATED

    triangleMesh.def_static(
        "createFromPlyFile",
        [](const std::filesystem::path& path) { return TriangleMesh::createFromPlyFile(getActiveAssetResolver().resolvePath(path)); },
        "path"_a
    ); // PYTHONDEPRECATED
}
} // namespace Falcor
