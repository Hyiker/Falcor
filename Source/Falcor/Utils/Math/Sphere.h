#pragma once

namespace Falcor
{
/**
 * @brief Sphere bounding structure stored by its center and radius.
 *
 */
struct Sphere
{
    float3 center;       ///< Center of the sphere.
    float radius = -1.f; ///< Radius of the sphere.

    Sphere() = default;

    /// Construct sphere initialized to center and radius.
    Sphere(const float3& c, float r) : center(c), radius(r) {}

    /// Set sphere to center and radius.
    void set(const float3& c, float r)
    {
        center = c;
        radius = r;
    }

    /// Invalidates the sphere.
    void invalidate()
    {
        center = float3(0);
        radius = -1.f;
    }

    /// Returns true if sphere is valid (radius is zero or larger).
    bool valid() const { return radius >= 0; }
};
} // namespace Falcor
