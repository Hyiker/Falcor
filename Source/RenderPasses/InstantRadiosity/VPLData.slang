/** Virtual point light source data.
 */
struct VPLData
{
    float3 position; /// World-space position.
    float3 radiant;  ///< Radiant intensity, W/sr.
    float3 normal;   ///< VPL direction in world space.

    __init(float3 position, float3 radiant, float3 normal)
    {
        this.position = position;
        this.radiant = radiant;
        this.normal = normal;
    }

    float3 getPosition() { return position.xyz; }

    float3 getRadiant() { return radiant.rgb; }

    float3 getNormal() { return normal.xyz; }
};
