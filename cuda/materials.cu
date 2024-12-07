#include <optix.h>
#include <vector_types.h>

#include "helpers.h"
#include "Soltrace.h"

// Launch parameters for soltrace
extern "C" {
    __constant__ soltrace::LaunchParams params;
}

static __device__ __inline__ soltrace::PerRayData getPayload()
{
    soltrace::PerRayData prd;
    prd.ray_path_index = optixGetPayload_0();
    prd.depth = optixGetPayload_1();
    return prd;
}

static __device__ __inline__ void setPayload(const soltrace::PerRayData& prd)
{
    optixSetPayload_0(prd.ray_path_index);
    optixSetPayload_1(prd.depth);
}

extern "C" __global__ void __closesthit__mirror()
{
    //const soltrace::HitGroupData* sbt_data = reinterpret_cast<soltrace::HitGroupData*>(optixGetSbtDataPointer());
    //const MaterialData::Mirror& mirror = sbt_data->material_data.mirror;

    // Fetch the normal vector from the hit attributes passed by OptiX
    float3 object_normal = make_float3( __uint_as_float( optixGetAttribute_0() ), __uint_as_float( optixGetAttribute_1() ),
                                        __uint_as_float( optixGetAttribute_2() ) );
    // Transform the object-space normal to world space using OptiX built-in function
    float3 world_normal  = normalize( optixTransformNormalFromObjectToWorldSpace( object_normal ) );
    // Compute the facing normal, which handles the direction of the normal based on the incoming ray direction
    float3 ffnormal      = faceforward( world_normal, -optixGetWorldRayDirection(), world_normal );

    // Get the incoming ray's origin, direction, and max t (intersection distance)
    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir  = optixGetWorldRayDirection();
    const float  ray_t    = optixGetRayTmax();

    // Compute the hit point of the ray using its origin and direction, scaled by the intersection distance (ray_t)
    const float3 hit_point = ray_orig + ray_t * ray_dir;

    soltrace::PerRayData prd = getPayload();
    const int new_depth = prd.depth + 1;    // Increment the ray depth for recursive tracing

    // Calculate ideal reflection direction using OptiX's built-in reflect function
    float3 reflected_dir = reflect(ray_dir, ffnormal);

    // TODO: Add some noise here

    // Check if the maximum recursion depth has not been reached
    if (new_depth < params.max_depth) {
        // Store the hit point in the hit point buffer (used for visualization or further calculations)
        params.hit_point_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(1.0f, hit_point);
        // Store the reflected direction in its buffer (used for visualization or further calculations)
        /*
        params.reflected_dir_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(1.0f, reflected_dir);
        */

        // Trace the reflected ray
        prd.depth = new_depth;
        optixTrace(
            params.handle,          // The handle to the acceleration structure
            hit_point,              // The starting point of the reflected ray
            reflected_dir,          // The direction of the reflected ray
            0.01f,                  // A small offset to avoid self-intersection (shadow acne)
            1e16f,                  // Maximum distance the ray can travel
            0.0f,                   // Ray time (used for time-dependent effects)
            OptixVisibilityMask(1), // Visibility mask (defines what the ray can interact with)
            OPTIX_RAY_FLAG_NONE,    // Ray flags (no special flags for now)
            soltrace::RAY_TYPE_RADIANCE,  // Use the radiance ray type
            soltrace::RAY_TYPE_COUNT,     // Total number of ray types
            soltrace::RAY_TYPE_RADIANCE,  // The ray type's offset into the SBT
            reinterpret_cast<unsigned int&>(prd.ray_path_index), // Pass the ray path index
            reinterpret_cast<unsigned int&>(prd.depth)           // Pass the updated depth
        );
    }

    setPayload(prd);
}

extern "C" __global__ void __closesthit__receiver()
{
    // Retrieve the hit group data and access the parallelogram geometry
    const soltrace::HitGroupData* sbt_data = reinterpret_cast<soltrace::HitGroupData*>(optixGetSbtDataPointer());
    const GeometryData::Parallelogram& parallelogram = sbt_data->geometry_data.getParallelogram();

    /*
    float3 object_normal = make_float3( __uint_as_float( optixGetAttribute_0() ), __uint_as_float( optixGetAttribute_1() ),
                                        __uint_as_float( optixGetAttribute_2() ) );
    float3 world_normal  = normalize( optixTransformNormalFromObjectToWorldSpace( object_normal ) );
    float3 ffnormal      = faceforward( world_normal, -optixGetWorldRayDirection(), world_normal );
    */

    // Incident ray properties (origin, direction, and max t distance)
    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir  = optixGetWorldRayDirection();
    const float  ray_t    = optixGetRayTmax();

    // Compute the normal of the receiver and dot with ray direction to determine which side was hit
    // TODO: this normal is hard coded based on how the geometry was defined, need to make more robust
    const float3 receiver_normal = cross(parallelogram.v2, parallelogram.v1);
    const float dot_product = dot(ray_dir, receiver_normal);

    float3 hit_point = ray_orig + ray_t * ray_dir;

    soltrace::PerRayData prd = getPayload();
    const int new_depth = prd.depth + 1;

    // Check if the ray hits the receiver surface (dot product negative means ray is hitting the front face)
    if (dot_product < 0.0f) {
        if (new_depth < params.max_depth) {
            params.hit_point_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(2.0f, hit_point);
            prd.depth = new_depth;
        }
    }

    setPayload(prd);
}

extern "C" __global__ void __miss__ms()
{
    // No action is taken here.
    // This function simply acts as a terminator for rays that miss all geometry.
    
    /*
    soltrace::PerRayData prd = getPayload();
    const int new_depth = prd.depth + 1;

    if (new_depth < params.max_depth) {
        params.hit_point_buffer[params.max_depth * ray_path_index + new_depth] = make_float4(4.0f);
    }
    */

    // Set the payload values to 0, indicating that the ray missed all geometry.
    optixSetPayload_0(0);  // Default value
    optixSetPayload_1(0);
}