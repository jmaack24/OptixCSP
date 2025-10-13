#include <optix_device.h>
#include <vector_types.h>
#include "Soltrace.h"
#include <stdio.h>
#include "MaterialDataST.h"


namespace OptixCSP {
    static __device__ __inline__ OptixCSP::PerRayData getPayload()
    {
        OptixCSP::PerRayData prd;
        prd.ray_path_index = optixGetPayload_0();
        prd.depth = optixGetPayload_1();
        return prd;
    }

    static __device__ __inline__ void setPayload(const OptixCSP::PerRayData& prd)
    {
        optixSetPayload_0(prd.ray_path_index);
        optixSetPayload_1(prd.depth);
    }

    // 32-bit avalanche mix (fast, good diffusion)
    static __device__ __inline__ float rng_uniform(uint32_t x) {

        x ^= x >> 16;  
        x *= 0x85EBCA6Bu;
        x ^= x >> 13;  
        x *= 0xC2B2AE35u;
        x ^= x >> 16;  
		return float(x >> 8) * (1.0f / 16777216.0f); // Scale to [0, 1)
    }

}




// Launch parameters for soltrace
extern "C" {
    __constant__ OptixCSP::LaunchParams params;
}

extern "C" __global__ void __closesthit__mirror()
{
	OptixCSP::MaterialData material = params.material_data_array[optixGetPrimitiveIndex()];

    float transmissivity = material.transmissivity;
    bool use_transmmisivity = material.use_refraction;
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

    OptixCSP::PerRayData prd = OptixCSP::getPayload();
    const int new_depth = prd.depth + 1;    // Increment the ray depth for recursive tracing

    // we have two scenarios here
	// if we use refraction, then we look at transmissivity to determine if the ray will refract 
    // or get obsorbed. otherwise, it will get reflected.
    float3 new_dir;
	bool absorbed = false;  // determine whether the ray is absorbed or not, this is montecarlo based, should be applied to reflection and refraction

    if (use_transmmisivity) {
		new_dir = refract(ray_dir, ffnormal);

		// now we figure out the random number to determine if the ray is absorbed or refracted
        uint32_t seed = seed = params.sun_dir_seed ^ (prd.ray_path_index * 0x9E3779B9u)   // golden ratio mix
            ^ (prd.depth * 0x85EBCA6Bu);
		float xi = OptixCSP::rng_uniform(seed); // random number in [0,1)
        if (xi > transmissivity) { absorbed = true; 
        //printf("ray is absorbed! ray index is %d, depth %d\n", prd.ray_path_index, prd.depth); 
        }   // ray is absorbed
    }
    else {
		new_dir = reflect(ray_dir, ffnormal);
    }

    // Check if the maximum recursion depth has not been reached
    if (new_depth < params.max_depth) {
        // Store the hit point in the hit point buffer (used for visualization or further calculations)
        params.hit_point_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(new_depth, hit_point);
        // Store the reflected direction in its buffer (used for visualization or further calculations)
        /*
        params.reflected_dir_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(1.0f, reflected_dir);
        */

        // Trace the reflected ray
        prd.depth = new_depth;
        if (!absorbed) {
            //printf("launching rays that are not absorbed: %d, depth, %d\n", prd.ray_path_index, prd.depth - 1);
            optixTrace(
                params.handle,          // The handle to the acceleration structure
                hit_point,              // The starting point of the reflected ray
                new_dir,          // The direction of the reflected ray
                0.01f,                  // A small offset to avoid self-intersection (shadow acne)
                1e16f,                  // Maximum distance the ray can travel
                0.0f,                   // Ray time (used for time-dependent effects)
                OptixVisibilityMask(1), // Visibility mask (defines what the ray can interact with)
                OPTIX_RAY_FLAG_NONE,    // Ray flags (no special flags for now)
                OptixCSP::RAY_TYPE_RADIANCE,  // Use the radiance ray type
                OptixCSP::RAY_TYPE_COUNT,     // Total number of ray types
                OptixCSP::RAY_TYPE_RADIANCE,  // The ray type's offset into the SBT
                reinterpret_cast<unsigned int&>(prd.ray_path_index), // Pass the ray path index
                reinterpret_cast<unsigned int&>(prd.depth)           // Pass the updated depth
            );

        }
        else {
			//printf("ray %d is absorbed, terminate! depth = %d\n", prd.ray_path_index, prd.depth - 1);
			prd.depth = params.max_depth; // terminate the ray by setting depth to max depth
        }
    }

    setPayload(prd);
}

extern "C" __global__ void __closesthit__receiver()
{
    float3 object_normal = make_float3( __uint_as_float( optixGetAttribute_0() ),
                                        __uint_as_float( optixGetAttribute_1() ),
                                        __uint_as_float( optixGetAttribute_2() ) );

    // Incident ray properties (origin, direction, and max t distance)
    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir  = optixGetWorldRayDirection();
    const float  ray_t    = optixGetRayTmax();
    OptixCSP::PerRayData prd = OptixCSP::getPayload();

    //printf("ray id hitting the receiver: %d, depth %d\n", prd.ray_path_index, prd.depth);

    // Compute the normal of the receiver and dot with ray direction to determine which side was hit
    const float dot_product = dot(ray_dir, object_normal);

    float3 hit_point = ray_orig + ray_t * ray_dir;

    const int new_depth = prd.depth + 1;

    // Check if the ray hits the receiver surface (dot product negative means ray is hitting the front face)
    if (dot_product < 0.0f) {
        if (new_depth < params.max_depth) {
            params.hit_point_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(new_depth, hit_point);
            prd.depth = new_depth;
        }
    }

    setPayload(prd);
}

extern "C" __global__ void __closesthit__receiver__cylinder__y()
{
    //// Retrieve the hit group data and access the parallelogram geometry
    //const OptixCSP::HitGroupData* sbt_data = reinterpret_cast<OptixCSP::HitGroupData*>(optixGetSbtDataPointer());

    

    const OptixCSP::GeometryDataST::Cylinder_Y& cyl = params.geometry_data_array[optixGetPrimitiveIndex()].getCylinder_Y();

    /*
    float3 object_normal = make_float3( __uint_as_float( optixGetAttribute_0() ), __uint_as_float( optixGetAttribute_1() ),
                                        __uint_as_float( optixGetAttribute_2() ) );
    float3 world_normal  = normalize( optixTransformNormalFromObjectToWorldSpace( object_normal ) );
    float3 ffnormal      = faceforward( world_normal, -optixGetWorldRayDirection(), world_normal );
    */

    // Incident ray properties (origin, direction, and max t distance)
    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = optixGetWorldRayDirection();
    const float  ray_t = optixGetRayTmax();

    // Compute the normal of the receiver and dot with ray direction to determine which side was hit
    // TODO: this normal is hard coded based on how the geometry was defined, need to make more robust
    //const float3 receiver_normal = cyl.base_x;
    //const float dot_product = dot(ray_dir, receiver_normal);

    float3 hit_point = ray_orig + ray_t * ray_dir;

    OptixCSP::PerRayData prd = OptixCSP::getPayload();
    const int new_depth = prd.depth + 1;

    // Check if the ray hits the receiver surface (dot product negative means ray is hitting the front face)
    //if (dot_product < 0.0f) {
        if (new_depth < params.max_depth) {
            params.hit_point_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(new_depth, hit_point);
            prd.depth = new_depth;
        }
    //}

    setPayload(prd);
}


// Closest-hit for a parabolic mirror surface.
// This function is nearly identical to __closesthit__mirror(), but is intended to be
// used when the surface is a parabolic mirror. The intersection shader (e.g. 
// __intersection__rectangle_parabolic) reports a normal that already accounts for the curvature.
extern "C" __global__ void __closesthit__mirror__parabolic()
{
    // Optionally, you can access material data if needed:
    // const OptixCSP::HitGroupData* sbt_data = reinterpret_cast<OptixCSP::HitGroupData*>( optixGetSbtDataPointer() );
    // const MaterialData::Mirror& mirror = sbt_data->material_data.mirror;

    // Retrieve the hit normal from the attributes.
    // The intersection shader for the parabolic surface reported the normal (using float3_as_args)
    // in the hit attributes. In many cases this normal is already in world space.
    float3 object_normal = make_float3(__uint_as_float(optixGetAttribute_0()),
        __uint_as_float(optixGetAttribute_1()),
        __uint_as_float(optixGetAttribute_2()));
    // If the normal reported by the intersection shader is in object space you can transform it;
    // if it is already in world space, this call may be omitted.
    float3 world_normal = normalize(optixTransformNormalFromObjectToWorldSpace(object_normal));

    // Ensure that the normal is facing the incoming ray.
    float3 ffnormal = faceforward(world_normal, -optixGetWorldRayDirection(), world_normal);

    // Retrieve ray data.
    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = optixGetWorldRayDirection();
    const float  ray_t = optixGetRayTmax();

    // Compute the hit point.
    const float3 hit_point = ray_orig + ray_t * ray_dir;

    // Retrieve per�ray payload.
    OptixCSP::PerRayData prd = OptixCSP::getPayload();
    const int new_depth = prd.depth + 1; // Increase recursion depth.

    // Compute the reflected ray direction.
    float3 reflected_dir = reflect(ray_dir, ffnormal);

    // (Optional: Add noise to reflected_dir if desired.)

    // If the new depth is below the maximum, trace the reflected ray.
    if (new_depth < params.max_depth) {
        // Save the hit point (for visualization or further processing).
        params.hit_point_buffer[params.max_depth * prd.ray_path_index + new_depth] = make_float4(new_depth, hit_point);

        prd.depth = new_depth;
        optixTrace(
            params.handle,          // Acceleration structure handle.
            hit_point,              // Ray origin.
            reflected_dir,          // Ray direction.
            0.01f,                  // Minimum t to avoid self-intersection.
            1e16f,                  // Maximum t.
            0.0f,                   // Ray time.
            OptixVisibilityMask(1), // Visibility mask.
            OPTIX_RAY_FLAG_NONE,    // Ray flags.
            OptixCSP::RAY_TYPE_RADIANCE,  // Ray type.
            OptixCSP::RAY_TYPE_COUNT,     // Number of ray types.
            OptixCSP::RAY_TYPE_RADIANCE,  // SBT offset for this ray type.
            reinterpret_cast<unsigned int&>(prd.ray_path_index), // Ray path index.
            reinterpret_cast<unsigned int&>(prd.depth)           // Current recursion depth.
        );
    }

    // Store the updated payload.
    setPayload(prd);
}




extern "C" __global__ void __miss__ms()
{
    // No action is taken here.
    // This function simply acts as a terminator for rays that miss all geometry.
    
    /*
    OptixCSP::PerRayData prd = getPayload();
    const int new_depth = prd.depth + 1;

    if (new_depth < params.max_depth) {
        params.hit_point_buffer[params.max_depth * ray_path_index + new_depth] = make_float4(4.0f);
    }
    */

    // Set the payload values to 0, indicating that the ray missed all geometry.
    optixSetPayload_0(0);  // Default value
    optixSetPayload_1(0);
}