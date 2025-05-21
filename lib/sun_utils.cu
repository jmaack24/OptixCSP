#include <optix.h>
#include <curand_kernel.h>
#include <vector_types.h>

#include <cuda/helpers.h>
#include <cuda/random.h>
#include <cuda/Soltrace.h>
#include <cuda_runtime.h>
#include <iostream>

// TODO: need to figure out native support to this, there has to be a way 
// to call atomicMax with float 
__device__ float atomicMaxFloat(float* address, float val) {
    int* address_as_int = (int*)address;
    int old = *address_as_int, assumed;

    do {
        assumed = old;
        float f = __int_as_float(assumed);
        if (f >= val) break;
        old = atomicCAS(address_as_int, assumed, __float_as_int(val));
    } while (assumed != old);

    return __int_as_float(old);
}

__device__ float atomicMinFloat(float* address, float val) {
    int* address_as_int = (int*)address;
    int old = *address_as_int, assumed;

    do {
        assumed = old;
        float f = __int_as_float(assumed);
        if (f <= val) break;
        old = atomicCAS(address_as_int, assumed, __float_as_int(val));
    } while (assumed != old);

    return __int_as_float(old);
}

__device__ inline void getAABBCornersDevice(const OptixAabb& aabb, float3 corners[8]) {
    corners[0] = make_float3(aabb.minX, aabb.minY, aabb.minZ);
    corners[1] = make_float3(aabb.maxX, aabb.minY, aabb.minZ);
	corners[2] = make_float3(aabb.minX, aabb.maxY, aabb.minZ);
	corners[3] = make_float3(aabb.maxX, aabb.maxY, aabb.minZ);
	corners[4] = make_float3(aabb.minX, aabb.minY, aabb.maxZ);
	corners[5] = make_float3(aabb.maxX, aabb.minY, aabb.maxZ);
	corners[6] = make_float3(aabb.minX, aabb.maxY, aabb.maxZ);
	corners[7] = make_float3(aabb.maxX, aabb.maxY, aabb.maxZ);
}


__global__ void calculateMaxD_Kernel(
    const OptixAabb* all_aabbs_D,
    int num_aabbs,
    float3 sun_dir_normalized,
    float* out_max_d_D)
{

    extern __shared__ float sdata[]; // shared for max D reduction

    
    unsigned int thread_id = blockIdx.x * blockDim.x + threadIdx.x;
    if (thread_id >= num_aabbs) {
        sdata[threadIdx.x] = 0.0f;
    }

    // Each thread computes the max D for its assigned AABB
	OptixAabb aabb = all_aabbs_D[thread_id];

    float3 corners[8];
    getAABBCornersDevice(aabb, corners);
	float max_d = 0.0f;

	for (int i = 0; i < 8; i++) {
		max_d = fmaxf(max_d, abs(dot(corners[i], sun_dir_normalized)));
	}

    sdata[threadIdx.x] = max_d;


    __syncthreads();


	// reduction in shared memory
    for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (threadIdx.x < s) {
            sdata[threadIdx.x] = fmaxf(sdata[threadIdx.x], sdata[threadIdx.x + s]);
        }
        __syncthreads(); //
    }

    if (threadIdx.x == 0) {
        atomicMaxFloat(out_max_d_D, sdata[0]);
    }


}

__global__ void calculateUVBounds_Kernel( const OptixAabb* all_aabbs_D,
    int num_aabbs, 
    float d_plane_distance, 
    float3 sun_vec_norm,
    float3 sun_u,
    float3 sun_v,
    float tan_sun_angle,
    float* out_uv_bounds_D // Points to [u_min, u_max, v_min, v_max] on GPU
) {

    // map aabb index to vertices
    unsigned int total_vertices = num_aabbs * 8;
    unsigned int vertex_global_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (vertex_global_idx >= total_vertices) {
        return;
    }

    // Find which AABB and which corner this vertex_global_idx corresponds to
    unsigned int aabb_idx = vertex_global_idx / 8;
    unsigned int corner_idx = vertex_global_idx % 8;

    OptixAabb current_aabb = all_aabbs_D[aabb_idx];
    float3 corners[8];
    getAABBCornersDevice(current_aabb, corners);
    float3 vertex_point = corners[corner_idx];

    float dist_along_sun_axis = abs(dot(vertex_point, sun_vec_norm));
    float buffer = dist_along_sun_axis * tan_sun_angle;

    float3 plane_center = d_plane_distance * sun_vec_norm;
    float3 projected_point = vertex_point - dot(vertex_point - plane_center, sun_vec_norm) * sun_vec_norm;

    float u = dot(projected_point, sun_u);
    float v = dot(projected_point, sun_v);


    // TODO: shared memory for this? 
    atomicMinFloat(&out_uv_bounds_D[0], u - buffer);
    atomicMaxFloat(&out_uv_bounds_D[1], u + buffer);
    atomicMinFloat(&out_uv_bounds_D[2], v - buffer);
    atomicMaxFloat(&out_uv_bounds_D[3], v + buffer);
}


// == Host Wrapper for launching calculateMaxD_Kernel ==
void compute_d_on_gpu(const OptixAabb* all_aabbs_D,
                      int num_aabbs,
                      float3 sun_dir_normalized,
                      float* d_out_max_d_on_gpu) {

    float initial_d_val = 0.0f;
    cudaMemcpy(d_out_max_d_on_gpu, &initial_d_val, sizeof(float), cudaMemcpyHostToDevice);


    int threads_per_block = 256;
    int blocks_per_grid = (num_aabbs + threads_per_block - 1) / threads_per_block;
    size_t shared_mem_size = threads_per_block * sizeof(float); // For the reduction in the kernel

    calculateMaxD_Kernel<<<blocks_per_grid, threads_per_block, shared_mem_size>>>(all_aabbs_D, num_aabbs, sun_dir_normalized, d_out_max_d_on_gpu);
}

// wrapper for launching uv bounds kernel
void compute_uv_bounds_on_gpu(
    const OptixAabb* d_all_aabbs,
    int num_aabbs,
    float d_plane_val,
    const float3& sun_vector_normalized,
    const float3& sun_u_basis,
    const float3& sun_v_basis,
    float tan_max_angle,
    float* d_out_uv_bounds
){

    float initial_bounds[4] = { FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX };
    cudaMemcpy(d_out_uv_bounds, initial_bounds, 4 * sizeof(float), cudaMemcpyHostToDevice);

    unsigned int total_vertices = num_aabbs * 8;
    if (total_vertices == 0) return;

    int threads_per_block = 256;
    int blocks_per_grid = (total_vertices + threads_per_block - 1) / threads_per_block;

    calculateUVBounds_Kernel<<<blocks_per_grid, threads_per_block>>>(d_all_aabbs, num_aabbs, d_plane_val, sun_vector_normalized, sun_u_basis, sun_v_basis, tan_max_angle, d_out_uv_bounds);
}