#include <glad/glad.h> // Needs to be included before gl_interop

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>

#include <sampleConfig.h>

#include <cuda/Soltrace.h>

#include <sutil/CUDAOutputBuffer.h>
#include <sutil/Exception.h>
#include <sutil/Matrix.h>
#include <sutil/Record.h>
#include <sutil/sutil.h>
#include <sutil/vec_math.h>

#include <GLFW/glfw3.h>
#include <iomanip>
#include <cstring>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

typedef sutil::Record<soltrace::HitGroupData> HitGroupRecord;

const uint32_t OBJ_COUNT = 4;
const int      max_trace = 5;

struct SoltraceState
{
    OptixDeviceContext          context                         = 0;
    OptixTraversableHandle      gas_handle                      = {};
    CUdeviceptr                 d_gas_output_buffer             = {};

    OptixModule                 geometry_module                 = 0;
    OptixModule                 shading_module                  = 0;
    OptixModule                 sun_module                      = 0;

    OptixProgramGroup           raygen_prog_group               = 0;
    OptixProgramGroup           radiance_miss_prog_group        = 0;
    OptixProgramGroup           radiance_mirror_prog_group      = 0;
    OptixProgramGroup           radiance_receiver_prog_group    = 0;

    OptixPipeline               pipeline                        = 0;
    OptixPipelineCompileOptions pipeline_compile_options        = {};

    CUstream                    stream                          = 0;
    
    soltrace::LaunchParams      params;
    soltrace::LaunchParams*     d_params                        = nullptr;

    OptixShaderBindingTable     sbt                             = {};

    // TODO: list of geometries - add geometries first and then iterate through list to create SBT
};

// Scene Setup
const GeometryData::Parallelogram heliostat1(
    make_float3(-1.0f, 0.0f, 0.0f),    // v1
    make_float3(0.0f, 1.897836f, 0.448018f),    // v2
    make_float3(0.5f, 4.051082f, -0.224009f)  // anchor
);
const GeometryData::Parallelogram heliostat2(
    make_float3(0.0f, 1.0f, 0.0f),    // v1
    make_float3(1.897836f, 0.0f, 0.448018f),    // v2
    make_float3(4.051082f, -0.5f, -0.224009f)  // anchor
);
const GeometryData::Parallelogram heliostat3(
    make_float3(0.0f, -1.0f, 0.0f),    // v1
    make_float3(-1.897836f, 0.0f, 0.448018f),    // v2
    make_float3(-4.051082f, 0.5f, -0.224009f)  // anchor
);
const GeometryData::Parallelogram receiver(
    make_float3(2.0f, 0.0f, 0.0f),    // v1
    make_float3(0.0f, 1.788854f, 0.894428f),    // v2
    make_float3(-1.0f, -0.894427f, 9.552786f)     // anchor
);

// Compute an axis-aligned bounding box (AABB) for a parallelogram.
//   v1, v2: Vectors defining the parallelogram's sides.
//   anchor: The anchor point of the parallelogram.
inline OptixAabb parallelogram_bound( float3 v1, float3 v2, float3 anchor )
{
    const float3 tv1  = v1 / dot( v1, v1 );
    const float3 tv2  = v2 / dot( v2, v2 );
    // Compute the four corners of the parallelogram in 3D space.
    const float3 p00  = anchor;                 // Lower-left corner
    const float3 p01  = anchor + tv1;           // Lower-right corner
    const float3 p10  = anchor + tv2;           // Upper-left corner
    const float3 p11  = anchor + tv1 + tv2;     // Upper-right corner

    float3 m_min = fminf( fminf( p00, p01 ), fminf( p10, p11 ));
    float3 m_max = fmaxf( fmaxf( p00, p01 ), fmaxf( p10, p11 ));
    return {
        m_min.x, m_min.y, m_min.z,
        m_max.x, m_max.y, m_max.z
    };
}

// Build a GAS (Geometry Acceleration Structure) for the scene.
static void buildGas(
    const SoltraceState &state,
    const OptixAccelBuildOptions &accel_options,
    const OptixBuildInput &build_input,
    OptixTraversableHandle &gas_handle,
    CUdeviceptr &d_gas_output_buffer
    ) {

    OptixAccelBufferSizes gas_buffer_sizes;     // Holds required sizes for temp and output buffers.
    CUdeviceptr d_temp_buffer_gas;              // Temporary buffer for building the GAS.

    // Query the memory usage required for building the GAS.
    OPTIX_CHECK( optixAccelComputeMemoryUsage(
        state.context,
        &accel_options,
        &build_input,
        1,
        &gas_buffer_sizes));

    // Allocate memory for the temporary buffer on the device.
    CUDA_CHECK( cudaMalloc(
        reinterpret_cast<void**>( &d_temp_buffer_gas ),
        gas_buffer_sizes.tempSizeInBytes));

    // Non-compacted output and size of compacted GAS
    CUdeviceptr d_buffer_temp_output_gas_and_compacted_size;
    size_t compactedSizeOffset = roundUp<size_t>( gas_buffer_sizes.outputSizeInBytes, 8ull );
    CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &d_buffer_temp_output_gas_and_compacted_size ),
                compactedSizeOffset + 8
                ) );

    // Emit property to store the compacted GAS size.
    OptixAccelEmitDesc emitProperty = {};
    emitProperty.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitProperty.result = (CUdeviceptr)((char*)d_buffer_temp_output_gas_and_compacted_size + compactedSizeOffset);

    // Build the GAS.
    OPTIX_CHECK( optixAccelBuild(
        state.context,                                  // OptiX context
        0,                                              // CUDA stream (default is 0)
        &accel_options,                                 // Acceleration build options
        &build_input,                                   // Build inputs
        1,                                              // Number of build inputs
        d_temp_buffer_gas,                              // Temporary buffer
        gas_buffer_sizes.tempSizeInBytes,               // Size of temporary buffer
        d_buffer_temp_output_gas_and_compacted_size,    // Output buffer
        gas_buffer_sizes.outputSizeInBytes,             // Size of output buffer
        &gas_handle,                                    // Output handle
        &emitProperty,                                  // Emitted properties
        1) );                                           // Number of emitted properties
        
    CUDA_CHECK( cudaFree( (void*)d_temp_buffer_gas ) );

    size_t compacted_gas_size;
    CUDA_CHECK( cudaMemcpy( &compacted_gas_size, (void*)emitProperty.result, sizeof(size_t), cudaMemcpyDeviceToHost ) );

    // If the compacted GAS size is smaller, allocate a smaller buffer and compact the GAS
    if( compacted_gas_size < gas_buffer_sizes.outputSizeInBytes )
    {
        CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_gas_output_buffer ), compacted_gas_size ) );

        // use handle as input and output
        OPTIX_CHECK( optixAccelCompact( state.context, 0, gas_handle, d_gas_output_buffer, compacted_gas_size, &gas_handle ) );

        CUDA_CHECK( cudaFree( (void*)d_buffer_temp_output_gas_and_compacted_size ) );
    }
    else
    {
        d_gas_output_buffer = d_buffer_temp_output_gas_and_compacted_size;
    }
}

// Print a float3 structure
void printFloat3(const char* label, const float3& vec) {
    std::cout << label << ": (" << vec.x << ", " << vec.y << ", " << vec.z << ")\n";
}

// Build custom primitives (parallelograms for now, TODO generalize)
void createGeometry(SoltraceState& state)
{
    // Define the AABBs for each object in the scene using the parallelogram bounds.
    OptixAabb aabb[OBJ_COUNT] = { parallelogram_bound(heliostat1.v1, heliostat1.v2, heliostat1.anchor),
                                    parallelogram_bound(heliostat2.v1, heliostat2.v2, heliostat2.anchor),
                                    parallelogram_bound(heliostat3.v1, heliostat3.v2, heliostat3.anchor),
                                    parallelogram_bound(receiver.v1, receiver.v2, receiver.anchor) };

    // Initialize the overall min and max using the first object's AABB
    float3 overallMin = { aabb[0].minX, aabb[0].minY, aabb[0].minZ };
    float3 overallMax = { aabb[0].maxX, aabb[0].maxY, aabb[0].maxZ };

    // Loop through the remaining AABBs to find the overall scene bounds
    for (int i = 1; i < OBJ_COUNT; ++i) {
        float3 currentMin = { aabb[i].minX, aabb[i].minY, aabb[i].minZ };
        float3 currentMax = { aabb[i].maxX, aabb[i].maxY, aabb[i].maxZ };

        // Update the overall min and max
        overallMin = fminf(overallMin, currentMin);
        overallMax = fmaxf(overallMax, currentMax);
    }

    // DEBUG: Print the results
    /*
    printFloat3("Overall Min", overallMin);
    printFloat3("Overall Max", overallMax);
    */

    // Allocate a result array to return the overall min and max
    state.params.scene_aabb = {
        overallMin.x, overallMin.y, overallMin.z,
        overallMax.x, overallMax.y, overallMax.z
    };

    // Allocate memory on the device for the AABB array.
    CUdeviceptr d_aabb;
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_aabb
        ), OBJ_COUNT * sizeof( OptixAabb ) ) );
    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( d_aabb ),
                &aabb,
                OBJ_COUNT * sizeof( OptixAabb ),
                cudaMemcpyHostToDevice
                ) );

    // Define flags for each AABB. These flags configure how OptiX handles each geometry during traversal.
    uint32_t aabb_input_flags[] = {
        /* flags for heliostat 1 */
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
        /* flags for heliostat 2 */
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
        /* flags for heliostat 3 */
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
        /* flag for receiver */
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
    };

    // Define shader binding table (SBT) indices for each geometry. TODO generalize
    const uint32_t sbt_index[] = { 0, 1, 2, 3};
    CUdeviceptr    d_sbt_index;

    // Allocate memory on the device for the SBT indices. Copy the SBT indices from the host to the device.
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_sbt_index ), sizeof(sbt_index) ) );
    CUDA_CHECK( cudaMemcpy(
        reinterpret_cast<void*>( d_sbt_index ),
        sbt_index,
        sizeof( sbt_index ),
        cudaMemcpyHostToDevice ) );

    // Configure the input for the GAS build process.
    OptixBuildInput aabb_input = {};
    aabb_input.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
    aabb_input.customPrimitiveArray.aabbBuffers   = &d_aabb;
    aabb_input.customPrimitiveArray.flags         = aabb_input_flags;
    aabb_input.customPrimitiveArray.numSbtRecords = OBJ_COUNT;
    aabb_input.customPrimitiveArray.numPrimitives = OBJ_COUNT;
    aabb_input.customPrimitiveArray.sbtIndexOffsetBuffer         = d_sbt_index;
    aabb_input.customPrimitiveArray.sbtIndexOffsetSizeInBytes    = sizeof( uint32_t );
    aabb_input.customPrimitiveArray.primitiveIndexOffset         = 0;

    // Set up acceleration structure (AS) build options.
    OptixAccelBuildOptions accel_options = {
        OPTIX_BUILD_FLAG_ALLOW_COMPACTION,  // buildFlags. Enable compaction to reduce memory usage.
        OPTIX_BUILD_OPERATION_BUILD         // operation. Build a new acceleration structure (not an update).
    };

    // Build the GAS using the defined AABBs and options.
    buildGas(
        state,             // Application state with OptiX context.
        accel_options,     // Build options.
        aabb_input,        // AABB input description.
        state.gas_handle,  // Output: traversable handle for the GAS.
        state.d_gas_output_buffer // Output: device buffer for the GAS.
    );

    CUDA_CHECK( cudaFree( (void*)d_aabb) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>(d_sbt_index) ) );
}

// Create OptiX modules for different components of the application.
// Modules correspond to different functionality, such as geometry handling, materials, and the sun.
void createModules( SoltraceState &state )
{
    // Options to control optimization and debugging settings.
    OptixModuleCompileOptions module_compile_options = {};
#if !defined( NDEBUG )
    module_compile_options.optLevel   = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
    module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
#endif

    // Create geometry module.
    {
        size_t inputSize = 0;   // Variable to store the size of the CUDA input source.
        const char* input = sutil::getInputData(
            nullptr,            // No additional input path.
            nullptr,            // No output file name.
            "parallelogram.cu", // Name of the CUDA file containing geometry logic.
            inputSize           // Output: Size of the input CUDA source code.
        );

        OPTIX_CHECK_LOG(optixModuleCreate(
            state.context,                       // OptiX context for the application.
            &module_compile_options,             // Module compilation options.
            &state.pipeline_compile_options,     // Pipeline-level compile options.
            input,                               // CUDA source code as input.
            inputSize,                           // Size of the CUDA source code.
            LOG, &LOG_SIZE,                      // Logs for diagnostic output.
            &state.geometry_module               // Output: Handle for the compiled module.
        ));
    }

    // Create shading/materials module.
    {
        size_t inputSize = 0;
        const char* input = sutil::getInputData(
            nullptr,
            nullptr,
            "materials.cu",
            inputSize
        );

        OPTIX_CHECK_LOG(optixModuleCreate(
            state.context,
            &module_compile_options,
            &state.pipeline_compile_options,
            input,
            inputSize,
            LOG, &LOG_SIZE,
            &state.shading_module
        ));
    }

    // Create the sun module.
    {
        size_t inputSize = 0;
        const char* input = sutil::getInputData(
            nullptr,
            nullptr,
            "sun.cu", 
            inputSize
        );

        OPTIX_CHECK_LOG(optixModuleCreate(
            state.context,
            &module_compile_options,
            &state.pipeline_compile_options,
            input,
            inputSize,
            LOG, &LOG_SIZE,
            &state.sun_module
        ));
    }
}

// Create program group for the sun's ray generation program.
static void createSunProgram( SoltraceState &state, std::vector<OptixProgramGroup> &program_groups )
{
    OptixProgramGroup           sun_prog_group;                 // Handle for the sun program group.
    OptixProgramGroupOptions    sun_prog_group_options = {};    // Options for the program group (none needed in this case).
    OptixProgramGroupDesc       sun_prog_group_desc = {};       // Descriptor to define the program group.
    
    // Specify the kind of program group (Ray Generation).
    sun_prog_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    // Link the ray generation program to the sun module and specify the function name.
    sun_prog_group_desc.raygen.module            = state.sun_module;
    sun_prog_group_desc.raygen.entryFunctionName = "__raygen__sun_source";

    // Create the program group
    OPTIX_CHECK_LOG( optixProgramGroupCreate(
        state.context,                 // OptiX context.
        &sun_prog_group_desc,          // Descriptor defining the program group.
        1,                             // Number of program groups to create (1 in this case).
        &sun_prog_group_options,       // Options for the program group.
        LOG, &LOG_SIZE,                // Logs to capture diagnostic information.
        &sun_prog_group                // Output: Handle for the created program group.
    ));

    program_groups.push_back(sun_prog_group);
    state.raygen_prog_group = sun_prog_group;
}

// Create program group for handling rays interacting with mirrors.
static void createMirrorProgram( SoltraceState &state, std::vector<OptixProgramGroup> &program_groups )
{
    OptixProgramGroup           radiance_mirror_prog_group;                 // Handle for the mirror program group.
    OptixProgramGroupOptions    radiance_mirror_prog_group_options = {};    // Options for the program group (none needed).
    OptixProgramGroupDesc       radiance_mirror_prog_group_desc = {};       // Descriptor for the program group.

    // Specify the kind of program group (Hit Group for handling intersections and shading).
    radiance_mirror_prog_group_desc.kind   = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    // Link the intersection shader (geometry handling) to the geometry module.
    radiance_mirror_prog_group_desc.hitgroup.moduleIS               = state.geometry_module;
    radiance_mirror_prog_group_desc.hitgroup.entryFunctionNameIS    = "__intersection__parallelogram";
    // Link the closest-hit shader (shading logic) to the shading module.
    radiance_mirror_prog_group_desc.hitgroup.moduleCH               = state.shading_module;
    radiance_mirror_prog_group_desc.hitgroup.entryFunctionNameCH    = "__closesthit__mirror";
    // No any-hit shader is used in this configuration (set to nullptr).
    radiance_mirror_prog_group_desc.hitgroup.moduleAH               = nullptr;
    radiance_mirror_prog_group_desc.hitgroup.entryFunctionNameAH    = nullptr;

    // Create the program group
    OPTIX_CHECK_LOG( optixProgramGroupCreate(
        state.context,
        &radiance_mirror_prog_group_desc,
        1,
        &radiance_mirror_prog_group_options,
        LOG, &LOG_SIZE,
        &radiance_mirror_prog_group ) );

    program_groups.push_back(radiance_mirror_prog_group);
    state.radiance_mirror_prog_group = radiance_mirror_prog_group;
}

// Create program group for handling rays interacting with the receiver.
static void createReceiverProgram( SoltraceState &state, std::vector<OptixProgramGroup> &program_groups )
{
    OptixProgramGroup           radiance_receiver_prog_group;               // Handle for the receiver program group.
    OptixProgramGroupOptions    radiance_receiver_prog_group_options = {};  // Options for the program group (none needed).
    OptixProgramGroupDesc       radiance_receiver_prog_group_desc = {};     // Descriptor for the program group.

    // Specify the kind of program group (Hit Group for handling intersections and shading).
    radiance_receiver_prog_group_desc.kind   = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    // Link the intersection shader (geometry handling) to the geometry module.
    radiance_receiver_prog_group_desc.hitgroup.moduleIS               = state.geometry_module;
    radiance_receiver_prog_group_desc.hitgroup.entryFunctionNameIS    = "__intersection__parallelogram";
    // Link the closest-hit shader (shading logic) to the shading module.
    radiance_receiver_prog_group_desc.hitgroup.moduleCH               = state.shading_module;
    radiance_receiver_prog_group_desc.hitgroup.entryFunctionNameCH    = "__closesthit__receiver";
    // No any-hit shader is used in this configuration (set to nullptr).
    radiance_receiver_prog_group_desc.hitgroup.moduleAH               = nullptr;
    radiance_receiver_prog_group_desc.hitgroup.entryFunctionNameAH    = nullptr;

    // Create the program group
    OPTIX_CHECK_LOG( optixProgramGroupCreate(
        state.context,
        &radiance_receiver_prog_group_desc,
        1,
        &radiance_receiver_prog_group_options,
        LOG, &LOG_SIZE,
        &radiance_receiver_prog_group ) );

    program_groups.push_back(radiance_receiver_prog_group);
    state.radiance_receiver_prog_group = radiance_receiver_prog_group;
}

// Create program group for handling rays that miss all geometry.
static void createMissProgram( SoltraceState &state, std::vector<OptixProgramGroup> &program_groups )
{
    OptixProgramGroup           radiance_miss_prog_group;       // Handle for the miss program group.
    OptixProgramGroupOptions    miss_prog_group_options = {};   // Options for the program group (none needed).
    OptixProgramGroupDesc       miss_prog_group_desc = {};      // Descriptor for the program group.
    
    // Specify the kind of program group (Miss Program for handling missed rays).
    miss_prog_group_desc.kind   = OPTIX_PROGRAM_GROUP_KIND_MISS;
    // Link the miss shader (background or environment shading) to the shading module.
    miss_prog_group_desc.miss.module             = state.shading_module;
    miss_prog_group_desc.miss.entryFunctionName  = "__miss__ms";

    // Create the program grou
    OPTIX_CHECK_LOG( optixProgramGroupCreate(
        state.context,
        &miss_prog_group_desc,
        1,
        &miss_prog_group_options,
        LOG, &LOG_SIZE,
        &radiance_miss_prog_group ) );

    program_groups.push_back(radiance_miss_prog_group);
    state.radiance_miss_prog_group = radiance_miss_prog_group;
}

// Create and configure the OptiX pipeline.
// The pipeline is a core component in OptiX that ties together all program groups, modules, 
// and other configurations needed for ray tracing execution.
void createPipeline( SoltraceState &state )
{
    std::vector<OptixProgramGroup> program_groups;

    // Configure the pipeline compile options.
    state.pipeline_compile_options = {
        false,                                                  // usesMotionBlur: Disable motion blur.
        OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS,          // traversableGraphFlags: Allow only a single GAS.
        2,    /* RadiancePRD uses 5 payloads */                 // numPayloadValues
        5,    /* Parallelogram intersection uses 5 attrs */     // numAttributeValues 
        OPTIX_EXCEPTION_FLAG_NONE,                              // exceptionFlags
        "params"                                                // pipelineLaunchParamsVariableName
    };

    // Prepare modules and program groups
    createModules( state );
    createSunProgram( state, program_groups );
    createMirrorProgram( state, program_groups );
    createReceiverProgram( state, program_groups );
    createMissProgram( state, program_groups );

    // Link program groups to pipeline
    OptixPipelineLinkOptions pipeline_link_options = {};
    pipeline_link_options.maxTraceDepth            = max_trace; // Maximum recursion depth for ray tracing.
    
    // Create the OptiX pipeline by linking the program groups.
    OPTIX_CHECK_LOG(optixPipelineCreate(
        state.context,                        // OptiX context.
        &state.pipeline_compile_options,      // Compile options for the pipeline.
        &pipeline_link_options,               // Link options for the pipeline.
        program_groups.data(),                // Array of program groups.
        static_cast<unsigned int>(program_groups.size()), // Number of program groups.
        LOG, &LOG_SIZE,                       // Logs for diagnostics.
        &state.pipeline                       // Output: Handle for the created pipeline.
    ));

    // Compute and configure the stack sizes for the pipeline.
    OptixStackSizes stack_sizes = {};
    for( auto& prog_group : program_groups )
    {
        OPTIX_CHECK( optixUtilAccumulateStackSizes( prog_group, &stack_sizes, state.pipeline ) );
    }

    uint32_t direct_callable_stack_size_from_traversal;
    uint32_t direct_callable_stack_size_from_state;
    uint32_t continuation_stack_size;

    // Compute stack sizes based on the maximum trace depth and other settings.
    OPTIX_CHECK(optixUtilComputeStackSizes(
        &stack_sizes,                      // Input stack sizes.
        max_trace,                         // Maximum trace depth.
        0,                                 // maxCCDepth: Maximum depth of continuation callables (none in this case).
        0,                                 // maxDCDepth: Maximum depth of direct callables (none in this case).
        &direct_callable_stack_size_from_traversal, // Output: Stack size for callable traversal.
        &direct_callable_stack_size_from_state,     // Output: Stack size for callable state.
        &continuation_stack_size                    // Output: Stack size for continuation stack.
    ));
    // Set the computed stack sizes for the pipeline.
    OPTIX_CHECK(optixPipelineSetStackSize(
        state.pipeline,                               // Pipeline to configure.
        direct_callable_stack_size_from_traversal,    // Stack size for direct callable traversal.
        direct_callable_stack_size_from_state,        // Stack size for direct callable state.
        continuation_stack_size,                      // Stack size for continuation stack.
        1                                            // maxTraversableDepth: Maximum depth of traversable hierarchy.
    ));
}

// Ccreate and configure the Shader Binding Table (SBT).
// The SBT is a crucial data structure in OptiX that links geometry and ray types
// with their corresponding programs (ray generation, miss, and hit group).
void createSBT( SoltraceState &state )
{
    // Ray generation program record
    {
        CUdeviceptr d_raygen_record;                   // Device pointer to hold the raygen SBT record.
        size_t      sizeof_raygen_record = sizeof( sutil::EmptyRecord );
        
        // Allocate memory for the raygen SBT record on the device.
        CUDA_CHECK( cudaMalloc(
            reinterpret_cast<void**>( &d_raygen_record ),
            sizeof_raygen_record ) );

        sutil::EmptyRecord rg_sbt;  // Host representation of the raygen SBT record.

        // Pack the program header into the raygen SBT record.
        optixSbtRecordPackHeader( state.raygen_prog_group, &rg_sbt );

        // Copy the raygen SBT record from host to device.
        CUDA_CHECK( cudaMemcpy(
            reinterpret_cast<void*>( d_raygen_record ),
            &rg_sbt,
            sizeof_raygen_record,
            cudaMemcpyHostToDevice
        ) );

        // Assign the device pointer to the raygenRecord field in the SBT.
        state.sbt.raygenRecord = d_raygen_record;
    }

    // Miss program record
    {
        CUdeviceptr d_miss_record;
        size_t sizeof_miss_record = sizeof( sutil::EmptyRecord );
        
        CUDA_CHECK( cudaMalloc(
            reinterpret_cast<void**>( &d_miss_record ),
            sizeof_miss_record*soltrace::RAY_TYPE_COUNT ) );

        sutil::EmptyRecord ms_sbt[soltrace::RAY_TYPE_COUNT];
        // Pack the program header into the first miss SBT record.
        optixSbtRecordPackHeader( state.radiance_miss_prog_group, &ms_sbt[0] );

        CUDA_CHECK( cudaMemcpy(
            reinterpret_cast<void*>( d_miss_record ),
            ms_sbt,
            sizeof_miss_record*soltrace::RAY_TYPE_COUNT,
            cudaMemcpyHostToDevice
        ) );

        // Configure the SBT miss program fields.
        state.sbt.missRecordBase          = d_miss_record;                   // Base address of the miss records.
        state.sbt.missRecordCount         = soltrace::RAY_TYPE_COUNT;        // Number of miss records.
        state.sbt.missRecordStrideInBytes = static_cast<uint32_t>( sizeof_miss_record );    // Stride between miss records.
    }

    // Hitgroup program record
    {
        // Total number of hitgroup records : one per ray type per object.
        const size_t count_records = soltrace::RAY_TYPE_COUNT * OBJ_COUNT;
        HitGroupRecord hitgroup_records[count_records];

        // Note: Fill SBT record array the same order that acceleration structure is built.
        int sbt_idx = 0; // Index to track current record.

        // TODO: Material params - arbitrary right now
         
        // Configure Heliostat 1 SBT record.
        OPTIX_CHECK( optixSbtRecordPackHeader(
            state.radiance_mirror_prog_group,
            &hitgroup_records[sbt_idx] ) );
        hitgroup_records[ sbt_idx ].data.geometry_data.setParallelogram( heliostat1 );
        hitgroup_records[ sbt_idx ].data.material_data.mirror = {
            0.95f, // Reflectivity.
            0.0f,  // Transmissivity.
            0.0f,  // Slope error.
            0.0f   // Specularity error.
        };
        sbt_idx++;

        // Configure Heliostat 2 SBT record.
        OPTIX_CHECK(optixSbtRecordPackHeader(
            state.radiance_mirror_prog_group,
            &hitgroup_records[sbt_idx]));
        hitgroup_records[sbt_idx].data.geometry_data.setParallelogram(heliostat2);
        hitgroup_records[sbt_idx].data.material_data.mirror = {
            0.95f, // Reflectivity.
            0.0f,  // Transmissivity.
            0.0f,  // Slope error.
            0.0f   // Specularity error.
        };
        sbt_idx++;

        // Configure Heliostat 3 SBT record.
        OPTIX_CHECK(optixSbtRecordPackHeader(
            state.radiance_mirror_prog_group,
            &hitgroup_records[sbt_idx]));
        hitgroup_records[sbt_idx].data.geometry_data.setParallelogram(heliostat3);
        hitgroup_records[sbt_idx].data.material_data.mirror = {
            0.95f, // Reflectivity.
            0.0f,  // Transmissivity.
            0.0f,  // Slope error.
            0.0f   // Specularity error.
        };
        sbt_idx++;

        // Configure Receiver SBT record.
        OPTIX_CHECK( optixSbtRecordPackHeader(
            state.radiance_receiver_prog_group,
            &hitgroup_records[sbt_idx] ) );
        hitgroup_records[ sbt_idx ].data.geometry_data.setParallelogram( receiver );
        hitgroup_records[ sbt_idx ].data.material_data.receiver = {
            0.95f, // Reflectivity.
            0.0f,  // Transmissivity.
            0.0f,  // Slope error.
            0.0f   // Specularity error.
        };
        sbt_idx++;

        // Allocate memory for hitgroup records on the device.
        CUdeviceptr d_hitgroup_records;
        size_t      sizeof_hitgroup_record = sizeof( HitGroupRecord );
        CUDA_CHECK( cudaMalloc(
            reinterpret_cast<void**>( &d_hitgroup_records ),
            sizeof_hitgroup_record*count_records
        ) );

        // Copy hitgroup records from host to device.
        CUDA_CHECK( cudaMemcpy(
            reinterpret_cast<void*>( d_hitgroup_records ),
            hitgroup_records,
            sizeof_hitgroup_record*count_records,
            cudaMemcpyHostToDevice
        ) );

        // Configure the SBT hitgroup fields.
        state.sbt.hitgroupRecordBase            = d_hitgroup_records;             // Base address of hitgroup records.
        state.sbt.hitgroupRecordCount           = count_records;                  // Total number of hitgroup records.
        state.sbt.hitgroupRecordStrideInBytes   = static_cast<uint32_t>( sizeof_hitgroup_record );  // Stride size.
    }
}

// Callback function for logging messages from the OptiX context.
static void context_log_cb(unsigned int level, const char* tag, const char* message, void* /*cbdata */)
{
    // Format and print the log message to the standard error stream.
    // The message includes:
    // - The log level (e.g., error, warning, info).
    // - A tag for categorization (e.g., "API", "Internal").
    // - The actual message content.
    std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: "
        << message << "\n";
}

// Create and initialize an OptiX context.
void createContext( SoltraceState& state )
{
    // Initialize CUDA
    CUDA_CHECK( cudaFree( 0 ) );

    OptixDeviceContext context;
    CUcontext          cuCtx = 0;  // Use the current CUDA context (zero indicates current).

    // Set OptiX device context options.
    OPTIX_CHECK( optixInit() );
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction       = &context_log_cb;    // Optional: Set a logging callback function for debugging.
    options.logCallbackLevel          = 4;                  // Verbosity level for logging (e.g., errors, warnings, etc.).
    
    // Create and store OptiX device context
    OPTIX_CHECK( optixDeviceContextCreate( cuCtx, &options, &context ) );
    state.context = context;
}

// Initialize the launch parameters used for ray tracing.
void initLaunchParams( SoltraceState& state )
{
    // Set maximum ray depth.
    state.params.max_depth = max_trace;

    // Allocate memory for the hit point buffer.
    // The size depends on the sun (ray generation resolution) parameters and the maximum ray depth.
    const size_t hit_point_buffer_size = state.params.width * state.params.height * sizeof(float4) * state.params.max_depth;
    
    CUDA_CHECK(cudaMalloc(
        reinterpret_cast<void**>(&state.params.hit_point_buffer),
        hit_point_buffer_size
    ));
    CUDA_CHECK(cudaMemset(state.params.hit_point_buffer, 0, hit_point_buffer_size));

    // Reflected direction buffer (commented out for memory saving).
    /*
    const size_t reflected_dir_buffer_size = state.params.width * state.params.height * sizeof(float4) * state.params.max_depth;
    CUDA_CHECK(cudaMalloc(
        reinterpret_cast<void**>(&state.params.reflected_dir_buffer),
        reflected_dir_buffer_size
    ));
    CUDA_CHECK(cudaMemset(state.params.reflected_dir_buffer, 0, reflected_dir_buffer_size));
    */ 

    // Create a CUDA stream for asynchronous operations.
    CUDA_CHECK( cudaStreamCreate( &state.stream ) );

    // Allocate memory for device-side launch parameters.
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &state.d_params ), sizeof( soltrace::LaunchParams ) ) );

    // Link the GAS handle.
    state.params.handle = state.gas_handle;
}

// Clean up resources and deallocate memory
void cleanupState( SoltraceState& state )
{
    OPTIX_CHECK( optixPipelineDestroy     ( state.pipeline                ) );
    OPTIX_CHECK( optixProgramGroupDestroy ( state.raygen_prog_group       ) );
    OPTIX_CHECK( optixProgramGroupDestroy ( state.radiance_mirror_prog_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy ( state.radiance_receiver_prog_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy ( state.radiance_miss_prog_group         ) );
    OPTIX_CHECK( optixModuleDestroy       ( state.shading_module          ) );
    OPTIX_CHECK( optixModuleDestroy       ( state.geometry_module         ) );
    OPTIX_CHECK( optixDeviceContextDestroy( state.context                 ) );


    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.sbt.raygenRecord       ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.sbt.missRecordBase     ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.sbt.hitgroupRecordBase ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.d_gas_output_buffer    ) ) );
    CUDA_CHECK(cudaFree(reinterpret_cast<void*>(state.params.hit_point_buffer)));
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.d_params               ) ) );
}

// Check if components of a float4 are all zero. 
// If they are, it returns TRUE; otherwise, it returns FALSE.
// TODO: Move to util file
bool allZeros(float4 element) {
    return (element.y == 0 && element.z == 0 && element.w == 0);
}

// Write a vector of float4 data to a CSV file, filtering out rows based on allZeros.
void writeVectorToCSV(const std::string& filename, const std::vector<float4>& data) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open the file " << filename << " for writing." << std::endl;
        return;
    }

    // Write header
    outFile << "number,stage,loc_x,loc_y,loc_z\n";

    // Write data
    int ray_idx = 1;
    int idx = 0;
    for (const auto& element : data) {
        ++idx;
        if (idx <= max_trace && !allZeros(element)) {
            outFile << ray_idx << "," << element.x << "," << element.y << "," << element.z << "," << element.w << "\n";
        }
        else {
            idx = 0;
            ++ray_idx;
        }
    }

    outFile.close();
    std::cout << "Data successfully written to " << filename << std::endl;
}

int main(int argc, char* argv[])
{
    SoltraceState state;

    try
    {
        // Initialize simulation parameters
        state.params.sun_center = make_float3(0.0f, 0.0f, 20.0f);
        state.params.max_sun_angle = 0.00465;     // 4.65 mrad
        state.params.num_sun_points = 1000000;

        state.params.width  = state.params.num_sun_points;
        state.params.height = 1;

        // Initialize OptiX components
        createContext(state);
        createGeometry(state);
        createPipeline(state);
        createSBT(state);
        initLaunchParams(state);

        // Copy launch parameters to device memory
        CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(state.d_params), &state.params,
            sizeof(soltrace::LaunchParams), cudaMemcpyHostToDevice, state.stream));

        // Launch the OptiX pipeline
        OPTIX_CHECK( optixLaunch(
        state.pipeline,     // OptiX pipeline
        state.stream,       // CUDA stream used for this launch
        reinterpret_cast<CUdeviceptr>( state.d_params ),    // Device pointer to the launch parameters structure.
        sizeof( soltrace::LaunchParams ),                   // Size of launch parameters structure
        &state.sbt,          // Shader Binding Table(SBT): contains the pointers to program groups and associated data.
        state.params.width,  // Number of threads to launch along the X dimension.
        state.params.height, // Number of threads to launch along the Y dimension.
        1                    // Number of threads to launch along the Z dimension. Often set to 1 for 2D images or other non-volumetric workloads.
        ) );

        CUDA_SYNC_CHECK();

        // Copy hit point results from device memory
        std::vector<float4> hp_output_buffer(state.params.width * state.params.height * state.params.max_depth);
        CUDA_CHECK(cudaMemcpy(hp_output_buffer.data(), state.params.hit_point_buffer, state.params.width * state.params.height * state.params.max_depth * sizeof(float4), cudaMemcpyDeviceToHost));

        // Copy reflected direction results from device memory
        /*
        std::vector<float4> rd_output_buffer(state.params.width * state.params.height * state.params.max_depth);
        CUDA_CHECK(cudaMemcpy(rd_output_buffer.data(), state.params.reflected_dir_buffer, state.params.width * state.params.height * state.params.max_depth * sizeof(float4), cudaMemcpyDeviceToHost));
        */

        writeVectorToCSV("test_output_new_sun_model_v13.csv", hp_output_buffer);

        cleanupState(state);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Caught exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}