#pragma once

#include <cuda_runtime.h>

namespace OptixCSP
{
    struct MaterialData
    {
        float reflectivity;
        float transmissivity;
        float slope_error;
        float specularity_error;
		bool  use_refraction;  // todo: for now, the ray goes through the object if true, otherwise it reflects
    };
}   