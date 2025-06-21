#ifndef GEOMETRYMANAGER_H
#define GEOMETRYMANAGER_H


#include <string>
#include <vector>
#include <cuda_runtime.h>
#include <optix.h>

#include "shaders/Soltrace.h"
#include "CspElement.h"
#include "soltrace_state.h"

namespace OptixCSP {

	class dataManager;
	/**
	 * @class geometryManager
	 * @brief Given the geoemtry of the elements, populate the list of aabb,
	 * compute the sun plane, and build the GAS (Geometry Acceleration Structure) for ray tracing.
	 */
	class GeometryManager {
	public:
		GeometryManager(SoltraceState& state) : m_state(state), m_obj_counts(0) {}
		~GeometryManager() {}

		/// go through the list of elements and collect the geometry info on the host: 
		/// - AABBs
		/// - GeometryDataST on the host
		/// - SBT index
		void collect_geometry_info(const std::vector<std::shared_ptr<CspElement>>& element_list,
			LaunchParams& params);

		/// build the GAS (Geometry Acceleration Structure) using the AABB list, populate optix state
		void create_geometries(LaunchParams& params);


		/// update the GAS (Geometry Acceleration Structure) using the AABB list, populate optix state
		void update_geometry_info(const std::vector<std::shared_ptr<CspElement>>& element_list,
			LaunchParams& params);

		/// return the list of geometry data vector
		std::vector<GeometryDataST>& get_geometry_data_array() { return m_geometry_data_array_H; }

		// compute sun plane 
		void compute_sun_plane_H(LaunchParams& params);


	private:
		SoltraceState& m_state;
		float m_sun_plane_distance = -1.0f; // distance of the sun plane from the origin
		uint32_t m_obj_counts;

		// data related to the geometry and the scene on the host side
		std::vector<OptixAabb>      m_aabb_list_H;           // aabb list
		std::vector<GeometryDataST> m_geometry_data_array_H; // geometry data
		std::vector<uint32_t>       m_sbt_index_H;           // sbt offset index

		// members related to building GAS
		OptixBuildInput        m_aabb_input = {};                   // needed after the first build
		OptixAccelBuildOptions m_accel_build_options = {};  // needed after the first build
		CUdeviceptr            m_aabb_list_D{};          // device pointer to the aabb list


		CUdeviceptr m_output_buffer{};   // output buffer
		CUdeviceptr m_temp_buffer{};     // temporary buffer for building GAS
		size_t m_output_buffer_size = 0;   // size of that scratch
		size_t m_temp_buffer_size = 0;     // size of the output buffer
	};
}
#endif // GEOMETRY_MANAGER_H
