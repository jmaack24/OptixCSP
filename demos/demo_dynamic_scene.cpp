#include "soltrace_system.h"
#include "lib/element.h"
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <optix_function_table_definition.h>
#include <timer.h>


int main(int argc, char* argv[]) {

	// parabolic mirror, cylindrical receiver
	// number of rays launched for the simulation
	int num_rays = 1000;
	// Create the simulation system.
	SolTraceSystem system(num_rays);

	double curv_x = 0.0170679f;
	double curv_y = 0.0370679f;
	double dim_x = 1.0;
	double dim_y = 1.95;


	// Element 3
	Vector3d origin_e2(0, 5, 0); // origin of the element
	Vector3d aim_point_e2(0, -17.360680, 94.721360); // aim point of the element
	// z of aimpoint can go all the way to 100 
	//Vector3d aim_point_e3(0, 0, 9.5);

	auto e2 = std::make_shared<Element>();
	e2->set_origin(origin_e2);
	e2->set_aim_point(aim_point_e2); // Aim direction
	e2->set_zrot(0.0);

	auto surface_e2 = std::make_shared<SurfaceParabolic>();
	//auto surface_e3 = std::make_shared<SurfaceFlat>();

	surface_e2->set_curvature(curv_x, curv_y);
	e2->set_surface(surface_e2);

	auto aperture_e3 = std::make_shared<ApertureRectangle>(dim_x, dim_y);
	e2->set_aperture(aperture_e3);

	system.add_element(e2);

	//////////////////////////////////////////////
	// STEP 2.1 Create receiver, flat rectangle //
	//////////////////////////////////////////////
	Vector3d receiver_origin(0, 0, 10.0); // origin of the receiver
	Vector3d receiver_aim_point(0, 5, 0.0); // aim point of the receiver

	auto e4 = std::make_shared<Element>();
	e4->set_origin(receiver_origin);
	e4->set_aim_point(receiver_aim_point); // Aim direction
	e4->set_zrot(0.0); // No rotation for the receiver


	///////////////////////////////////////////
	// STEP 2.2 create rectangle aperture    //
	///////////////////////////////////////////
	double receiver_dim_x = 2.0;  // diameter of the receiver
	double receiver_dim_y = 4.0;  // full height of the cylindrical receiver

	auto receiver_aperture = std::make_shared<ApertureRectangle>(receiver_dim_x, receiver_dim_y);
	e4->set_aperture(receiver_aperture);

	// Create a cylindrical surface if use_cylindical is true
	auto receiver_surface = std::make_shared<SurfaceFlat>();
	e4->set_surface(receiver_surface);
	system.add_element(e4); // Add the receiver to the system

	// set up sun vector and angle 
	Vector3d sun_vector(0.0, -20.0, 100.0); // sun vector
	//double sun_angle = 0.00465; // 0.00465; // sun angle
	double sun_angle = 0.0; // 0.00465; // sun angle, set to 0 for this example	

	system.set_sun_vector(sun_vector);
	system.set_sun_angle(sun_angle);

	system.initialize();

	int end_frames = 40;
	for (int frame = 0; frame < end_frames; frame++) {
		system.run();
		std::string filename = "output_dynamic_" + std::to_string(frame) + ".csv";
		system.write_output(filename);

		// update strategy, can either be sun vector or pose/position of the heliostats
		//aim_point_e3[2] += 1;
		//e3->update_element(aim_point_e3, zrot_e3);

		 //update sun position
		sun_vector[1] += 1; // sun goes around
		system.set_sun_vector(sun_vector);

		system.update();
	}

	system.clean_up();
	return 0;
}