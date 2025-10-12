#include "core/soltrace_system.h"
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <filesystem>

// three heliostat
// two "receivers" made of different transmissivity 
// all flat surfaces and rectangular apertures


using namespace std;
using namespace OptixCSP;

enum RECEIVER_TYPE { FLAT, CYLINDRICAL, TRIANGLE };

int main(int argc, char* argv[]) {
    int num_rays = 1000;

    if (argc == 2) {
        num_rays = atoi(argv[1]);
    }

    // Create the simulation system.
    SolTraceSystem system(num_rays);

    Vec3d origin_e1(-5, 0, 0); // origin of the element
    Vec3d aim_point_e1(17.360680, 0, 94.721360); // aim point of the element
    auto e1 = std::make_shared<CspElement>();
    e1->set_origin(origin_e1);
    e1->set_aim_point(aim_point_e1); // Aim direction
    e1->set_zrot(-90.0); // Set the rotation around the Z-axis

    // Create a flat surface if not parabolic
    auto surface_e1 = std::make_shared<SurfaceFlat>();
    e1->set_surface(surface_e1);


    double dim_x = 1.0;
    double dim_y = 1.95;
    auto aperture_e1 = std::make_shared<ApertureRectangle>(dim_x, dim_y);
    e1->set_aperture(aperture_e1);

    //system.add_element(e1);

    //// Element 2
    Vec3d origin_e2(0, 5, 0); // origin of the element
    Vec3d aim_point_e2(0, -17.360680, 94.721360); // aim point of the element
    auto e2 = std::make_shared<CspElement>();
    e2->set_origin(origin_e2);
    e2->set_aim_point(aim_point_e2); // Aim direction
    e2->set_zrot(0.0); // No rotation for the element

    // Create a flat surface if not parabolic
    auto surface_e2 = std::make_shared<SurfaceFlat>();
    e2->set_surface(surface_e2);

    auto aperture_e2 = std::make_shared<ApertureRectangle>(dim_x, dim_y);
    e2->set_aperture(aperture_e2);

    system.add_element(e2);

    //// Element 3
    Vec3d origin_e3(5, 0, 0); // origin of the element
    Vec3d aim_point_e3(-17.360680, 0, 94.721360); // aim point of the element
    auto e3 = std::make_shared<CspElement>();
    e3->set_origin(origin_e3);
    e3->set_aim_point(aim_point_e3); // Aim direction
    e3->set_zrot(90.0);

    // Create a flat surface if not parabolic
    auto surface_e3 = std::make_shared<SurfaceFlat>();
    e3->set_surface(surface_e3);

    auto aperture_e3 = std::make_shared<ApertureRectangle>(dim_x, dim_y);
    e3->set_aperture(aperture_e3);

    //system.add_element(e3);

    //////////////////////////////////////////////
    // STEP 2.1 Create receiver, flat rectangle //
    //////////////////////////////////////////////
    Vec3d receiver_1_origin(0, 0, 9.5); // origin of the receiver
    Vec3d receiver_1_aim_point(0, 5, 0); // aim point of the receiver

    auto e4 = std::make_shared<CspElement>();
    e4->set_origin(receiver_1_origin);
    e4->set_aim_point(receiver_1_aim_point); // Aim direction
    e4->set_zrot(0.0); // No rotation for the receiver

    double receiver_dim_x;
    double receiver_dim_y;

    std::cout << "Using flat receiver" << std::endl;
    receiver_dim_x = 2.0; // width of the receiver
    receiver_dim_y = 2.0; // height of the receiver

    e4->set_aperture(std::make_shared<ApertureRectangle>(receiver_dim_x, receiver_dim_y));
    // Create a flat surface if not using cylindrical
    auto receiver_surface = std::make_shared<SurfaceFlat>();
    e4->set_surface(receiver_surface);
    e4->set_receiver(true); // Mark this element as a receiver
    system.add_element(e4); // Add the receiver to the system

    // create another element e5 same as e4
    CspElement e5 = *e4; // Copy e4 to e5
    e5.set_origin(Vec3d(0, 1, 7.5));
    system.add_element(std::make_shared<CspElement>(e5)); // Add e5 to the system
    e5.set_receiver(false);

    Vec3d sun_vector(0.0, 0.0, 100.0); // sun vector

    // set up sun angle 
    //double sun_angle = 0.00465; // 0.00465; // sun angle
    //double sun_angle = 0; // 0.00465; // sun angle

    //system.set_sun_angle(sun_angle);
    system.set_sun_vector(sun_vector);

    ///////////////////////////////////
    // STEP 3  Initialize the system //
    ///////////////////////////////////
    system.initialize();

    ////////////////////////////
    // STEP 4  Run Ray Trace //
    ///////////////////////////
    // TODO: set up different sun position trace // 
    system.run();

    //////////////////////////
    // STEP 5  Post process //
    //////////////////////////
    int num_hits = system.get_num_hits_receiver();
    std::cout << "Number of rays hitting the receiver: " << num_hits << std::endl;

    std::string out_dir = "out_transmissivity/";
    if (!std::filesystem::exists(std::filesystem::path(out_dir))) {
        std::cout << "Creating output directory: " << out_dir << std::endl;
        if (!std::filesystem::create_directory(std::filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

    }

    system.write_hp_output(out_dir + "sun_error_hit_points_" + to_string(num_rays) + "_rays.csv");
    system.write_simulation_json(out_dir + "sun_error_summary_" + to_string(num_rays) + "_rays.json");

    /////////////////////////////////////////
    // STEP 6  Be a good citizen, clean up //
    /////////////////////////////////////////
    system.clean_up();



    return 0;
}