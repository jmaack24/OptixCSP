#include <cstdint>
#include <string>

#include "vector3d.h"
#include "soltrace_type.h"
#include "surface.h"
#include "aperture.h"
#include "math_util.h"
#include "GeometryDataST.h"
#include "element.h"
#include "vector"

ElementBase::ElementBase() {}

Element::Element() {
    m_origin = Vector3d(0.0, 0.0, 0.0);
    m_aim_point = Vector3d(0.0, 0.0, 1.0); // Default aim direction
    m_euler_angles = Vector3d(0.0, 0.0, 0.0); // Default orientation
    m_zrot = 0.0;
    m_surface = nullptr;
    m_aperture = nullptr;
}

// set and get origin 
const Vector3d& Element::get_origin() const{
    return m_origin;
}

void Element::set_origin(const Vector3d& o) {
    m_origin = o;
}

void Element::set_aim_point(const Vector3d& a) {
    m_aim_point = a;
}

const Vector3d& Element::get_aim_point() const {
    return m_aim_point;
}

void Element::set_zrot(double zrot) {
    m_zrot = zrot;
}

double Element::get_zrot() const {
    return m_zrot;
}


std::shared_ptr<Aperture> Element::get_aperture() const {
    return m_aperture;
}

std::shared_ptr<Surface> Element::get_surface() const {
    return m_surface;
}

ApertureType Element::get_aperture_type() const {
    return m_aperture->get_aperture_type();
}

SurfaceType Element::get_surface_type() const {
    return m_surface->get_surface_type();
}

// Optical elements setters.
void Element::set_aperture(const std::shared_ptr<Aperture>& aperture)
{
    m_aperture = aperture;
}
void Element::set_surface(const std::shared_ptr<Surface>& surface)
{
    m_surface = surface;
}

// set orientation based on aimpoint and zrot
void Element::update_euler_angles(const Vector3d& aim_point, const double zrot) {
    Vector3d normal = aim_point - m_origin;
    normal.normalized();
    m_euler_angles = mathUtil::normal_to_euler(normal, zrot);
}

void Element::update_euler_angles() {
    Vector3d normal = m_aim_point - m_origin;
    normal.normalized();
    m_euler_angles = mathUtil::normal_to_euler(normal, m_zrot);
}

void Element::update_element(const Vector3d& aim_point, const double zrot) {
    m_aim_point = aim_point;
    m_zrot = zrot;
    update_euler_angles();
}
// return L2G rotation matrix
Matrix33d Element::get_rotation_matrix() const {
    // get G2L rotation matrix from euler angles 
    // TODO: need to think about if we store this or not
    Matrix33d mat_G2L = mathUtil::get_rotation_matrix_G2L(m_euler_angles);
    return mat_G2L.transpose();
}


// return upper bounding box
Vector3d Element::get_upper_bounding_box() const {
    return m_upper_box_bound;
}

// return lower bounding box
Vector3d Element::get_lower_bounding_box() const {
    return m_lower_box_bound;
}


GeometryDataST Element::toDeviceGeometryData() const {

    GeometryDataST geometry_data;

    SurfaceType surface_type = m_surface->get_surface_type();
    ApertureType aperture_type = m_aperture->get_aperture_type();

    if (aperture_type == ApertureType::RECTANGLE) {

        double width = m_aperture->get_width();
        double height = m_aperture->get_height();

        Matrix33d rotation_matrix = get_rotation_matrix();  // L2G rotation matrix

        Vector3d v1 = rotation_matrix.get_x_basis();
        Vector3d v2 = rotation_matrix.get_y_basis();

        if (surface_type == SurfaceType::FLAT) {
            GeometryDataST::Rectangle_Flat heliostat(mathUtil::toFloat3(m_origin), mathUtil::toFloat3(v1), mathUtil::toFloat3(v2), (float)width, (float)height);
            geometry_data.setRectangle_Flat(heliostat);
        }

        if (surface_type == SurfaceType::PARABOLIC) {
			v1 = v1 * (float)(-width);
			v2 = v2 * (float)height;
			float3 anchor = mathUtil::toFloat3(m_origin - v1 * 0.5 - v2 * 0.5);
            GeometryDataST::Rectangle_Parabolic heliostat(mathUtil::toFloat3(v1), mathUtil::toFloat3(v2),  anchor,
                (float)m_surface->get_curvature_1(),
                (float)m_surface->get_curvature_2());
            geometry_data.setRectangleParabolic(heliostat);
        }

		if (surface_type == SurfaceType::CYLINDER) {
            float radius = static_cast<float>(width) / 2.0f;
            float half_height = static_cast<float>(height) / 2.0f;

			float3 center = mathUtil::toFloat3(m_origin);
			Matrix33d rotation_matrix = get_rotation_matrix();  // L2G rotation matrix

			float3 base_x = mathUtil::toFloat3(rotation_matrix.get_x_basis());

			float3 base_z = mathUtil::toFloat3(rotation_matrix.get_z_basis());

			GeometryDataST::Cylinder_Y heliostat(center, radius, half_height, base_x, base_z);

			geometry_data.setCylinder_Y(heliostat);
		}
    }

    return geometry_data;
}


// we also need to implement the bounding box computation
// for a case like a rectangle aperture,
// once we have the origin, euler angles, rotatioin matrix
// and the aperture size, we can compute the bounding box
// this can be called when adding an element to the system
void Element::compute_bounding_box() {
    // this can also be called while "initializing" the element
    // get the rotation matrix first
    Matrix33d rotation_matrix = get_rotation_matrix();  // L2G rotation matrix

    // now check the type of the aperture
    ApertureType aperture_type = m_aperture->get_aperture_type();
	SurfaceType surface_type = m_surface->get_surface_type();

    if (aperture_type == ApertureType::RECTANGLE && surface_type != SurfaceType::CYLINDER) {
        // get the width and height of the aperture
        double width = m_aperture->get_width();
        double height = m_aperture->get_height();

        // compute the four corners of the rectangle locally
        Vector3d corner1 = Vector3d(-width / 2, -height / 2, 0.0);
        Vector3d corner2 = Vector3d( width / 2, -height / 2, 0.0);
        Vector3d corner3 = Vector3d( width / 2,  height / 2, 0.0);
        Vector3d corner4 = Vector3d(-width / 2,  height / 2, 0.0);

        // transform the corners to the global frame
        Vector3d corner1_global = rotation_matrix * corner1 + m_origin;
        Vector3d corner2_global = rotation_matrix * corner2 + m_origin;
        Vector3d corner3_global = rotation_matrix * corner3 + m_origin;
        Vector3d corner4_global = rotation_matrix * corner4 + m_origin;

        // now update the bounding box, need to find the min and max x, y, z
        m_lower_box_bound[0] = fmin(fmin(corner1_global[0], corner2_global[0]), fmin(corner3_global[0], corner4_global[0]));
        m_lower_box_bound[1] = fmin(fmin(corner1_global[1], corner2_global[1]), fmin(corner3_global[1], corner4_global[1]));
        m_lower_box_bound[2] = fmin(fmin(corner1_global[2], corner2_global[2]), fmin(corner3_global[2], corner4_global[2]));

        m_upper_box_bound[0] = fmax(fmax(corner1_global[0], corner2_global[0]), fmax(corner3_global[0], corner4_global[0]));
        m_upper_box_bound[1] = fmax(fmax(corner1_global[1], corner2_global[1]), fmax(corner3_global[1], corner4_global[1]));
        m_upper_box_bound[2] = fmax(fmax(corner1_global[2], corner2_global[2]), fmax(corner3_global[2], corner4_global[2]));
    }

    // slightly different for the cylinder, we want to know the radius and half height
	if (surface_type == SurfaceType::CYLINDER) {
		// get the radius and full height of the cylinder
        double width  = m_aperture->get_width();
        double height = m_aperture->get_height();

        // compute 8 corners of the cyliinder box locally
		Vector3d corner1 = Vector3d(-width / 2, -height / 2, -width / 2);
		Vector3d corner2 = Vector3d( width / 2, -height / 2, -width / 2);
		Vector3d corner3 = Vector3d(-width / 2,  height / 2, -width / 2);
		Vector3d corner4 = Vector3d( width / 2,  height / 2, -width / 2);
        Vector3d corner5 = Vector3d(-width / 2, -height / 2,  width / 2);
        Vector3d corner6 = Vector3d( width / 2, -height / 2,  width / 2);
        Vector3d corner7 = Vector3d(-width / 2,  height / 2,  width / 2);
        Vector3d corner8 = Vector3d( width / 2,  height / 2,  width / 2);

		// get the rotation matrix
		Matrix33d rotation_matrix = get_rotation_matrix();  // L2G rotation matrix

		// transform the corners to the global frame
		Vector3d corner1_global = rotation_matrix * corner1 + m_origin;
		Vector3d corner2_global = rotation_matrix * corner2 + m_origin;
		Vector3d corner3_global = rotation_matrix * corner3 + m_origin;
		Vector3d corner4_global = rotation_matrix * corner4 + m_origin;
		Vector3d corner5_global = rotation_matrix * corner5 + m_origin;
		Vector3d corner6_global = rotation_matrix * corner6 + m_origin;
		Vector3d corner7_global = rotation_matrix * corner7 + m_origin;
		Vector3d corner8_global = rotation_matrix * corner8 + m_origin;

		// go through the corners and find the min and max x, y, z
		std::vector<Vector3d> corners = { corner1_global, corner2_global, corner3_global, corner4_global,
					corner5_global, corner6_global, corner7_global, corner8_global };

		double min_x = std::numeric_limits<double>::max();
		double min_y = std::numeric_limits<double>::max();
		double min_z = std::numeric_limits<double>::max();

		double max_x = std::numeric_limits<double>::lowest();
		double max_y = std::numeric_limits<double>::lowest();
		double max_z = std::numeric_limits<double>::lowest();

		for (auto& corner : corners) {
			min_x = fmin(min_x, corner[0]);
			min_y = fmin(min_y, corner[1]);
			min_z = fmin(min_z, corner[2]);

			max_x = fmax(max_x, corner[0]);
			max_y = fmax(max_y, corner[1]);
			max_z = fmax(max_z, corner[2]);
		}

		// set the lower and upper bounds
		m_lower_box_bound[0] = min_x;
		m_lower_box_bound[1] = min_y;
		m_lower_box_bound[2] = min_z;

		m_upper_box_bound[0] = max_x;
		m_upper_box_bound[1] = max_y;
		m_upper_box_bound[2] = max_z;
	}







}