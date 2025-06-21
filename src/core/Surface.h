#ifndef SOLTRACE_SURFACE_H
#define SOLTRACE_SURFACE_H

#include "soltrace_type.h"    // For SurfaceType enum

namespace OptixCSP
{


    /**
     * @class Surface
     * @brief Define surface of an element
     *
     * Base class for different types of surfaces (flat, parabolic, spherical, etc.).
     */
    class Surface {
    public:
        Surface() = default;
        virtual ~Surface() = default;

        // Returns the type of this surface (flat, parabolic, spherical, etc.)
        virtual SurfaceType get_surface_type() const = 0;

        // Returns the curvature parameters for the surface (if applicable) 
        virtual double get_curvature_1() const { return 0.0; }


        virtual double get_curvature_2() const { return 0.0; }
    };



    /**
     * @class SurfaceParabolic
     * @brief Define parabolic surface of an element
     *
     * Parabolic surface defined by two curvature parameters (c1 and c2)
     * local coordinate origin is at the origin of the element
     */
    class SurfaceParabolic : public Surface {
    public:
        SurfaceParabolic() : m_c1(0.0), m_c2(0.0) {}
        SurfaceParabolic(double c1, double c2) : m_c1(c1), m_c2(c2) {}
        ~SurfaceParabolic() = default;

        SurfaceType get_surface_type() const override {
            return SurfaceType::PARABOLIC;
        }

        void set_curvature(double c1, double c2) {
            m_c1 = c1;
            m_c2 = c2;
        }

        virtual double get_curvature_1() const override {
            return m_c1;
        }

        virtual double get_curvature_2() const override {
            return m_c2;
        }

    private:
        double m_c1;
        double m_c2;
    };


    /// flat surface 
    class SurfaceFlat : public Surface {
    public:
        SurfaceFlat() = default;
        virtual ~SurfaceFlat() = default;

        virtual SurfaceType get_surface_type() const override {
            return SurfaceType::FLAT;
        }

    };

    /// cylindrical surface (with caps)
    class SurfaceCylinder : public Surface {
    public:
        SurfaceCylinder() : m_radius(1.0), m_half_height(1.0) {} // Initialize member variables
        virtual ~SurfaceCylinder() = default;

        SurfaceType get_surface_type() const override {
            return SurfaceType::CYLINDER;
        }

        void set_radius(double radius) {
            m_radius = radius;
        }

        double get_radius() const {
            return m_radius;
        }

        void set_half_height(double half_height) {
            m_half_height = half_height;
        }

        double get_half_height() const {
            return m_half_height;
        }

    private:
        double m_radius{ 1.0 };       // Default initialization
        double m_half_height{ 1.0 }; // Default initialization
    };

}
#endif // SOLTRACE_SURFACE_H
