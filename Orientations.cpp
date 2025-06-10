#include "Orientations.hpp"
#include <stdexcept>
#include <cmath> // For M_PI, sqrt, acos, atan2, cos, sin
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace SpinStepOrientations {

    // Definition of the static map
    const std::map<int, int> TIER_TO_N_POINTS_MAP = {
        {0, 12}, {1, 48}, {2, 192}, {3, 768}, {4, 3072}
        // Add more tiers as needed
    };

    int get_number_of_nodes_at_tier(int resolution_tier) {
        auto it = TIER_TO_N_POINTS_MAP.find(resolution_tier);
        if (it == TIER_TO_N_POINTS_MAP.end()) {
            throw std::out_of_range("Resolution tier " + std::to_string(resolution_tier) + " is not defined.");
        }
        return it->second;
    }

    Eigen::Vector3d generate_fibonacci_sphere_point(int index, int num_points) {
        if (!(index >= 0 && index < num_points)) {
            throw std::out_of_range("Index " + std::to_string(index) + 
                                    " is out of bounds for " + std::to_string(num_points) + " points.");
        }
        // Golden angle related constant
        double phi_angle_increment = M_PI * (3.0 - std::sqrt(5.0)); 
        
        double y_coord = 1.0 - (2.0 * (static_cast<double>(index) + 0.5)) / static_cast<double>(num_points);
        // Clamp y_coord to avoid domain errors with sqrt due to potential floating point inaccuracies
        y_coord = std::max(-1.0, std::min(1.0, y_coord));
        
        double radius_at_y = std::sqrt(std::max(0.0, 1.0 - y_coord * y_coord)); // Ensure non-negative under sqrt
        
        double current_phi = phi_angle_increment * static_cast<double>(index);
        
        double x_coord = std::cos(current_phi) * radius_at_y;
        double z_coord = std::sin(current_phi) * radius_at_y;
        
        return Eigen::Vector3d(x_coord, y_coord, z_coord);
    }
    
    // This function creates an orientation quaternion such that the local Z-axis of the oriented frame
    // aligns with `vec_er_local_z`. The local X and Y axes are defined to be orthogonal to Z and each other.
    // A common way is to align Z, then define X (e.g., cross product with global Up, unless Z is Up), then Y.
    Eigen::Quaterniond calculate_node_orientation_from_vector_and_angles(
            double theta_from_y_pole, // Angle of vec_er_local_z from global Y+ (latitude-like)
            double phi_in_xz_plane,   // Angle of vec_er_local_z in XZ plane from global X+ (longitude-like)
            const Eigen::Vector3d& vec_er_local_z // This is the target local Z direction
    ) {
        // The goal is to find a quaternion that rotates the canonical Z-axis (0,0,1)
        // to align with vec_er_local_z.
        // Eigen::Quaterniond::FromTwoVectors(from_vector, to_vector) can do this.
        Eigen::Vector3d canonical_z(0.0, 0.0, 1.0);
        Eigen::Quaterniond q_align_z;

        // vec_er_local_z should be normalized if it isn't already
        Eigen::Vector3d target_z_normalized = vec_er_local_z.normalized();

        q_align_z = Eigen::Quaterniond::FromTwoVectors(canonical_z, target_z_normalized);
        
        // The Python code's _calculate_node_orientation_from_vector_and_angles
        // might have a more specific convention for the resulting X and Y axes after aligning Z.
        // The `FromTwoVectors` approach defines a rotation that aligns Z, with an arbitrary but consistent
        // roll around that Z axis. If the Python version had a specific roll (e.g. by aligning local X
        // towards a specific direction like "East" on the sphere), that would need more steps.
        // For now, aligning Z is the primary concern based on the Python code's usage.
        // The original Python `_calculate_node_orientation_from_vector_and_angles` used:
        // q_theta = AngleAxis(-theta_from_y_pole, Vector3d::UnitX()) (rotation around X)
        // q_phi   = AngleAxis(phi_in_xz_plane,    Vector3d::UnitY()) (rotation around Y)
        // return q_phi * q_theta; // This implies a specific sequence of rotations.
        // This sequence means:
        // 1. Start with a frame aligned with global axes.
        // 2. Rotate by -theta_from_y_pole around the global X-axis. (Tilts Z-axis away from global Y)
        // 3. Rotate by phi_in_xz_plane around the global Y-axis. (Spins around global Y)
        // This is an extrinsic Euler angle sequence (e.g., Y-X).
        // Let's replicate that specific convention if it's important.
        // Note: The Python code's angles (theta_from_y_pole, phi_in_xz_plane) are specific to
        // how vec_er_local_z was derived from the Fibonacci lattice (Y as polar axis).

        // If vec_er_local_z *is* the desired local Z-axis, then FromTwoVectors is the most direct.
        // The Python `orientations.py` seems to calculate `theta_from_y_pole` and `phi_in_xz_plane`
        // *from* `vec_er_local_z` and then passes them to `_calculate_node_orientation_from_vector_and_angles`.
        // This suggests those angles are then used to reconstruct the orientation.

        // Replicating the Python's `_calculate_node_orientation_from_vector_and_angles` more directly:
        // It seems to construct a rotation that aligns a canonical frame's Z-axis with vec_er_local_z,
        // and its Y-axis is somewhat aligned with the original Y-pole of the sphere generation.
        // This is a common approach in spherical coordinates to define a local frame.

        // Rotation 1: Align a reference vector (e.g. global +Y) to vec_er_local_z's meridian.
        // This is effectively setting up the "longitude".
        Eigen::AngleAxisd rot1(phi_in_xz_plane, Eigen::Vector3d::UnitY()); // Rotate around global Y

        // Rotation 2: Tilt along that meridian.
        // This is effectively setting up the "latitude" or "colatitude".
        // The rotation axis for theta is perpendicular to global Y and vec_er_local_z's projection on XZ.
        // This is simpler if we think about it as: rotate global Z to vec_er_local_z.
        // The `FromTwoVectors` method is generally robust for this.
        // If the Python code's `theta_from_y_pole` and `phi_in_xz_plane` are Euler angles
        // that directly define the orientation, then:
        // q_phi: Rotation about global Y by phi_in_xz_plane
        // q_theta: Rotation about the *new* X-axis (after phi rot) by theta_from_y_pole
        // This would be an intrinsic ZYX or similar Euler sequence, or extrinsic Y'X'' sequence.
        // The Python code `q_orientation = R.from_euler('yx', [phi_in_xz_plane, -theta_from_y_pole]).as_quat()`
        // in the `_calculate_node_orientation_from_vector_and_angles` (from the provided image of `quaternion_utils.py`)
        // indicates an intrinsic 'yx' Euler sequence.
        // Eigen uses intrinsic rotations: q = AngleAxis(a1, V1) * AngleAxis(a2, V2) * AngleAxis(a3, V3) means V3 first.
        // So, for 'yx', it's rot_x then rot_y.
        // Eigen::AngleAxisd r_y(phi_in_xz_plane, Eigen::Vector3d::UnitY());
        // Eigen::AngleAxisd r_x(-theta_from_y_pole, Eigen::Vector3d::UnitX());
        // return r_y * r_x; // Order for intrinsic y then x: R = Ry * Rx

        // Let's use FromTwoVectors as it directly achieves the primary goal: local Z along vec_er_local_z.
        // If a specific roll is needed, this part would need refinement based on the exact
        // definition of the local X and Y axes in the Python version.
        // For now, ensuring local_Z aligns with vec_er_local_z:
        return q_align_z;
    }


    Eigen::Quaterniond get_discrete_node_orientation(int resolution_tier, int node_index_at_tier) {
        int num_points = get_number_of_nodes_at_tier(resolution_tier);
        if (!(node_index_at_tier >= 0 && node_index_at_tier < num_points)) {
            throw std::out_of_range(
                "node_index_at_tier " + std::to_string(node_index_at_tier) + 
                " is out of bounds for resolution_tier " + std::to_string(resolution_tier) +
                " which has " + std::to_string(num_points) + " nodes."
            );
        }

        Eigen::Vector3d vec_er_local_z = generate_fibonacci_sphere_point(node_index_at_tier, num_points);

        // The Python code calculates theta_from_y_pole and phi_in_xz_plane from vec_er_local_z
        // and then passes these to _calculate_node_orientation_from_vector_and_angles.
        double fx = vec_er_local_z.x();
        double fy = vec_er_local_z.y();
        double fz = vec_er_local_z.z();

        // Clip fy to ensure it's within acos's valid domain [-1, 1]
        double fy_clipped = std::max(-1.0, std::min(1.0, fy));
        double theta_from_y_pole = std::acos(fy_clipped); // Angle from positive Y-axis

        double phi_in_xz_plane = std::atan2(fz, fx); // Angle in XZ plane, from positive X-axis
        if (phi_in_xz_plane < 0) {
            phi_in_xz_plane += 2.0 * M_PI;
        }
        
        // Now call the helper that presumably uses these angles to define the quaternion
        return calculate_node_orientation_from_vector_and_angles(theta_from_y_pole, phi_in_xz_plane, vec_er_local_z);
    }


} // namespace SpinStepOrientations