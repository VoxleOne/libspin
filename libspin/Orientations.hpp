#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <map>

namespace SpinStepOrientations {

    // Equivalent to _TIER_TO_N_POINTS_MAP
    // Using a static map or a function to get these values.
    // A simple array can also work if tiers are contiguous and start from 0.
    extern const std::map<int, int> TIER_TO_N_POINTS_MAP;

    int get_number_of_nodes_at_tier(int resolution_tier);

    // Corresponds to _generate_fibonacci_sphere_point
    Eigen::Vector3d generate_fibonacci_sphere_point(int index, int num_points);

    // Corresponds to get_discrete_node_orientation
    // (and _calculate_node_orientation_from_vector_and_angles from Python's quaternion_utils)
    Eigen::Quaterniond get_discrete_node_orientation(int resolution_tier, int node_index_at_tier);

    // Helper that was in Python's quaternion_utils.py and used by orientations.py
    // _calculate_node_orientation_from_vector_and_angles
    Eigen::Quaterniond calculate_node_orientation_from_vector_and_angles(
        double theta_from_y_pole, 
        double phi_in_xz_plane,
        const Eigen::Vector3d& vec_er_local_z // The vector that becomes the local Z
    );

} // namespace SpinStepOrientations
