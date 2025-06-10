#include "Node.hpp"
#include <stdexcept> // For exceptions
#include <cmath>     // For std::abs, std::sqrt, M_PI (if needed directly)
#include <sstream>   // For to_string
#include <iomanip>   // For std::fixed, std::setprecision

// Constructor
Node::Node(int layer_idx, int node_idx_on_layer, int res_tier, 
           const Eigen::Quaterniond& _orientation, double _radius)
    : id{layer_idx, node_idx_on_layer}, 
      resolution_tier(res_tier), 
      orientation(_orientation), 
      radius(_radius),
      radial_outward_neighbor(nullptr),
      radial_inward_neighbor(nullptr) {

    if (layer_idx < 0) {
        throw std::invalid_argument("layer_index must be a non-negative integer.");
    }
    if (node_idx_on_layer < 0) {
        throw std::invalid_argument("node_index_on_layer must be a non-negative integer.");
    }
    if (radius <= 0) {
        throw std::invalid_argument("radius must be a positive number.");
    }

    // Ensure orientation is normalized
    if (std::abs(orientation.norm() - 1.0) > 1e-6) {
        orientation.normalize();
        if (std::abs(orientation.norm() - 1.0) > 1e-6) { // Check again after normalization
             // If still not normalized (e.g. was zero quat), throw or handle
            throw std::runtime_error("Orientation quaternion could not be normalized (is it zero?).");
        }
    }
    
    _initialize_position();
}

Eigen::Vector3d Node::calculate_position(const Eigen::Quaterniond& q_orientation, double r) {
    // The local Z-axis of the node points radially outward.
    // This direction is obtained by rotating a unit vector along Z (0,0,1) by the node's orientation.
    Eigen::Vector3d local_z_axis(0.0, 0.0, 1.0);
    Eigen::Vector3d direction_vector = q_orientation * local_z_axis; // Eigen handles q * v
    return direction_vector * r;
}

void Node::_initialize_position() {
    this->position = calculate_position(this->orientation, this->radius);
}

void Node::add_tangential_neighbor(Node* neighbor_node, const Eigen::Quaterniond& relative_spin_q) {
    if (!neighbor_node) {
        throw std::invalid_argument("neighbor_node cannot be null.");
    }
    // Optional: check if relative_spin_q is normalized
    tangential_neighbors.push_back({neighbor_node, relative_spin_q.normalized()});
}

void Node::set_radial_outward_neighbor(Node* neighbor_node) {
    // neighbor_node can be nullptr
    radial_outward_neighbor = neighbor_node;
}

void Node::set_radial_inward_neighbor(Node* neighbor_node) {
    // neighbor_node can be nullptr
    radial_inward_neighbor = neighbor_node;
}

std::string Node::to_string() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Node(id={" << id.layer_index << "," << id.node_index_on_layer
        << "}, tier=" << resolution_tier
        << ", r=" << radius
        << ", pos=(" << position.x() << "," << position.y() << "," << position.z() << ")"
        << ", ori_q=(" << orientation.x() << "," << orientation.y() << ","
        << orientation.z() << "," << orientation.w() << ")"
        << ", T_neigh=" << tangential_neighbors.size()
        << ", R_out=" << (radial_outward_neighbor ? "Yes" : "No")
        << ", R_in=" << (radial_inward_neighbor ? "Yes" : "No") << ")";
    return oss.str();
}

bool Node::operator==(const Node& other) const {
    return this->id == other.id;
}

// Note: A std::hash<Node> specialization would be needed if Node objects themselves are used as keys
// in unordered containers. Here, we've specialized std::hash<NodeId>.