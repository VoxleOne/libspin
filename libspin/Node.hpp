#pragma once

#include <vector>
#include <string> // Or a custom ID struct if preferred over std::pair for node ID
#include <utility> // For std::pair if used for ID or neighbors
// It's good practice to forward-declare types from other headers if only pointers/references are used
// class SpinStepGraph; // If Node needs to know about SpinStepGraph (likely not directly)

// Assuming use of Eigen for quaternions and vectors
// You'll need to include Eigen headers in your project's build system
#include <Eigen/Dense> // For Vector3d
#include <Eigen/Geometry> // For Quaterniond

// For hashing node IDs if used as keys in std::unordered_map
struct NodeId {
    int layer_index;
    int node_index_on_layer;

    bool operator==(const NodeId& other) const {
        return layer_index == other.layer_index && node_index_on_layer == other.node_index_on_layer;
    }
};

// Hash function for NodeId
namespace std {
    template <>
    struct hash<NodeId> {
        size_t operator()(const NodeId& id) const {
            // A simple hash combination
            size_t h1 = std::hash<int>()(id.layer_index);
            size_t h2 = std::hash<int>()(id.node_index_on_layer);
            return h1 ^ (h2 << 1); 
        }
    };
}


class Node {
public:
    NodeId id;
    int resolution_tier;
    Eigen::Quaterniond orientation; // orientation_q
    double radius;
    Eigen::Vector3d position;      // Calculated from orientation and radius

    // Connections
    // Stores pairs of (neighbor_node_pointer, relative_spin_quaternion)
    std::vector<std::pair<Node*, Eigen::Quaterniond>> tangential_neighbors;
    Node* radial_outward_neighbor;
    Node* radial_inward_neighbor;

    Node(int layer_idx, int node_idx_on_layer, int res_tier, 
         const Eigen::Quaterniond&_orientation, double _radius);

    void add_tangential_neighbor(Node* neighbor_node, const Eigen::Quaterniond& relative_spin_q);
    void set_radial_outward_neighbor(Node* neighbor_node);
    void set_radial_inward_neighbor(Node* neighbor_node);

    // Helper to calculate position based on orientation and radius
    static Eigen::Vector3d calculate_position(const Eigen::Quaterniond& q_orientation, double r);
    
    // For debugging or representation
    std::string to_string() const;

    // Equality and hash for use in sets or as map keys (based on ID)
    bool operator==(const Node& other) const;
    // For std::unordered_map/set, provide a hash function or specialize std::hash

private:
    void _initialize_position(); // Called by constructor
};
