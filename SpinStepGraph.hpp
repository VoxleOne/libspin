#pragma once

#include "Node.hpp" // Includes Eigen Quaterniond, Vector3d, NodeId
#include "Orientations.hpp" // For SpinStepOrientations namespace

#include <vector>
#include <string>
#include <unordered_map> // For nodes_by_id
#include <memory> // For std::unique_ptr to manage Node lifetimes
#include <limits> // For std::numeric_limits

// Forward declaration for KDTree placeholder/library if we decide to integrate one later
// For now, we might implement a simpler search or note it for optimization.

// Definition for a layer
struct LayerDefinition {
    double radius;
    int resolution_tier;
};

// Enum for radial connection mode
enum class RadialConnectionMode {
    MATCH_INDEX,
    CLOSEST_ORIENTATION
};

class SpinStepGraph {
public:
    // Constructor
    SpinStepGraph(const std::vector<LayerDefinition>& layer_defs,
                  int num_tang_neighbors = 6,
                  RadialConnectionMode radial_mode = RadialConnectionMode::CLOSEST_ORIENTATION,
                  double radial_angle_thresh_rad = M_PI / 4.0);

    // Destructor to clean up nodes
    ~SpinStepGraph();

    // Public API mirroring Python version
    Node* get_node(const NodeId& id) const; // Using NodeId struct
    Node* get_node(int layer_index, int node_index_on_layer) const;

    // Returns {neighbor_node_id, relative_spin_quaternion}
    // Using std::optional or a struct to return multiple values, or pass by reference
    struct NeighborDetail {
        NodeId node_id;
        Eigen::Quaterniond relative_spin_q;
        bool found = false;
    };
    NeighborDetail get_tangential_neighbor_details(const NodeId& current_node_id, int tangential_neighbor_index) const;
    NeighborDetail get_radial_neighbor_details(const NodeId& current_node_id, const std::string& direction) const; // direction: "inward" or "outward"

    struct SpinStepResult {
        NodeId best_next_node_id;
        Eigen::Quaterniond new_absolute_orientation_q;
        Eigen::Quaterniond applied_spin_instruction_q; // The original spin_instruction_q
        bool found = false;
    };
    SpinStepResult initiate_spin_step(const NodeId& current_node_id,
                                      const Eigen::Quaterniond& current_absolute_orientation_q,
                                      const Eigen::Quaterniond& spin_instruction_q) const;
    
    // Accessor for layers and nodes if needed for debugging or external tools
    const std::vector<std::vector<Node*>>& get_layers() const { return layers_node_ptrs_; }
    const std::unordered_map<NodeId, Node*>& get_nodes_by_id() const { return nodes_by_id_ptrs_; }


private:
    std::vector<LayerDefinition> layer_definitions_;
    int num_tangential_neighbors_;
    RadialConnectionMode radial_connection_mode_;
    double radial_angle_threshold_rad_;

    // Node storage and access
    // Master storage for all nodes - ensures they are deallocated
    std::vector<std::unique_ptr<Node>> all_nodes_storage_; 
    // Fast lookup by ID (raw pointers, lifetime managed by all_nodes_storage_)
    std::unordered_map<NodeId, Node*> nodes_by_id_ptrs_;
    // Nodes organized by layer (raw pointers)
    std::vector<std::vector<Node*>> layers_node_ptrs_;

    void _build_graph();

    // Helper for quaternion math (can also be static members of a utility class or free functions)
    static Eigen::Quaterniond get_relative_spin_quaternion(const Eigen::Quaterniond& q_from, const Eigen::Quaterniond& q_to);
    static double quaternion_angular_distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
};