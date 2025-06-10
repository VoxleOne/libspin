#include "SpinStepGraph.hpp"
#include <stdexcept>
#include <algorithm> // For std::min, std::sort
#include <cmath> // For M_PI
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// --- Quaternion Math Helpers ---
Eigen::Quaterniond SpinStepGraph::get_relative_spin_quaternion(const Eigen::Quaterniond& q_from, const Eigen::Quaterniond& q_to) {
    // If q_rel * q_from = q_to, then q_rel = q_to * q_from.inverse()
    return q_to * q_from.inverse();
}

double SpinStepGraph::quaternion_angular_distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    // Eigen's angularDistance is 2 * acos( |q1.dot(q2)| ) for normalized quaternions.
    // This is the shortest angle between the orientations.
    return q1.angularDistance(q2);
}


// --- Constructor & Destructor ---
SpinStepGraph::SpinStepGraph(const std::vector<LayerDefinition>& layer_defs,
                             int num_tang_neighbors,
                             RadialConnectionMode radial_mode,
                             double radial_angle_thresh_rad)
    : layer_definitions_(layer_defs),
      num_tangential_neighbors_(num_tang_neighbors),
      radial_connection_mode_(radial_mode),
      radial_angle_threshold_rad_(radial_angle_thresh_rad) {
    _build_graph();
}

SpinStepGraph::~SpinStepGraph() {
    // all_nodes_storage_ (vector of unique_ptr) will automatically delete Nodes.
    // Other containers store raw pointers, so no explicit deletion needed here for them.
}

// --- Graph Building ---
void SpinStepGraph::_build_graph() {
    all_nodes_storage_.clear();
    nodes_by_id_ptrs_.clear();
    layers_node_ptrs_.assign(layer_definitions_.size(), std::vector<Node*>());

    // Phase 1: Create all nodes
    for (size_t layer_idx = 0; layer_idx < layer_definitions_.size(); ++layer_idx) {
        const auto& layer_def = layer_definitions_[layer_idx];
        int num_nodes_on_this_layer = SpinStepOrientations::get_number_of_nodes_at_tier(layer_def.resolution_tier);

        for (int node_idx_on_layer = 0; node_idx_on_layer < num_nodes_on_this_layer; ++node_idx_on_layer) {
            Eigen::Quaterniond orientation = SpinStepOrientations::get_discrete_node_orientation(
                layer_def.resolution_tier, node_idx_on_layer);
            
            // Create node and transfer ownership to all_nodes_storage_
            auto node_ptr = std::make_unique<Node>(
                static_cast<int>(layer_idx), node_idx_on_layer, layer_def.resolution_tier, orientation, layer_def.radius);
            
            Node* raw_node_ptr = node_ptr.get();
            all_nodes_storage_.push_back(std::move(node_ptr));
            
            nodes_by_id_ptrs_[raw_node_ptr->id] = raw_node_ptr;
            layers_node_ptrs_[layer_idx].push_back(raw_node_ptr);
        }
    }

    // Phase 2: Connect tangential neighbors
    for (size_t layer_idx = 0; layer_idx < layers_node_ptrs_.size(); ++layer_idx) {
        std::vector<Node*>& current_layer_nodes = layers_node_ptrs_[layer_idx];
        if (current_layer_nodes.empty() || num_tangential_neighbors_ == 0) continue;

        // Placeholder for KDTree: Using brute force for now.
        // For production, a C++ KD-Tree library (e.g., nanoflann, FLANN) would be needed here.
        for (Node* node : current_layer_nodes) {
            std::vector<std::pair<double, Node*>> neighbor_candidates;
            for (Node* potential_neighbor : current_layer_nodes) {
                if (node == potential_neighbor) continue;
                // Using squared Euclidean distance on positions for KD-tree like behavior
                double dist_sq = (node->position - potential_neighbor->position).squaredNorm();
                neighbor_candidates.push_back({dist_sq, potential_neighbor});
            }

            std::sort(neighbor_candidates.begin(), neighbor_candidates.end(), 
                      [](const auto& a, const auto& b){ return a.first < b.first; });

            int added_count = 0;
            for (size_t i = 0; i < neighbor_candidates.size() && added_count < num_tangential_neighbors_; ++i) {
                Node* neighbor_node_obj = neighbor_candidates[i].second;
                Eigen::Quaterniond spin_q = get_relative_spin_quaternion(node->orientation, neighbor_node_obj->orientation);
                node->add_tangential_neighbor(neighbor_node_obj, spin_q);
                added_count++;
            }
        }
    }

    // Phase 3: Connect radial neighbors
    for (size_t layer_idx = 0; layer_idx < layers_node_ptrs_.size(); ++layer_idx) {
        std::vector<Node*>& current_layer_nodes_for_radial = layers_node_ptrs_[layer_idx];
        if (current_layer_nodes_for_radial.empty()) continue;

        for (Node* node : current_layer_nodes_for_radial) {
            // Outward connections
            if (layer_idx + 1 < layers_node_ptrs_.size()) {
                const auto& outer_layer_nodes = layers_node_ptrs_[layer_idx + 1];
                if (!outer_layer_nodes.empty()) {
                    if (radial_connection_mode_ == RadialConnectionMode::MATCH_INDEX) {
                        const auto& outer_layer_def = layer_definitions_[layer_idx + 1];
                        int num_nodes_on_outer = SpinStepOrientations::get_number_of_nodes_at_tier(outer_layer_def.resolution_tier);
                        if (node->id.node_index_on_layer < num_nodes_on_outer) {
                            NodeId candidate_id = {static_cast<int>(layer_idx + 1), node->id.node_index_on_layer};
                            auto it = nodes_by_id_ptrs_.find(candidate_id);
                            if (it != nodes_by_id_ptrs_.end()) {
                                node->set_radial_outward_neighbor(it->second);
                            }
                        }
                    } else if (radial_connection_mode_ == RadialConnectionMode::CLOSEST_ORIENTATION) {
                        Node* best_neighbor = nullptr;
                        double min_angle = radial_angle_threshold_rad_;
                        for (Node* candidate_neighbor : outer_layer_nodes) {
                            double angle = quaternion_angular_distance(node->orientation, candidate_neighbor->orientation);
                            if (angle < min_angle) {
                                min_angle = angle;
                                best_neighbor = candidate_neighbor;
                            }
                        }
                        if (best_neighbor) {
                            node->set_radial_outward_neighbor(best_neighbor);
                        }
                    }
                }
            }
            // Inward connections (similar logic)
            if (layer_idx > 0) {
                 const auto& inner_layer_nodes = layers_node_ptrs_[layer_idx - 1];
                if (!inner_layer_nodes.empty()) {
                     if (radial_connection_mode_ == RadialConnectionMode::MATCH_INDEX) {
                        const auto& inner_layer_def = layer_definitions_[layer_idx - 1];
                        int num_nodes_on_inner = SpinStepOrientations::get_number_of_nodes_at_tier(inner_layer_def.resolution_tier);
                        if (node->id.node_index_on_layer < num_nodes_on_inner) {
                            NodeId candidate_id = {static_cast<int>(layer_idx - 1), node->id.node_index_on_layer};
                             auto it = nodes_by_id_ptrs_.find(candidate_id);
                            if (it != nodes_by_id_ptrs_.end()) {
                                node->set_radial_inward_neighbor(it->second);
                            }
                        }
                    } else if (radial_connection_mode_ == RadialConnectionMode::CLOSEST_ORIENTATION) {
                        Node* best_neighbor = nullptr;
                        double min_angle = radial_angle_threshold_rad_;
                        for (Node* candidate_neighbor : inner_layer_nodes) {
                            double angle = quaternion_angular_distance(node->orientation, candidate_neighbor->orientation);
                            if (angle < min_angle) {
                                min_angle = angle;
                                best_neighbor = candidate_neighbor;
                            }
                        }
                        if (best_neighbor) {
                            node->set_radial_inward_neighbor(best_neighbor);
                        }
                    }
                }
            }
        }
    }
}


// --- Public API Methods ---
Node* SpinStepGraph::get_node(const NodeId& id) const {
    auto it = nodes_by_id_ptrs_.find(id);
    return (it == nodes_by_id_ptrs_.end()) ? nullptr : it->second;
}

Node* SpinStepGraph::get_node(int layer_index, int node_index_on_layer) const {
    return get_node(NodeId{layer_index, node_index_on_layer});
}

SpinStepGraph::NeighborDetail SpinStepGraph::get_tangential_neighbor_details(
    const NodeId& current_node_id, int tangential_neighbor_index) const {
    Node* current_node = get_node(current_node_id);
    if (!current_node) return { {}, {}, false};
    if (tangential_neighbor_index < 0 || 
        static_cast<size_t>(tangential_neighbor_index) >= current_node->tangential_neighbors.size()) {
        return { {}, {}, false};
    }
    const auto& neighbor_pair = current_node->tangential_neighbors[tangential_neighbor_index];
    return { neighbor_pair.first->id, neighbor_pair.second, true };
}

SpinStepGraph::NeighborDetail SpinStepGraph::get_radial_neighbor_details(
    const NodeId& current_node_id, const std::string& direction) const {
    Node* current_node = get_node(current_node_id);
    if (!current_node) return { {}, {}, false};
    
    Node* neighbor_node_obj = nullptr;
    if (direction == "outward") {
        neighbor_node_obj = current_node->radial_outward_neighbor;
    } else if (direction == "inward") {
        neighbor_node_obj = current_node->radial_inward_neighbor;
    } else {
        return { {}, {}, false}; // Invalid direction
    }

    if (!neighbor_node_obj) return { {}, {}, false}; // No neighbor in that direction

    Eigen::Quaterniond relative_spin_q = get_relative_spin_quaternion(current_node->orientation, neighbor_node_obj->orientation);
    return { neighbor_node_obj->id, relative_spin_q, true };
}


SpinStepGraph::SpinStepResult SpinStepGraph::initiate_spin_step(
    const NodeId& current_node_id,
    const Eigen::Quaterniond& current_absolute_orientation_q,
    const Eigen::Quaterniond& spin_instruction_q) const {
    
    Node* current_node = get_node(current_node_id);
    if (!current_node) return { {}, {}, {}, false };

    Eigen::Quaterniond new_absolute_orientation_q = spin_instruction_q * current_absolute_orientation_q;
    new_absolute_orientation_q.normalize(); // Ensure it's a unit quaternion

    std::vector<Node*> candidate_nodes;
    candidate_nodes.push_back(current_node); // Start with the current node

    for (const auto& tan_neighbor_pair : current_node->tangential_neighbors) {
        candidate_nodes.push_back(tan_neighbor_pair.first);
    }
    if (current_node->radial_outward_neighbor) {
        candidate_nodes.push_back(current_node->radial_outward_neighbor);
    }
    if (current_node->radial_inward_neighbor) {
        candidate_nodes.push_back(current_node->radial_inward_neighbor);
    }
    
    // Remove duplicates just in case, though graph structure should prevent it for direct neighbors
    std::sort(candidate_nodes.begin(), candidate_nodes.end());
    candidate_nodes.erase(std::unique(candidate_nodes.begin(), candidate_nodes.end()), candidate_nodes.end());

    if (candidate_nodes.empty()) { // Should not happen if current_node is valid
         return { current_node->id, new_absolute_orientation_q, spin_instruction_q, true }; // Or indicate error
    }

    Node* best_next_node = current_node; // Default to current node
    double min_angle_to_new_orientation = quaternion_angular_distance(current_node->orientation, new_absolute_orientation_q);

    for (Node* candidate_node_obj : candidate_nodes) {
        // if (candidate_node_obj == current_node) continue; // Already calculated for current_node
        double angle = quaternion_angular_distance(candidate_node_obj->orientation, new_absolute_orientation_q);
        if (angle < min_angle_to_new_orientation) {
            min_angle_to_new_orientation = angle;
            best_next_node = candidate_node_obj;
        }
    }
    
    return { best_next_node->id, new_absolute_orientation_q, spin_instruction_q, true };
}
