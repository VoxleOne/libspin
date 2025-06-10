#include <iostream>
#include <vector>
#include <string>
#include <iomanip> // For std::fixed, std::setprecision

#include "SpinStepGraph.hpp" // This should include Node.hpp and Orientations.hpp indirectly or directly

// Helper to print quaternion
void print_quaternion(const Eigen::Quaterniond& q, const std::string& label = "") {
    if (!label.empty()) {
        std::cout << label << ": ";
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[w:" << q.w() << ", x:" << q.x() << ", y:" << q.y() << ", z:" << q.z() << "]"
              << std::endl;
}

// Helper to print NodeId
void print_node_id(const NodeId& id, const std::string& label = "") {
    if (!label.empty()) {
        std::cout << label << ": ";
    }
    std::cout << "{" << id.layer_index << ", " << id.node_index_on_layer << "}" << std::endl;
}


int main() {
    std::cout << "SpinStep C++ Test Program" << std::endl;
    std::cout << "=========================" << std::endl;

    // 1. Define Layer Definitions
    std::vector<LayerDefinition> layer_defs = {
        {1.0, 0}, // Layer 0: radius 1.0, resolution_tier 0 (12 nodes)
        {1.5, 1}  // Layer 1: radius 1.5, resolution_tier 1 (48 nodes)
        // Add more layers as needed for testing
    };
    
    int num_tangential_neighbors = 4; // Example
    RadialConnectionMode radial_mode = RadialConnectionMode::CLOSEST_ORIENTATION;
    double radial_angle_thresh = M_PI / 3.0; // Example: 60 degrees

    std::cout << "\n--- Initializing SpinStepGraph ---" << std::endl;
    try {
        SpinStepGraph graph(layer_defs, num_tangential_neighbors, radial_mode, radial_angle_thresh);
        std::cout << "Graph initialized successfully." << std::endl;

        // 2. Print some basic graph info
        std::cout << "\n--- Graph Info ---" << std::endl;
        std::cout << "Total layers: " << graph.get_layers().size() << std::endl;
        for (size_t i = 0; i < graph.get_layers().size(); ++i) {
            std::cout << "Layer " << i << " has " << graph.get_layers()[i].size() << " nodes." << std::endl;
        }
        std::cout << "Total nodes in graph: " << graph.get_nodes_by_id().size() << std::endl;

        // 3. Get a specific node and print its details
        NodeId test_node_id = {0, 0}; // Get node 0 from layer 0
        Node* node0_0 = graph.get_node(test_node_id);

        if (node0_0) {
            std::cout << "\n--- Details for Node ";
            print_node_id(test_node_id, "");
            std::cout << "---" << std::endl;
            std::cout << node0_0->to_string() << std::endl;

            // Print its tangential neighbors
            std::cout << "  Tangential Neighbors (" << node0_0->tangential_neighbors.size() << "):" << std::endl;
            for (size_t i = 0; i < node0_0->tangential_neighbors.size(); ++i) {
                const auto& neighbor_pair = node0_0->tangential_neighbors[i];
                std::cout << "    Neighbor " << i << ": ";
                print_node_id(neighbor_pair.first->id);
                print_quaternion(neighbor_pair.second, "      Relative Spin");
                
                // Also test get_tangential_neighbor_details
                auto detail = graph.get_tangential_neighbor_details(test_node_id, static_cast<int>(i));
                if(detail.found){
                    std::cout << "      (Detail Check) ID: {" << detail.node_id.layer_index << "," << detail.node_id.node_index_on_layer << "}, Spin: [w:" 
                              << detail.relative_spin_q.w() << ",x:" << detail.relative_spin_q.x() << "]" << std::endl;
                }
            }

            // Print its radial neighbors
            std::cout << "  Radial Neighbors:" << std::endl;
            if (node0_0->radial_outward_neighbor) {
                std::cout << "    Outward: ";
                print_node_id(node0_0->radial_outward_neighbor->id);
                auto detail_out = graph.get_radial_neighbor_details(test_node_id, "outward");
                 if(detail_out.found){
                    std::cout << "      (Detail Check) ID: {" << detail_out.node_id.layer_index << "," << detail_out.node_id.node_index_on_layer << "}, Spin: [w:" 
                              << detail_out.relative_spin_q.w() << ",x:" << detail_out.relative_spin_q.x() << "]" << std::endl;
                }
            } else {
                std::cout << "    Outward: None" << std::endl;
            }
            if (node0_0->radial_inward_neighbor) {
                std::cout << "    Inward: ";
                print_node_id(node0_0->radial_inward_neighbor->id);
                 auto detail_in = graph.get_radial_neighbor_details(test_node_id, "inward");
                 if(detail_in.found){
                    std::cout << "      (Detail Check) ID: {" << detail_in.node_id.layer_index << "," << detail_in.node_id.node_index_on_layer << "}, Spin: [w:" 
                              << detail_in.relative_spin_q.w() << ",x:" << detail_in.relative_spin_q.x() << "]" << std::endl;
                }
            } else {
                std::cout << "    Inward: None" << std::endl;
            }
        } else {
            std::cout << "Node ";
            print_node_id(test_node_id);
            std::cout << " not found!" << std::endl;
        }

        // 4. Test initiate_spin_step
        std::cout << "\n--- Testing initiate_spin_step ---" << std::endl;
        if (node0_0) { // Use the node we found earlier
            Eigen::Quaterniond current_abs_orientation = node0_0->orientation; // Or some arbitrary orientation
            // Define a sample spin instruction (e.g., 90 degrees around Z-axis)
            Eigen::AngleAxisd spin_aa(M_PI / 2.0, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond spin_instruction_q(spin_aa);
            
            print_quaternion(current_abs_orientation, "Current Absolute Orientation");
            print_quaternion(spin_instruction_q, "Spin Instruction");

            SpinStepGraph::SpinStepResult result = graph.initiate_spin_step(
                test_node_id, current_abs_orientation, spin_instruction_q);

            if (result.found) {
                std::cout << "Spin Step Result:" << std::endl;
                print_node_id(result.best_next_node_id, "  Best Next Node ID");
                print_quaternion(result.new_absolute_orientation_q, "  New Absolute Orientation");
                print_quaternion(result.applied_spin_instruction_q, "  Applied Spin Instruction");
            } else {
                std::cout << "initiate_spin_step failed to find a result for node ";
                print_node_id(test_node_id);
            }
        }


    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\nTest program finished." << std::endl;
    return 0;
}