// solution.cpp
#include <limits>
#include "config.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <string>
#include "solution.h" // Route, Solution, VRPInstance
#include "untils.h" // evaluate_route_with_penalty
#include "insertion.h" // insert_customer_best_position, local_post_optimization
#include "vrp_parser.h" // Customer, VRPInstance
#include "solution.h" // Route, Solution
/*struct Route {
    std::vector<int> customer_ids; // sequence of customer indices (0-based)
    int total_demand = 0;
    double total_cost = 0.0;
};
struct Solution {
    std::vector<Route> routes;
    double total_cost = 0.0;
};*/

// Evaluate a route: compute demand and cost
// Evaluate một tuyến
using namespace std;
double evaluate_route_with_penalty(Route& route, const VRPInstance& vrp, double alpha, double beta) {
    if (route.customer_ids.empty()) {
        // Bỏ qua route rỗng
        return 0.0;
    }

    int total_demand = 0;
    double total_length = 0.0;
    int prev = 0;
    for (int cid : route.customer_ids) {
        total_demand += vrp.customers[cid].demand;
        total_length += vrp.distance_matrix[prev][cid];
        prev = cid;
    }
    total_length += vrp.distance_matrix[prev][0]; // quay về depot

    route.total_demand = total_demand;
    route.total_cost   = total_length;

    if (USE_PENALTY) {
        // Phiên bản có penalty
        double penalty = 0.0;
        if (total_demand > vrp.capacity) {
            penalty += alpha * max(0,total_demand - vrp.capacity);
        }
        if (total_length > vrp.distance) {
            penalty += beta * max(0.0,((total_length - vrp.distance)/vrp.distance)*total_length);
        }
        return total_length + penalty;
    } else {
        // Phiên bản không penalty: tuyến không hợp lệ → gán cost = 1e9
        if (total_demand > vrp.capacity || total_length > vrp.distance) {
            return 1e9;
        }
        return total_length;
    }
}

// Evaluate toàn bộ solution
// Trong solution.cpp
// Sửa lại hàm evaluate_solution

double evaluate_solution(Solution& sol, const VRPInstance& vrp, double alpha, double beta) {
    double total_penalized_cost = 0.0;
    double total_actual_cost = 0.0; // Thêm biến này

    for (auto& route : sol.routes) {
        // evaluate_route_with_penalty đã cập nhật route.total_cost (chi phí thực)
        // và route.total_demand
        double penalized_cost = evaluate_route_with_penalty(route, vrp, alpha, beta);

        if (penalized_cost < 0) { // Tuyến rỗng
            continue;
        }

        if (!USE_PENALTY && penalized_cost >= 1e9) {
            // Chế độ Hard và vi phạm
            sol.total_cost = 1e9;
            return 1e9;
        }

        total_penalized_cost += penalized_cost;
        total_actual_cost += route.total_cost; // Cộng chi phí thực
    }
    
    // Cập nhật chi phí THỰC của giải pháp
    sol.total_cost = total_actual_cost; 
    
    // Trả về chi phí CÓ PHẠT để thuật toán sử dụng
    return total_penalized_cost;
}

// Print solution to console with actual distances
void print_solution(const Solution& sol, const VRPInstance& vrp) {
    std::cout << std::fixed << std::setprecision(2);
    double total_distance = 0.0;

    for (size_t i = 0; i < sol.routes.size(); ++i) {
        std::cout << "Route #" << i + 1 << ":  0 ";
        double route_distance = 0.0;
        
        if (!sol.routes[i].customer_ids.empty()) {
            // Distance: depot to first
            route_distance += vrp.distance_matrix[0][sol.routes[i].customer_ids[0]];
            
            // Print customers and add distances between them
            for (size_t j = 0; j < sol.routes[i].customer_ids.size(); ++j) {
                int cid = sol.routes[i].customer_ids[j];
                std::cout << cid << " ";
                
                if (j + 1 < sol.routes[i].customer_ids.size()) {
                    route_distance += vrp.distance_matrix[cid][sol.routes[i].customer_ids[j + 1]];
                }
            }
            
            // Distance: last customer back to depot
            route_distance += vrp.distance_matrix[sol.routes[i].customer_ids.back()][0];
        }
        
        total_distance += route_distance;
        std::cout << "0    Distance: " << route_distance + sol.routes[i].customer_ids.size()*vrp.service_time<< "\n";
    }
    std::cout << "Total Cost: " << total_distance << std::endl;
}

void write_solution_to_file(const Solution& sol,
                            const std::string& filename,
                            const std::string& vrp_name,
                            double time,
                            int vehicle_count,
                            bool penalty_mode,
                            double alpha,
                            double beta,
                            int iter,
                            int tabu_tenure,
                            const VRPInstance& vrp) 
{
    std::ofstream fout(filename);
    if (!fout) {
        std::cerr << "Error writing to file: " << filename << std::endl;
        return;
    }
    fout << std::fixed << std::setprecision(2);

    // Write header info
    fout << "Instance: " << vrp_name << "\n";
    fout << "Vehicle count: " << vehicle_count << "\n";
    fout << "Penalty mode: " << (penalty_mode ? 1 : 0) << "\n";
    fout << "Alpha: " << alpha << " | Beta: " << beta << "\n";
    fout << "Iter stop: " << iter << "\n";
    fout << "Tabu tenure: " << tabu_tenure << "\n";

    double total_distance = 0.0;

    // Write each route with actual distances
    for (size_t i = 0; i < sol.routes.size(); ++i) {
        fout << "Route #" << i + 1 << ": 0 ";
        int route_demand = 0;
        double route_distance = 0.0;

        if (!sol.routes[i].customer_ids.empty()) {
            // Distance: depot to first
            route_distance += vrp.distance_matrix[0][sol.routes[i].customer_ids[0]];
            
            // Write customers and calculate distances
            for (size_t j = 0; j < sol.routes[i].customer_ids.size(); ++j) {
                int cid = sol.routes[i].customer_ids[j];
                fout << cid << " ";
                route_demand += vrp.customers[cid].demand;

                if (j + 1 < sol.routes[i].customer_ids.size()) {
                    route_distance += vrp.distance_matrix[cid][sol.routes[i].customer_ids[j + 1]];
                }
            }
            
            // Distance: last customer back to depot
            route_distance += vrp.distance_matrix[sol.routes[i].customer_ids.back()][0];
        }

        total_distance += route_distance;
        fout << "0 | Demand = " << route_demand 
             << " | Distance = " << route_distance + sol.routes[i].customer_ids.size()*vrp.service_time<< "\n";
    }

    fout << "Total Cost: " << total_distance << "\n";
    fout << "Time: " << time << "\n";
    fout.close();
}


