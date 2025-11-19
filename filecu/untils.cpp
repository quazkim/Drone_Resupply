// utils.cpp
#include "config.h"
#include <cmath>
#include <iostream> 
#include <vector>
#include "solution.h" // Route, Solution, VRPInstance
#include "untils.h" // evaluate_route_with_penalty
#include "insertion.h" // insert_customer_best_position, local_post_optimization
#include "vrp_parser.h" // Customer, VRPInstance
#include "solution.h" // Route, Solution
// Tính khoảng cách Euclidean giữa hai điểm
double euclidean_distance(const Customer& a, const Customer& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Tính tổng tải trọng của một tuyến
int calculate_total_demand(const std::vector<int>& customer_ids, const VRPInstance& vrp) {
    int total = 0;
    for (int id : customer_ids) {
        total += vrp.customers[id].demand;
        //std::cout << "Customer ID: " << id << ", Demand: " << vrp.customers[id].demand << std::endl;
    }
    return total;
}

// Kiểm tra một tuyến có hợp lệ hay không (không vượt quá tải trọng)
bool is_feasible_route(const std::vector<int>& customer_ids, const VRPInstance& vrp) {
    int total_demand = calculate_total_demand(customer_ids, vrp);
    double route_length = calculate_route_length(customer_ids, vrp);
    return total_demand <= vrp.capacity && (vrp.distance == 0 || route_length <= vrp.distance);
   
}


// Tính tổng độ dài của một tuyến (bắt đầu và kết thúc tại depot 0)
double calculate_route_length(const std::vector<int>& customer_ids, const VRPInstance& vrp) {
    if (customer_ids.empty()) return 0.0;
    double total = 0.0;
    int prev = 0; // depot
    for (int id : customer_ids) {
        total += vrp.distance_matrix[prev][id];
        prev = id;
    }
    total += vrp.distance_matrix[prev][0];
    // Cộng thêm service time cho mỗi khách hàng (không tính depot)
    total += customer_ids.size() * vrp.service_time;
    return total;
}
// Kiểm tra một solution có hợp lệ hay không
bool is_feasible_solution(const Solution& sol, const VRPInstance& vrp) {
    for (const auto& route : sol.routes) {
        if (!is_feasible_route(route.customer_ids, vrp)) {
            return false; // Có route vi phạm
        }
    }
    return true; // Tất cả các route đều hợp lệ
}

// Kiểm tra một solution có vi phạm tải trọng không
bool violate_capacity(const Solution& sol, const VRPInstance& vrp) {
    for (const auto& route : sol.routes) {
        int total_demand = calculate_total_demand(route.customer_ids, vrp);
        if (total_demand > vrp.capacity) return true;
    }
    return false;
}

// Kiểm tra một solution có vi phạm chiều dài tuyến không
bool violate_length(const Solution& sol, const VRPInstance& vrp) {
    if (vrp.distance == 0) return false; // Không giới hạn chiều dài
    for (const auto& route : sol.routes) {
        double route_length = calculate_route_length(route.customer_ids, vrp);
        if (route_length > vrp.distance) return true;
    }
    return false;
}