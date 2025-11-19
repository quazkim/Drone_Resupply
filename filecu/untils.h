#pragma once
#include "vrp_parser.h" // Đảm bảo Customer và VRPInstance đã được khai báo
#include <vector>

// Tính khoảng cách Euclidean giữa hai điểm
double euclidean_distance(const Customer& a, const Customer& b);

// Tính tổng tải trọng của một tuyến
int calculate_total_demand(const std::vector<int>& customer_ids, const VRPInstance& vrp);

// Kiểm tra một tuyến có hợp lệ hay không (không vượt quá tải trọng)
bool is_feasible_route(const std::vector<int>& customer_ids, const VRPInstance& vrp);

// Tính tổng độ dài của một tuyến (bắt đầu và kết thúc tại depot 0)
double calculate_route_length(const std::vector<int>& customer_ids, const VRPInstance& vrp);
// Kiểm tra một solution có hợp lệ hay không
bool is_feasible_solution(const Solution& sol, const VRPInstance& vrp);
//Kiểm tra một solution có vi phạm tải trọng hay không 
bool violate_capacity(const Solution& sol, const VRPInstance& vrp);
//Kiểm tra một solution có vi phạm distance hay không 
bool violate_length(const Solution& sol, const VRPInstance& vrp);
