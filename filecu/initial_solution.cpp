#include "config.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "vrp_parser.h"
#include "solution.h"
#include "untils.h"
#include "initial_solution.h"
using namespace std;

// Chọn k khách xa depot nhất
vector<int> select_k_farthest_customers(const VRPInstance& vrp, int k) {
    vector<pair<int, double>> distances;
    for (int i = 1; i < (int)vrp.customers.size(); ++i) {
        double dist = euclidean_distance(vrp.customers[0], vrp.customers[i]);
        distances.emplace_back(i, dist);
    }
    // Sort theo khoảng cách giảm dần (xa nhất trước)
    sort(distances.begin(), distances.end(),
         [](const auto& a, const auto& b) { return a.second > b.second; });

    vector<int> farthest;
    for (int i = 0; i < k && i < (int)distances.size(); ++i) {
        farthest.push_back(distances[i].first);
    }
    return farthest;
}

void initial_solution(const VRPInstance& vrp, Solution& sol, int k) {
    sol.routes.clear();
    sol.total_cost = 0.0;

    int n_customers = (int)vrp.customers.size() - 1;
    if (n_customers <= 0) return;

    k = std::max(1, std::min(k, n_customers));
    vector<bool> visited(vrp.customers.size(), false);
    visited[0] = true;

    // Seed routes với k khách xa depot nhất
    vector<int> seeds = select_k_farthest_customers(vrp, k);
    for (int s : seeds) {
        Route r;
        r.customer_ids = { s };
        r.total_demand = calculate_total_demand(r.customer_ids, vrp);
        r.total_cost = calculate_route_length(r.customer_ids, vrp);
        sol.routes.push_back(r);
        visited[s] = true;
    }

    // Thêm route rỗng nếu cần
    while ((int)sol.routes.size() < k) {
        Route r;
        r.customer_ids.clear();
        r.total_demand = 0;
        r.total_cost = 0.0;
        sol.routes.push_back(r);
    }

    // Tính score cho mỗi khách chưa được gán
    vector<pair<int, double>> unassigned;
    for (int i = 1; i < (int)vrp.customers.size(); ++i) {
        if (!visited[i]) {
            // Score = demand*(0.7) + distance_to_depot*(0.3)
            double dist = euclidean_distance(vrp.customers[0], vrp.customers[i]);
            double score = 0.7 * vrp.customers[i].demand + 0.3 * dist;
            unassigned.emplace_back(i, score);
        }
    }
    
    // Sort theo score giảm dần (ưu tiên high demand & far customers)
    sort(unassigned.begin(), unassigned.end(),
         [](const auto& a, const auto& b) { return a.second > b.second; });

    // Greedy assignment cho từng khách
    for (const auto& p : unassigned) {
        int cust_id = p.first;
        double cust_score = p.second;
        struct BestPos {
            int route_idx = -1;
            int pos = -1;
            double cost_increase = std::numeric_limits<double>::infinity();
            double route_length = 0;
            double balanced_score = std::numeric_limits<double>::infinity();
        } best;

        // Tính độ dài trung bình của các route
        double avg_length = 0.0;
        int non_empty_routes = 0;
        for (const auto& r : sol.routes) {
            if (!r.customer_ids.empty()) {
                avg_length += calculate_route_length(r.customer_ids, vrp);
                non_empty_routes++;
            }
        }
        avg_length = non_empty_routes > 0 ? avg_length / non_empty_routes : 0;

        // Thử mọi vị trí có thể
        for (int r = 0; r < (int)sol.routes.size(); ++r) {
            const Route& route = sol.routes[r];
            double current_length = calculate_route_length(route.customer_ids, vrp);
            
            for (int pos = 0; pos <= (int)route.customer_ids.size(); ++pos) {
                vector<int> temp_ids = route.customer_ids;
                temp_ids.insert(temp_ids.begin() + pos, cust_id);

                int temp_demand = calculate_total_demand(temp_ids, vrp);
                if (temp_demand > vrp.capacity) continue;

                double new_length = calculate_route_length(temp_ids, vrp);
                double increase = new_length - current_length;

                // Tính balanced_score dựa trên:
                // 1. Tăng chi phí (increase)
                // 2. Độ lệch so với độ dài trung bình
                // 3. Số khách trong route
                double length_deviation = std::abs(new_length - avg_length);
                double balanced_score = increase + 
                                     0.3 * length_deviation + 
                                     0.2 * temp_ids.size();

                // Chấp nhận nếu:
                // 1. Thỏa distance constraint, HOẶC
                // 2. Vi phạm ít hơn best hiện tại
                if (!std::isfinite(vrp.distance) || 
                    new_length <= vrp.distance || 
                    (best.route_idx != -1 && new_length < best.route_length)) {
                    
                    if (balanced_score < best.balanced_score) {
                        best.route_idx = r;
                        best.pos = pos;
                        best.cost_increase = increase;
                        best.route_length = new_length;
                        best.balanced_score = balanced_score;
                    }
                }
            }
        }

        // Insert vào vị trí tốt nhất nếu tìm thấy
        if (best.route_idx != -1) {
            Route& route = sol.routes[best.route_idx];
            route.customer_ids.insert(route.customer_ids.begin() + best.pos, cust_id);
            route.total_demand = calculate_total_demand(route.customer_ids, vrp);
            route.total_cost = calculate_route_length(route.customer_ids, vrp);
            visited[cust_id] = true;
        } else {
            // Bắt buộc tạo route mới
            Route newr;
            newr.customer_ids = { cust_id };
            newr.total_demand = calculate_total_demand(newr.customer_ids, vrp);
            newr.total_cost = calculate_route_length(newr.customer_ids, vrp);
            sol.routes.push_back(newr);
            visited[cust_id] = true;
        }
    }

    // Finalize total cost
    sol.total_cost = 0.0;
    for (auto &r : sol.routes) {
        r.total_demand = calculate_total_demand(r.customer_ids, vrp);
        r.total_cost = calculate_route_length(r.customer_ids, vrp);
        sol.total_cost += r.total_cost;
    }
}


