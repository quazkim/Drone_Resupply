// insertion.cpp
#include "config.h"
#include <vector>
#include <limits>
#include <algorithm>
#include "solution.h" // Route, Solution, VRPInstance
#include "untils.h" // evaluate_route_with_penalty
#include "insertion.h" // insert_customer_best_position, local_post_optimization
#include "vrp_parser.h" // Customer, VRPInstance
#include "solution.h" // Route, Solution



// GENI-like insertion: insert customer into best position in given route
void insert_customer_best_position(Route& route, int customer_id, const VRPInstance& vrp, double alpha, double beta) {
    double best_cost = std::numeric_limits<double>::max();
    size_t best_pos = 0;

    // Try all positions to insert customer
    for (size_t i = 0; i <= route.customer_ids.size(); ++i) {
        std::vector<int> temp = route.customer_ids;
        temp.insert(temp.begin() + i, customer_id);

        Route test;
        test.customer_ids = temp;
        double cost = evaluate_route_with_penalty(test, vrp, alpha, beta); // truyền test

        if (cost < best_cost) {
            best_cost = cost;
            best_pos = i;
        }
    }

    // Insert at best position
    route.customer_ids.insert(route.customer_ids.begin() + best_pos, customer_id);
    evaluate_route_with_penalty(route, vrp, alpha, beta); // truyền route sau khi đã chèn
}

// US-like local post-optimization: remove and reinsert each customer
void local_post_optimization(Route& route, const VRPInstance& vrp, double alpha, double beta) {
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i < route.customer_ids.size(); ++i) {
            int cust = route.customer_ids[i];
            std::vector<int> temp = route.customer_ids;
            temp.erase(temp.begin() + i);

            double best_cost = std::numeric_limits<double>::max();
            size_t best_pos = 0;

            for (size_t j = 0; j < temp.size(); ++j) {
                std::vector<int> trial = temp;
                trial.insert(trial.begin() + j, cust);

                Route test;
                test.customer_ids = trial;
                double cost = evaluate_route_with_penalty(test,  vrp,  alpha, beta);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_pos = j;
                }
            }

            if (best_cost < route.total_cost) {
                route.customer_ids.erase(route.customer_ids.begin() + i);
                route.customer_ids.insert(route.customer_ids.begin() + best_pos, cust);
                evaluate_route_with_penalty(route,  vrp,  alpha, beta);
                improved = true;
            }
        }
    }
}
// Thêm vào tệp insertion.cpp

// GENI-like insertion: insert customer into best position across ALL routes
void insert_customer_any_route(Solution& solution, int customer_id, const VRPInstance& vrp, double alpha, double beta) {
    double min_cost_increase = std::numeric_limits<double>::max();
    int best_route_idx = -1;

    // Lặp qua tất cả các tuyến trong giải pháp để tìm tuyến tốt nhất
    for (size_t i = 0; i < solution.routes.size(); ++i) {
        // Lấy chi phí hiện tại của tuyến
        double cost_before = evaluate_route_with_penalty(solution.routes[i], vrp, alpha, beta);

        // Tạo một bản sao của tuyến để thử nghiệm
        Route temp_route = solution.routes[i];
        
        // DÙNG HÀM CỦA BẠN: Tìm vị trí chèn tốt nhất trong tuyến tạm thời này
        insert_customer_best_position(temp_route, customer_id, vrp, alpha, beta);

        // Lấy chi phí sau khi chèn
        double cost_after = evaluate_route_with_penalty(temp_route, vrp, alpha, beta);

        // Tính toán chi phí tăng thêm
        double cost_increase = cost_after - cost_before;

        if (cost_increase < min_cost_increase) {
            min_cost_increase = cost_increase;
            best_route_idx = i;
        }
    }

    // Nếu đã tìm thấy một tuyến phù hợp
    if (best_route_idx != -1) {
        // Chèn khách hàng vào tuyến tốt nhất đã tìm thấy (lần này là chèn thật)
        insert_customer_best_position(solution.routes[best_route_idx], customer_id, vrp, alpha, beta);
    } else if (!solution.routes.empty()) {
        // Fallback: nếu không có gì tốt hơn, chèn vào tuyến đầu tiên
        insert_customer_best_position(solution.routes[0], customer_id, vrp, alpha, beta);
    }
    // (Nếu không có tuyến nào, khách hàng sẽ bị bỏ qua - cần xử lý nếu có trường hợp đó)
}