#include "config.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <algorithm>
#include <random>
#include <iostream>
#include <numeric>
#include "solution.h"
#include "insertion.h"
#include "untils.h"
#include "vrp_parser.h"
#include "tabu.h"
using namespace std;
TabuSearch::TabuSearch(const VRPInstance &instance, int max_iters)
    : vrp(instance), max_iterations(max_iters), tabu_tenure(0) {
    int n = (int)vrp.customers.size() - 1; // loại depot nếu có
    double k = 0.2;  // ví dụ 10% số khách
    int r = 10;      // biên độ random thêm
    
    tabu_tenure = (int)(k * n) + (rand() % (r + 1));  
    }

Solution TabuSearch::run(const Solution &initial_solution)
{
    this->alpha = 0.1;
    this->beta = 0.1;
    Solution current_solution = initial_solution;
    Solution best_solution = initial_solution;          // có thể vi phạm
    Solution best_feasible_solution = initial_solution; // chỉ lưu nghiệm hợp lệ

    evaluate_solution(current_solution, vrp, alpha, beta);
    evaluate_solution(best_solution, vrp, alpha, beta);

    if (is_feasible_solution(best_solution, vrp))
    {
        best_feasible_solution = best_solution;
    }
    else
    {
        best_feasible_solution.total_cost = std::numeric_limits<double>::infinity();
    }

    int no_improvement = 0;
    const int max_no_improvement = 30000;
    const int num_moves = 6;
    const double delta1 = 0.5, delta2 = 0.3, delta3 = 0.2, delta4 = 0.5; 
    const double delta = 0.3; // tốc độ điều chỉnh
    const double alpha_min = 0.1, alpha_max = 1000.0;
    const double beta_min = 0.1, beta_max = 1000.0;
    std::vector<double> weights(num_moves, 1.0);
    std::vector<double> scores(num_moves, 0.0);
    std::vector<int> used_count(num_moves, 0);
    int segment_length = 100;
    int iter;
    for ( iter = 0; iter < max_iterations; ++iter)
    {
        if (iter % 500 == 0) {
        cout << "Iter " << iter 
         << "  current=" << current_solution.total_cost 
         << "  best=" << best_solution.total_cost 
         << "  feasible=" << best_feasible_solution.total_cost 
         << "  alpha=" << alpha 
         << "  beta=" << beta ;
         double delta_capacity = 0;
         for (size_t r2 = 0; r2 < current_solution.routes.size(); ++r2) delta_capacity +=  max(0,(current_solution.routes[r2].total_demand-vrp.capacity));
         cout <<" delta capacity "<<delta_capacity; 
          double delta_distance = 0;
         for (size_t r2 = 0; r2 < current_solution.routes.size(); ++r2) delta_distance +=  max(0.0, (current_solution.routes[r2].total_cost-vrp.distance));
         cout <<" delta distance "<<delta_distance<<endl; 
        log_tabu_list(iter);

        }

        Move best_move(0, -1, -1, -1, -1, -1, -1);
        double best_delta = std::numeric_limits<double>::max();
        bool move_found = false;
        int move_index = select_move_index(weights);

        // Tìm move tốt nhất
        if (move_index == 0)
    find_best_move_10(current_solution, best_solution, best_delta, best_move, move_found, iter);
else if (move_index == 1)
    find_best_move_11(current_solution, best_solution, best_delta, best_move, move_found, iter);
else if (move_index == 2)
    find_best_move_2opt_cross(current_solution, best_solution, best_delta, best_move, move_found, iter);
else if (move_index == 3)
    find_best_move_2opt_star(current_solution, best_solution, best_delta, best_move, move_found, iter);
else if (move_index == 4)
    find_best_move_or_opt(current_solution, best_solution, best_delta, best_move, move_found, iter);
else if (move_index == 5)
    find_best_move_relocate_pair(current_solution, best_solution, best_delta, best_move, move_found, iter);
        if (move_found)
        {
            double previous_cost = current_solution.total_cost; // lưu chi phí trước move
            apply_move(current_solution, best_move, move_index);
            evaluate_solution(current_solution, vrp, alpha, beta);

            // Cập nhật điểm cho adaptive weight
            if (current_solution.total_cost < best_solution.total_cost)
            {
                scores[move_index] += delta1; // move tạo ra best solution mới
            }
            else if (current_solution.total_cost < previous_cost)
            {
                scores[move_index] += delta2; // move cải thiện current solution nhưng không phải best
            }
            else
            {
                scores[move_index] += delta3; // move tệ hơn current solution nhưng vẫn chấp nhận
            }

            used_count[move_index]++; // số lần dùng move

            if (current_solution.total_cost < best_solution.total_cost)
            {
                best_solution = current_solution;
                no_improvement = 0;
                Solution repaired_solution = best_solution; // Tạo một bản sao để sửa
                if (try_repair_solution(repaired_solution, alpha, beta)) {
                    // Nếu sửa thành công và nó tốt hơn best_feasible hiện tại
                    if (repaired_solution.total_cost < best_feasible_solution.total_cost) {
                    //cout << "!!! New best feasible solution found via repair: " 
                     //cout<< repaired_solution.total_cost << " !!!" << endl;
                     best_feasible_solution = repaired_solution;
                     }
                }
            }
            else
            {
                no_improvement++;
            }

            //  Update best_feasible nếu nghiệm hợp lệ và tốt hơn
            if (is_feasible_solution(current_solution, vrp) &&
                current_solution.total_cost < best_feasible_solution.total_cost)
            {
                best_feasible_solution = current_solution;
            }

            add_tabu(best_move, iter);
        }
        else
        {
            no_improvement++;
        }

        if (no_improvement >= max_no_improvement)
            break;

        // Cập nhật weights sau mỗi segment
        if ((iter + 1) % segment_length == 0)
        {
            for (int i = 0; i < num_moves; ++i)
            {
                if (used_count[i] > 0)
                    weights[i] = (1 - delta4) * weights[i] + delta4 * (scores[i] / used_count[i]);
                scores[i] = 0.0;
                used_count[i] = 0;
            }
        }

        // Điều chỉnh alpha, beta
        if (!is_feasible_solution(current_solution, vrp)&&USE_PENALTY)
        {
            if (violate_capacity(current_solution, vrp))
                alpha = std::min(alpha * (1 + delta), alpha_max);
            else
                alpha = std::max(alpha * (1 - delta), alpha_min);

            if (violate_length(current_solution, vrp))
                beta = std::min(beta * (1 + delta), beta_max);
            else
                beta = std::max(beta * (1 - delta), beta_min);
        }
        else
        {
            alpha = std::max(alpha * (1 - delta), alpha_min);
            beta = std::max(beta * (1 - delta), beta_min);
        }
    }
    last_iter = iter;   // lưu số vòng lặp dừng

     return best_feasible_solution;
    
}

std::string TabuSearch::move_key(const Move &move) const
{
    if (move.type == 0) // 1-0
        return "10-" + std::to_string(move.customer_id) + "-" +
               std::to_string(move.from_route) + "-" + std::to_string(move.to_route);
    if (move.type == 1) // 1-1
        return "11-" + std::to_string(move.customer_id) + "-" +
               std::to_string(move.from_route) + "-" +
               std::to_string(move.customer_id2) + "-" +
               std::to_string(move.to_route);
    if (move.type == 2) // 2-opt cross
        return "2opt-" + std::to_string(move.from_route) + "-" +
               std::to_string(move.i) + "-" +
               std::to_string(move.to_route) + "-" +
               std::to_string(move.j);
    if (move.type == 3) // 2-opt*
        return "2opt*-" + std::to_string(move.from_route) + "-" + std::to_string(move.i) + "-" + std::to_string(move.j);
    if (move.type == 4) // Or-opt
        return "Or-" + std::to_string(move.from_route) + "-" + std::to_string(move.i) + "-" + std::to_string(move.j) + "-" + std::to_string(move.block_size);
    if (move.type == 5) // Relocate Pair
        return "RelPair-" + std::to_string(move.customer_id) + "-" + std::to_string(move.from_route) + "-" + std::to_string(move.to_route);
    return "";
}

void TabuSearch::add_tabu(const Move &move, int current_iter)
{
    std::string key = move_key(move);
    tabu_list[key] = current_iter + tabu_tenure;
}

bool TabuSearch::is_tabu(const Move &move, int current_iter) const
{
    std::string key = move_key(move);
    auto it = tabu_list.find(key);
    if (it != tabu_list.end() && current_iter < it->second)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void TabuSearch::apply_move(Solution &solution, const Move &move, int move_index)
{
    if (move_index == 0)
    {
        apply_move_10_with_delta(solution, move, true); // commit thật
    }
    else if (move_index == 1)
    {
        apply_move_11_with_delta(solution, move, true);
    }
    else if (move_index == 2)
    {
        apply_move_2opt_cross_with_delta(solution, move, true);
    }
    else if (move_index == 3)
        apply_move_2opt_star_with_delta(solution, move, true);
    else if (move_index == 4)
        apply_move_or_opt_with_delta(solution, move, true);
    else if (move_index == 5)
        apply_move_relocate_pair_with_delta(solution, move, true);
}

int TabuSearch::select_move_index(const std::vector<double> &weights) const
{
    double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
    double rnd = ((double)rand() / RAND_MAX) * total_weight;
    double acc = 0.0;
    for (int i = 0; i < weights.size(); ++i)
    {
        acc += weights[i];
        if (rnd <= acc)
        {
            return i;
        }
    }
    return weights.size() - 1; // fallback
}
void TabuSearch::log_tabu_list(int current_iter) const
{
    std::cout << "[Tabu List at iter " << current_iter << "]" << std::endl;
    for (const auto &p : tabu_list)
    {
        const std::string &key = p.first;
        int expire = p.second;
        if (current_iter < expire)
        {
            std::cout << "  " << key << " expires at " << expire << std::endl;
        }
    }
}
void TabuSearch::find_best_move_10(const Solution &current_solution,
                                   Solution &best_solution,
                                   double &best_delta,
                                   Move &best_move,
                                   bool &move_found,
                                   int iter)
{
    best_delta = std::numeric_limits<double>::max();
    move_found = false;

    for (size_t r1 = 0; r1 < current_solution.routes.size(); ++r1)
    {
        for (size_t i = 0; i < current_solution.routes[r1].customer_ids.size() ; ++i)
        {
            int cust = current_solution.routes[r1].customer_ids[i];
            for (size_t r2 = 0; r2 < current_solution.routes.size(); ++r2)
            {
                if (r1 == r2)
                    continue;

                Move move(0, (int)r1, (int)r2, cust, -1, -1, -1);

                double new_total = apply_move_10_with_delta(
                    const_cast<Solution &>(current_solution),
                    move,
                    false);

                if (!USE_PENALTY && new_total >= 1e9)
                    continue;

                double delta = new_total - current_solution.total_cost;

                if (is_tabu(move, iter) && new_total >= best_solution.total_cost)
                    continue;

                if (delta < best_delta)
                {
                    best_delta = delta;
                    best_move = move;
                    move_found = true;
                }
            }
        }
    }
}

void TabuSearch::find_best_move_11(const Solution &current_solution,
                                   Solution &best_solution,
                                   double &best_delta,
                                   Move &best_move,
                                   bool &move_found,
                                   int iter)
{
    best_delta = std::numeric_limits<double>::max();
    move_found = false;

    for (size_t r1 = 0; r1 < current_solution.routes.size(); ++r1)
    {
        for (size_t i = 0; i < current_solution.routes[r1].customer_ids.size() ; ++i)
        {
            for (size_t r2 = r1 + 1; r2 < current_solution.routes.size(); ++r2)
            {
                for (size_t j = 0; j < current_solution.routes[r2].customer_ids.size(); ++j)
                {
                    Move move(
                        1,
                        static_cast<int>(r1), static_cast<int>(r2),
                        current_solution.routes[r1].customer_ids[i],
                        current_solution.routes[r2].customer_ids[j],
                        static_cast<int>(i), static_cast<int>(j));

                    // Dùng with_delta để thử move, không commit
                    double new_total = apply_move_11_with_delta(
                        const_cast<Solution &>(current_solution),
                        move,
                        false);

                    // Nếu No Penalty và nghiệm không hợp lệ → bỏ qua
                    if (!USE_PENALTY && new_total >= 1e9)
                        continue;

                    double delta = new_total - current_solution.total_cost;

                    if (is_tabu(move, iter) && new_total >= best_solution.total_cost)
                        continue;

                    if (delta < best_delta)
                    {
                        best_delta = delta;
                        best_move = move;
                        move_found = true;
                    }
                }
            }
        }
    }
}

void TabuSearch::find_best_move_2opt_cross(const Solution &current_solution,
                                           Solution &best_solution,
                                           double &best_delta,
                                           Move &best_move,
                                           bool &move_found,
                                           int iter)
{
    best_delta = std::numeric_limits<double>::max();
    move_found = false;

    for (size_t r1 = 0; r1 < current_solution.routes.size(); ++r1)
    {
        for (size_t r2 = r1 + 1; r2 < current_solution.routes.size(); ++r2)
        {
            for (size_t i = 0; i < current_solution.routes[r1].customer_ids.size() ; ++i)
            {
                for (size_t j = 0; j < current_solution.routes[r2].customer_ids.size() ; ++j)
                {
                    Move move(
                        2,
                        static_cast<int>(r1), static_cast<int>(r2),
                        -1, -1,
                        static_cast<int>(i), static_cast<int>(j));

                    // Thử move bằng delta evaluation (không commit)
                    double new_total = apply_move_2opt_cross_with_delta(
                        const_cast<Solution &>(current_solution),
                        move,
                        false);

                    // Nếu chạy chế độ hard (no penalty) và nghiệm vi phạm → bỏ qua
                    if (!USE_PENALTY && new_total >= 1e9)
                        continue;

                    double delta = new_total - current_solution.total_cost;

                    if (is_tabu(move, iter) && new_total >= best_solution.total_cost)
                        continue;

                    if (delta < best_delta)
                    {
                        best_delta = delta;
                        best_move = move;
                        move_found = true;
                    }
                }
            }
        }
    }
}
void TabuSearch::find_best_move_2opt_star(const Solution &current_solution,
                                          Solution &best_solution,
                                          double &best_delta,
                                          Move &best_move,
                                          bool &move_found,
                                          int iter) {
    best_delta = std::numeric_limits<double>::max();
    move_found = false;

    for (size_t r = 0; r < current_solution.routes.size(); ++r) {
        if (current_solution.routes[r].customer_ids.size() < 2) continue;

        for (size_t i = 0; i < current_solution.routes[r].customer_ids.size() - 1; ++i) {
            for (size_t j = i + 1; j < current_solution.routes[r].customer_ids.size(); ++j) {
                Move move(3, r, r, -1, -1, i, j);

                double new_total = apply_move_2opt_star_with_delta(const_cast<Solution &>(current_solution), move, false);
                if (!USE_PENALTY && new_total >= 1e9) continue;
                
                double delta = new_total - current_solution.total_cost;
                if (is_tabu(move, iter) && new_total >= best_solution.total_cost) continue;

                if (delta < best_delta) {
                    best_delta = delta;
                    best_move = move;
                    move_found = true;
                }
            }
        }
    }
}
void TabuSearch::find_best_move_or_opt(const Solution &current_solution,
                                       Solution &best_solution,
                                       double &best_delta,
                                       Move &best_move,
                                       bool &move_found,
                                       int iter) {
    best_delta = std::numeric_limits<double>::max();
    move_found = false;

    for (size_t r = 0; r < current_solution.routes.size(); ++r) {
        for (int block_size = 2; block_size >= 1; --block_size) { // Try block size 2, then 1
            if (current_solution.routes[r].customer_ids.size() < block_size + 1) continue;

            for (size_t i = 0; i <= current_solution.routes[r].customer_ids.size() - block_size; ++i) {
                // j is the new insertion position for the block
                for (size_t j = 0; j <= current_solution.routes[r].customer_ids.size() - block_size; ++j) {
                    if (i == j) continue;

                    Move move(4, r, r, -1, -1, i, j, block_size);
                    
                    double new_total = apply_move_or_opt_with_delta(const_cast<Solution &>(current_solution), move, false);
                    if (!USE_PENALTY && new_total >= 1e9) continue;

                    double delta = new_total - current_solution.total_cost;
                    if (is_tabu(move, iter) && new_total >= best_solution.total_cost) continue;

                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move = move;
                        move_found = true;
                    }
                }
            }
        }
    }
}
void TabuSearch::find_best_move_relocate_pair(const Solution &current_solution,
                                              Solution &best_solution,
                                              double &best_delta,
                                              Move &best_move,
                                              bool &move_found,
                                              int iter) {
    best_delta = std::numeric_limits<double>::max();
    move_found = false;

    for (size_t r1 = 0; r1 < current_solution.routes.size(); ++r1) {
        if (current_solution.routes[r1].customer_ids.size() < 2) continue;

        for (size_t r2 = 0; r2 < current_solution.routes.size(); ++r2) {
            if (r1 == r2) continue;

            for (size_t i = 0; i < current_solution.routes[r1].customer_ids.size() - 1; ++i) {
                int cust1 = current_solution.routes[r1].customer_ids[i];
                int cust2 = current_solution.routes[r1].customer_ids[i+1];

                for (size_t j = 0; j <= current_solution.routes[r2].customer_ids.size(); ++j) {
                     Move move(5, r1, r2, cust1, cust2, i, j);

                    double new_total = apply_move_relocate_pair_with_delta(const_cast<Solution &>(current_solution), move, false);
                    if (!USE_PENALTY && new_total >= 1e9) continue;

                    double delta = new_total - current_solution.total_cost;
                    if (is_tabu(move, iter) && new_total >= best_solution.total_cost) continue;

                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move = move;
                        move_found = true;
                    }
                }
            }
        }
    }
}

// Hàm thử move 1-0 (relocate) với delta evaluation
double TabuSearch::apply_move_10_with_delta(Solution &solution,
                                            const Move &move,
                                            bool commit)
{
    auto &from_route = solution.routes[move.from_route];
    auto &to_route = solution.routes[move.to_route];

    double old_cost = from_route.total_cost + to_route.total_cost;

    // Copy tạm
    Route temp_from = from_route;
    Route temp_to = to_route;

    // Xóa khách hàng khỏi route nguồn
    auto it = std::find(temp_from.customer_ids.begin(),
                        temp_from.customer_ids.end(),
                        move.customer_id);
    if (it != temp_from.customer_ids.end())
        temp_from.customer_ids.erase(it);

    // Chèn vào vị trí tốt nhất trong route đích
    insert_customer_best_position(temp_to, move.customer_id, vrp, alpha, beta);

    // Tính lại chi phí
    double new_cost_from = evaluate_route_with_penalty(temp_from, vrp, alpha, beta);
    double new_cost_to = evaluate_route_with_penalty(temp_to, vrp, alpha, beta);

    double new_total = solution.total_cost - old_cost + new_cost_from + new_cost_to;

    if (commit)
    {
        from_route = temp_from;
        to_route = temp_to;

        //  Tối ưu cục bộ từng route
        // local_post_optimization(from_route, vrp, alpha, beta);
        // local_post_optimization(to_route, vrp, alpha, beta);

        //  Cập nhật lại chi phí chính xác sau tối ưu
        from_route.total_cost = evaluate_route_with_penalty(from_route, vrp, alpha, beta);
        to_route.total_cost = evaluate_route_with_penalty(to_route, vrp, alpha, beta);
        solution.total_cost = solution.total_cost - old_cost + from_route.total_cost + to_route.total_cost;
    }
    return new_total;
}

double TabuSearch::apply_move_11_with_delta(Solution &solution,
                                            const Move &move,
                                            bool commit)
{
    auto &r1 = solution.routes[move.from_route];
    auto &r2 = solution.routes[move.to_route];

    // Kiểm tra chỉ số hợp lệ (không swap depot)


    double old_cost = r1.total_cost + r2.total_cost;

    // Copy tạm để thử nghiệm
    Route temp_r1 = r1;
    Route temp_r2 = r2;

    // Hoán đổi khách hàng
    std::swap(temp_r1.customer_ids[move.i], temp_r2.customer_ids[move.j]);
   
    // Nếu commit thì tối ưu cục bộ 2 route
    if (commit)
    {
        // local_post_optimization(temp_r1, vrp, alpha, beta);
        // local_post_optimization(temp_r2, vrp, alpha, beta);
    }

    // Tính lại cost cho 2 route
    double new_cost_r1 = evaluate_route_with_penalty(temp_r1, vrp, alpha, beta);
    double new_cost_r2 = evaluate_route_with_penalty(temp_r2, vrp, alpha, beta);

    double new_total = solution.total_cost - old_cost + new_cost_r1 + new_cost_r2;

    // Nếu commit thì cập nhật solution
    if (commit)
    {
        r1 = temp_r1;
        r2 = temp_r2;
        r1.total_cost = new_cost_r1;
        r2.total_cost = new_cost_r2;
        solution.total_cost = new_total;
    }

    return new_total;
}

double TabuSearch::apply_move_2opt_cross_with_delta(Solution &solution,
                                                    const Move &move,
                                                    bool commit)
{
    auto &r1 = solution.routes[move.from_route];
    auto &r2 = solution.routes[move.to_route];


    double old_cost = r1.total_cost + r2.total_cost;

    Route temp_r1 = r1;
    Route temp_r2 = r2;

    // Cắt đuôi từ i và j
    std::vector<int> tail1(temp_r1.customer_ids.begin() + move.i+1, temp_r1.customer_ids.end());
    std::vector<int> tail2(temp_r2.customer_ids.begin() + move.j+1, temp_r2.customer_ids.end());

    temp_r1.customer_ids.erase(temp_r1.customer_ids.begin() + move.i+1, temp_r1.customer_ids.end() );
    temp_r2.customer_ids.erase(temp_r2.customer_ids.begin() + move.j+1, temp_r2.customer_ids.end() );

    // Nối chéo
    temp_r1.customer_ids.insert(temp_r1.customer_ids.end() , tail2.begin(), tail2.end());
    temp_r2.customer_ids.insert(temp_r2.customer_ids.end() , tail1.begin(), tail1.end());

    double new_cost_r1 = evaluate_route_with_penalty(temp_r1, vrp, alpha, beta);
    double new_cost_r2 = evaluate_route_with_penalty(temp_r2, vrp, alpha, beta);

    double new_total = solution.total_cost - old_cost + new_cost_r1 + new_cost_r2;

    if (commit)
    {
        r1 = temp_r1;
        r2 = temp_r2;

        // Gọi tối ưu cục bộ để cải thiện (giống bản gốc)
        // local_post_optimization(r1, vrp, alpha, beta);
        // local_post_optimization(r2, vrp, alpha, beta);

        // Cập nhật chi phí mới
        r1.total_cost = evaluate_route_with_penalty(r1, vrp, alpha, beta);
        r2.total_cost = evaluate_route_with_penalty(r2, vrp, alpha, beta);
        solution.total_cost = solution.total_cost - old_cost + r1.total_cost + r2.total_cost;
    }

    return new_total;
}
double TabuSearch::apply_move_2opt_star_with_delta(Solution &solution, const Move &move, bool commit) {
    auto &route = solution.routes[move.from_route];
    double old_route_cost = evaluate_route_with_penalty(route, vrp, alpha, beta);

    Route temp_route = route;
    std::reverse(temp_route.customer_ids.begin() + move.i, temp_route.customer_ids.begin() + move.j + 1);

    double new_route_cost = evaluate_route_with_penalty(temp_route, vrp, alpha, beta);
    double new_total = solution.total_cost - old_route_cost + new_route_cost;

    if (commit) {
        solution.routes[move.from_route] = temp_route;
        solution.routes[move.from_route].total_cost = new_route_cost;
        solution.total_cost = new_total;
    }
    
    return new_total;
}
double TabuSearch::apply_move_or_opt_with_delta(Solution &solution, const Move &move, bool commit) {
    auto &route = solution.routes[move.from_route];
    double old_route_cost = evaluate_route_with_penalty(route, vrp, alpha, beta);

    Route temp_route = route;
    
    std::vector<int> block;
    auto start_it = temp_route.customer_ids.begin() + move.i;
    block.assign(start_it, start_it + move.block_size);
    temp_route.customer_ids.erase(start_it, start_it + move.block_size);

    auto insert_it = temp_route.customer_ids.begin() + move.j;
    temp_route.customer_ids.insert(insert_it, block.begin(), block.end());

    double new_route_cost = evaluate_route_with_penalty(temp_route, vrp, alpha, beta);
    double new_total = solution.total_cost - old_route_cost + new_route_cost;

    if (commit) {
        solution.routes[move.from_route] = temp_route;
        solution.routes[move.from_route].total_cost = new_route_cost;
        solution.total_cost = new_total;
    }
    
    return new_total;
}
double TabuSearch::apply_move_relocate_pair_with_delta(Solution &solution, const Move &move, bool commit) {
    auto &from_route_orig = solution.routes[move.from_route];
    auto &to_route_orig = solution.routes[move.to_route];
    double old_cost_from = evaluate_route_with_penalty(from_route_orig, vrp, alpha, beta);
    double old_cost_to = evaluate_route_with_penalty(to_route_orig, vrp, alpha, beta);

    Route temp_from = from_route_orig;
    Route temp_to = to_route_orig;

    std::vector<int> pair;
    auto start_it = temp_from.customer_ids.begin() + move.i;
    pair.assign(start_it, start_it + 2);
    temp_from.customer_ids.erase(start_it, start_it + 2);

    auto insert_it = temp_to.customer_ids.begin() + move.j;
    temp_to.customer_ids.insert(insert_it, pair.begin(), pair.end());

    double new_cost_from = evaluate_route_with_penalty(temp_from, vrp, alpha, beta);
    double new_cost_to = evaluate_route_with_penalty(temp_to, vrp, alpha, beta);
    double new_total = solution.total_cost - (old_cost_from + old_cost_to) + (new_cost_from + new_cost_to);

    if (commit) {
        solution.routes[move.from_route] = temp_from;
        solution.routes[move.to_route] = temp_to;
        solution.routes[move.from_route].total_cost = new_cost_from;
        solution.routes[move.to_route].total_cost = new_cost_to;
        solution.total_cost = new_total;
    }

    return new_total;
}
// Thay thế hàm try_repair_solution cũ trong tabu.cpp bằng hàm này

bool TabuSearch::try_repair_solution(Solution& solution, double, double) { // Bỏ qua alpha, beta đầu vào
    
    // *** SỬA ĐỔI: Dùng hằng số phạt CỰC CAO cho việc sửa chữa ***
    const double REPAIR_ALPHA = 10000.0;
    const double REPAIR_BETA = 10000.0;

    std::vector<int> customers_to_reinsert;
    std::vector<size_t> modified_route_indices; 
    
    // Bước 1: Phá hủy
    for (size_t i = 0; i < solution.routes.size(); ++i) {
        // Kiểm tra vi phạm dựa trên giá trị THỰC (chứ không phải chi phí có phạt)
        bool is_over_capacity = solution.routes[i].total_demand > vrp.capacity;
        bool is_over_length = solution.routes[i].total_cost > vrp.distance; // Giả định .total_cost là chi phí thực

        if (is_over_capacity || is_over_length) {
            customers_to_reinsert.insert(customers_to_reinsert.end(), 
                                         solution.routes[i].customer_ids.begin(), 
                                         solution.routes[i].customer_ids.end());
            solution.routes[i].customer_ids.clear();
            modified_route_indices.push_back(i); 
        }
    }

    if (customers_to_reinsert.empty()) {
        return is_feasible_solution(solution, vrp); 
    }

    // Đánh giá lại các tuyến rỗng (dùng penalty cao, mặc dù nó = 0)
    for (size_t idx : modified_route_indices) {
        evaluate_route_with_penalty(solution.routes[idx], vrp, REPAIR_ALPHA, REPAIR_BETA);
    }

    // Bước 2: Tái tạo
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(customers_to_reinsert.begin(), customers_to_reinsert.end(), g);
    
    for (int cust_id : customers_to_reinsert) {
        // Truyền các giá trị PHẠT CAO vào hàm chèn
        insert_customer_any_route(solution, cust_id, vrp, REPAIR_ALPHA, REPAIR_BETA);
    }
    
    // Đánh giá lại toàn bộ giải pháp (dùng penalty cao)
    evaluate_solution(solution, vrp, REPAIR_ALPHA, REPAIR_BETA);

    // Bước 3: Tăng cường (Ý tưởng của bạn vẫn rất tốt)
    // Bây giờ giải pháp có khả năng cao là HỢP LỆ
    if (is_feasible_solution(solution, vrp)) {
        for (auto& route : solution.routes) {
            if (!route.customer_ids.empty()) {
                // Chạy "dọn dẹp" (vẫn dùng penalty cao để đảm bảo)
                local_post_optimization(route, vrp, REPAIR_ALPHA, REPAIR_BETA);
            }
        }
        // Đánh giá lại lần cuối cùng (dùng penalty cao)
        evaluate_solution(solution, vrp, REPAIR_ALPHA, REPAIR_BETA);
    }

    // Trả về kết quả cuối cùng
    return is_feasible_solution(solution, vrp);
}

// Implementation of find_best_move functions
void TabuSearch::find_best_move_10(const Solution& current_solution, Solution& best_solution, double& best_delta, Move& best_move, bool& move_found, int iter) {
    move_found = false;
    best_delta = std::numeric_limits<double>::max();
    
    // Simple implementation: try removing one customer from each route
    for (size_t r = 0; r < current_solution.routes.size(); ++r) {
        const auto& route = current_solution.routes[r];
        if (route.customer_ids.size() <= 1) continue; // Skip routes with 1 or 0 customers
        
        for (size_t i = 0; i < route.customer_ids.size(); ++i) {
            int customer = route.customer_ids[i];
            
            // Create move
            Move move(0, r, -1, customer);
            
            // Check if tabu
            if (is_tabu(move, iter)) continue;
            
            // Calculate delta (simplified)
            double delta = 100.0; // Placeholder delta
            
            if (delta < best_delta) {
                best_delta = delta;
                best_move = move;
                move_found = true;
            }
        }
    }
}

void TabuSearch::find_best_move_11(const Solution& current_solution, Solution& best_solution, double& best_delta, Move& best_move, bool& move_found, int iter) {
    move_found = false;
    best_delta = std::numeric_limits<double>::max();
    
    // Simple implementation: try moving customer between routes
    for (size_t r1 = 0; r1 < current_solution.routes.size(); ++r1) {
        const auto& route1 = current_solution.routes[r1];
        if (route1.customer_ids.empty()) continue;
        
        for (size_t r2 = 0; r2 < current_solution.routes.size(); ++r2) {
            if (r1 == r2) continue;
            
            for (size_t i = 0; i < route1.customer_ids.size(); ++i) {
                int customer = route1.customer_ids[i];
                
                // Create move
                Move move(1, r1, r2, customer);
                
                // Check if tabu
                if (is_tabu(move, iter)) continue;
                
                // Calculate delta (simplified)
                double delta = 50.0; // Placeholder delta
                
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move = move;
                    move_found = true;
                }
            }
        }
    }
}

void TabuSearch::find_best_move_2opt_cross(const Solution& current_solution, Solution& best_solution, double& best_delta, Move& best_move, bool& move_found, int iter) {
    move_found = false;
    best_delta = std::numeric_limits<double>::max();
    
    // Simple implementation: try 2-opt cross between routes
    for (size_t r1 = 0; r1 < current_solution.routes.size(); ++r1) {
        const auto& route1 = current_solution.routes[r1];
        if (route1.customer_ids.size() < 2) continue;
        
        for (size_t r2 = 0; r2 < current_solution.routes.size(); ++r2) {
            if (r1 == r2) continue;
            const auto& route2 = current_solution.routes[r2];
            if (route2.customer_ids.size() < 2) continue;
            
            // Try simple cross
            Move move(2, r1, r2, -1, -1, 1, 1);
            
            // Check if tabu
            if (is_tabu(move, iter)) continue;
            
            // Calculate delta (simplified)
            double delta = 75.0; // Placeholder delta
            
            if (delta < best_delta) {
                best_delta = delta;
                best_move = move;
                move_found = true;
            }
        }
    }
}