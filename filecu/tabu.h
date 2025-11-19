#ifndef TABU_H
#define TABU_H
#include "config.h"

#include <unordered_map>
#include <string>
#include "solution.h"
#include "vrp_parser.h"
#include "insertion.h"
#include "untils.h"
#include <unordered_set>

// Cấu trúc lưu thông tin 1 phép di chuyển (move)
struct Move {
    int type; // 0: 1-0, 1: 1-1, 2: 2-opt cross
    int from_route, to_route;
    int customer_id, customer_id2;
    int i, j;
    int block_size;
    Move(int t, int fr, int tr, int cid, int cid2 = -1, int ii = -1, int jj = -1, int bs = 1)
        : type(t), from_route(fr), to_route(tr), customer_id(cid), customer_id2(cid2), i(ii), j(jj), block_size(bs) {}
};
class TabuSearch {
private:
    double alpha; 
    double beta;  
    const VRPInstance& vrp;
    int max_iterations;
    int tabu_tenure;
    std::unordered_map<std::string, int> tabu_list;
    int last_iter;
public:
   TabuSearch(const VRPInstance& instance, int max_iters);
    Solution run(const Solution& initial_solution);
    int get_last_iter() const { return last_iter; }
    int get_tabu_tenure() const { return tabu_tenure; }
    
private:
    std::string move_key(const Move& move) const;
    bool is_tabu(const Move& move, int current_iter) const;
    void add_tabu(const Move& move, int current_iter);
    void apply_move(Solution& sol, const Move& move, int move_index);
    void apply_move_10(Solution& solution, const Move& move);
    void apply_move_11(Solution& solution, int route1, int idx1, int route2, int idx2);
    void apply_move_2opt_cross(Solution& solution, int route1, int i, int route2, int j);
    int select_move_index(const std::vector<double>& weights) const;
    void log_tabu_list(int current_iter) const;
    void find_best_move_10(const Solution& current_solution, Solution& best_solution, double& best_delta, Move& best_move, bool&move_found, int iter);
    void find_best_move_11(const Solution& current_solution, Solution& best_solution, double& best_delta, Move& best_move, bool& move_found, int iter);
    void find_best_move_2opt_cross(const Solution& current_solution, Solution& best_solution, double& best_delta, Move& best_move, bool& move_found, int iter);
    double apply_move_10_with_delta(Solution &solution, const Move &move,bool commit);
    double apply_move_11_with_delta(Solution &solution, const Move &move, bool commit);
    double apply_move_2opt_cross_with_delta(Solution &solution, const Move &move, bool commit);
    bool try_repair_solution(Solution& solution, double alpha, double beta);
    void find_best_move_2opt_star(const Solution &current_solution, Solution &best_solution, double &best_delta, Move &best_move, bool &move_found, int iter);
    void find_best_move_or_opt(const Solution &current_solution, Solution &best_solution, double &best_delta, Move &best_move, bool &move_found, int iter);
    void find_best_move_relocate_pair(const Solution &current_solution, Solution &best_solution, double &best_delta, Move &best_move, bool &move_found, int iter);

    double apply_move_2opt_star_with_delta(Solution &solution, const Move &move, bool commit);
    double apply_move_or_opt_with_delta(Solution &solution, const Move &move, bool commit);
    double apply_move_relocate_pair_with_delta(Solution &solution, const Move &move, bool commit);
};
#endif // TABU_H

