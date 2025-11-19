#pragma once
#include <vector>
#include <string>

// Forward declaration
struct VRPInstance;

struct Route {
    std::vector<int> customer_ids;
    int total_demand = 0;
    double total_cost = 0.0;
};

struct Solution {
    std::vector<Route> routes;
    double total_cost = 0.0;
};

// Function declarations
double evaluate_route_with_penalty(Route& route, const VRPInstance& vrp, double alpha, double beta);
double evaluate_solution(Solution& sol, const VRPInstance& vrp, double anpha, double beta);
void print_solution(const Solution& sol, const VRPInstance& vrp);
void write_solution_to_file(const Solution& sol, const std::string& filename, const std::string& vrp_name, double time, int vehicle_count, bool penalty_mode, double alpha, double beta, int iter_stop, int tabu_tenure, const VRPInstance& vrp ); 