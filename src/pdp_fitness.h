#ifndef PDP_FITNESS_H
#define PDP_FITNESS_H

#include "pdp_types.h"
#include "pdp_cache.h"
#include <vector>

/**
 * @brief Core evaluation function: decode chromosome to solution and compute fitness.
 * 
 * Process:
 * 1. Interpret sequence as customer ordering
 * 2. Greedily assign each customer to the truck with earliest availability
 * 3. For each truck route, optimize drone resupply missions
 * 4. Compute total makespan (C_max) and time-window violation penalties
 * 
 * @param seq Chromosome - permutation sequence representing customer service order
 * @param data PDP instance with all problem parameters and constraints
 * 
 * @return PDPSolution containing:
 *   - routes: Assignment of customers to trucks
 *   - totalCost: Makespan objective value C_max
 *   - totalPenalty: Sum of constraint violation penalties
 *   - isFeasible: true if all time windows and endurance constraints satisfied
 *   - truck_details: Complete execution timeline for each truck
 *   - resupply_events: All drone missions executed
 */
PDPSolution decodeAndEvaluate(const std::vector<int>& seq, const PDPData& data);

/**
 * @brief Wrapper function for decodeAndEvaluate with solution caching.
 * 
 * Checks cache first (O(1) lookup). If sequence is found, returns cached solution.
 * Otherwise, calls decodeAndEvaluate, stores result in cache, and returns.
 * 
 * Usage:
 *   SolutionCache cache;
 *   PDPSolution sol = evaluateWithCache(population[i], data, cache);
 * 
 * @param seq Customer sequence (cache key)
 * @param data PDP instance
 * @param cache Reference to SolutionCache object maintaining state across evals
 * 
 * @return PDPSolution with all details (truck_details, resupply_events accessible for post-processing)
 */
PDPSolution evaluateWithCache(
    const std::vector<int>& seq,
    const PDPData& data,
    SolutionCache& cache
);

// Assignment encoding for Local Search post-processing
struct AssignmentEncoding {
    std::vector<int> truck_assign;
    std::vector<int> drone_assign;
    std::vector<int> break_bit;
};

AssignmentEncoding initFromSolution(
    const std::vector<int>& seq,
    const PDPSolution& sol,
    const PDPData& data
);

PDPSolution runAssignmentLS(
    const std::vector<int>& seq,
    AssignmentEncoding& enc,
    const PDPData& data,
    int max_iter
);

// Convenience: run assignment LS on a decoded solution
inline PDPSolution assignmentLSPostProcess(const std::vector<int>& seq, const PDPData& data) {
    PDPSolution sol = decodeAndEvaluate(seq, data);
    AssignmentEncoding enc = initFromSolution(seq, sol, data);
    PDPSolution ls_sol = runAssignmentLS(seq, enc, data, 200);
    double ls_cost = ls_sol.totalCost + ls_sol.totalPenalty * 1000.0;
    double cur_cost = sol.totalCost + sol.totalPenalty * 1000.0;
    return (ls_cost < cur_cost - 0.01) ? ls_sol : sol;
}

#endif // PDP_FITNESS_H
