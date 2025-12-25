#ifndef PDP_FITNESS_H
#define PDP_FITNESS_H

#include "pdp_types.h"
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

#endif // PDP_FITNESS_H
