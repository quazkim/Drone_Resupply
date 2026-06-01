#ifndef PDP_UTILS_H
#define PDP_UTILS_H

#include "pdp_types.h"

// Distance helper (used by reader)
double euclideanDistance(double x1, double y1, double x2, double y2);

/**
 * @brief Compute a lower bound on C_max for the given instance.
 *
 * For each D-type customer i, computes the earliest possible makespan
 * assuming the truck goes directly from depot to i (no detour), using
 * whichever is faster: drone resupply or truck-carry from depot.
 *
 * LB = max_i { min(drone_option_i, truck_carry_i) }
 *   drone_option_i  = max(truck_dist(d,i), r_i + drone_dist(d,i)) + resupplyTime + svc + truck_dist(i,d)
 *   truck_carry_i   = r_i + truck_dist(d,i) + svc + truck_dist(i,d)
 */
double computeLowerBound(const PDPData& data);

/**
 * @brief Validate encoding constraints (MD specs):
 *  - Each customer appears exactly once as a service node.
 *  - For each i[P], every package j in P must satisfy position(j) >= position(i) in the same route.
 */
bool validateEncodingConstraints(const SolutionEncoding& encoded, const PDPData& data);

/// Print an encoded solution in the MD notation.
void printEncodedSolution(const SolutionEncoding& encoded);

/// Print full decoded solution (summary + encoded routes).
void printSolution(const PDPSolution& solution, const PDPData& data);

#endif // PDP_UTILS_H
