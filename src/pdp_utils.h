#ifndef PDP_UTILS_H
#define PDP_UTILS_H

#include "pdp_types.h"

// Distance helper (used by reader)
double euclideanDistance(double x1, double y1, double x2, double y2);

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
