#ifndef PDP_ALNS_H
#define PDP_ALNS_H

#include "pdp_types.h"
#include <ostream>

/**
 * @brief Adaptive Large Neighborhood Search (ALNS) master over ROUTING,
 *        with periodic Tabu Search intensification and a greedy loading
 *        evaluator (TẦNG 1 + TẦNG 2 + TẦNG 3-fast).
 *
 * Architecture:
 *   TẦNG 1 - ALNS : destroy (random/worst/Shaw/route) + repair (greedy/regret-2),
 *                   adaptive operator weights, simulated-annealing acceptance.
 *                   C2 pickup-delivery handled as atomic pairs (precedence kept).
 *   TẦNG 2 - TS   : relocate/swap/or-opt focused on LastTruck, run on the
 *                   incumbent periodically / on each new global best.
 *   TẦNG 3 - LOAD : buildEncodingForRouting (greedy) + decode_solution gives
 *                   the true C_max. EXACT MILP loading is a future plug-in here.
 *
 * @return best decoded solution found (encoded routes in PDPSolution.encoded_routes).
 */
PDPSolution alnsSearchPDP(const PDPData& data,
                          const SolutionEncoding& initial,
                          int maxIterations,
                          double timeLimitSeconds,
                          int runNumber,
                          std::ostream* logStream = nullptr);

#endif // PDP_ALNS_H
