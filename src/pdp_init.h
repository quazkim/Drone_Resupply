#ifndef PDP_INIT_H
#define PDP_INIT_H

#include "pdp_types.h"
#include <random>
#include <vector>

/**
 * @brief Build an initial population of encoded solutions (MD encoding).
 * Each individual is SolutionEncoding = [Route_1..Route_n].
 */
std::vector<SolutionEncoding> initStructuredPopulationPDP(int populationSize,
                                                         const PDPData& data,
                                                         int runNumber = 1);

/**
 * @brief Loading evaluator (TẦNG 3 - fast path).
 *
 * Given a pure routing (ordered customer list per truck), greedily build a
 * full encoding with depot loads + drone resupply markers. Deterministic:
 * the same routing always maps to the same encoding/cost (fixed internal RNG),
 * so it can be used as a stable fitness for ALNS/TS acceptance decisions.
 */
SolutionEncoding buildEncodingForRouting(const std::vector<std::vector<int>>& customerRoutes,
                                         const PDPData& data);

#endif // PDP_INIT_H
