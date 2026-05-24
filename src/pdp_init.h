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

#endif // PDP_INIT_H
