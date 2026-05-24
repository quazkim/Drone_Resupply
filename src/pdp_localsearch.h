#ifndef PDP_LOCALSEARCH_H
#define PDP_LOCALSEARCH_H

#include "pdp_types.h"

/**
 * @brief Local search placeholder.
 *
 * After refactor to the MD encoding, the previous integrated LS (truck+drone) is disabled.
 * This stub keeps build compatibility.
 */
class IntegratedLocalSearch {
public:
    IntegratedLocalSearch(const PDPData& data, int maxIterations = 500);

    PDPSolution run(PDPSolution initialSolution);

private:
    const PDPData& data;
    int maxIterations;
};

#endif // PDP_LOCALSEARCH_H
