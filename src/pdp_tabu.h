#ifndef PDP_TABU_H
#define PDP_TABU_H

#include "pdp_types.h"

/**
 * @brief Tabu Search local search refinement (Intensification phase).
 */
class TabuSearchPDP {
public:
    TabuSearchPDP(const PDPData& data, int maxIterations, double globalBestFitness, int tabuTenure);
    SolutionEncoding run(const SolutionEncoding& initial);

private:
    const PDPData& data;
    int maxIterations;
    double globalBestFitness;
    int tabuTenure;
};

SolutionEncoding tabuSearchPDP(const SolutionEncoding& initial,
                               const PDPData& data,
                               int maxIterations,
                               double globalBestFitness,
                               int tabuTenure);

#endif // PDP_TABU_H

