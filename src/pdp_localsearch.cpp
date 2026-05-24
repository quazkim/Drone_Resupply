#include "pdp_localsearch.h"

IntegratedLocalSearch::IntegratedLocalSearch(const PDPData& data, int maxIterations)
    : data(data), maxIterations(maxIterations) {}

PDPSolution IntegratedLocalSearch::run(PDPSolution initialSolution) {
    (void)data;
    (void)maxIterations;
    return initialSolution;
}
