#ifndef PDP_VALIDATION_H
#define PDP_VALIDATION_H

#include "pdp_types.h"

/**
 * @brief Validate a PDP solution
 * @param solution The solution to validate
 * @param data Problem data
 * @param verbose Print detailed output
 * @return true if solution is valid
 */
bool validateSolution(const PDPSolution& solution, const PDPData& data, bool verbose = true);

/**
 * @brief Print solution summary
 * @param solution The solution
 * @param costBeforeLS Cost before local search
 * @param costAfterLS Cost after local search
 */
void printSolutionSummary(const PDPSolution& solution, double costBeforeLS, double costAfterLS);

#endif // PDP_VALIDATION_H
