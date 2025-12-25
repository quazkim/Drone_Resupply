#ifndef PDP_UTILS_H
#define PDP_UTILS_H

#include "pdp_types.h"

// Utility functions
double euclideanDistance(double x1, double y1, double x2, double y2);
bool validatePDPConstraints(const PDPSolution& solution, const PDPData& data);
void printSolution(const PDPSolution& solution, const PDPData& data);
double calculateSolutionCost(const PDPSolution& solution, const vector<vector<double>>& distMatrix);

#endif