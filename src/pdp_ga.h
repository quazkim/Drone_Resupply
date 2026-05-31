#ifndef PDP_GA_H
#define PDP_GA_H

#include "pdp_types.h"
#include <ostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>

struct Trip {
    int h = -1;
    std::vector<int> pkgs;
    int origin = 0; // 1=parent1, 2=parent2, 3=both
};

struct RouteIndexInfo {
    int route_id = -1;
    int pos = -1;
};

std::vector<std::vector<int>> extractCustomerRoutes(const SolutionEncoding& sol);
SolutionEncoding initializeEmptyEncodedRoutes(const std::vector<std::vector<int>>& customerRoutes);
std::vector<Trip> extractResupplyTrips(const SolutionEncoding& sol, int originFlag);
std::unordered_map<int, RouteIndexInfo> buildCustomerIndex(const SolutionEncoding& sol);
std::vector<int> getValidPackageSubset(const Trip& trip,
                                       const std::unordered_map<int, RouteIndexInfo>& childIndex,
                                       const std::unordered_set<int>& suppliedAlready);
void repairSolution(SolutionEncoding& child, const PDPData& data);
void repairC2Pairs(std::vector<std::vector<int>>& routes, const PDPData& data);

/**
 * @brief Genetic Algorithm for Truck--Drone Resupply (MD encoding).
 * Returns the best decoded solution found; the encoded solution is in PDPSolution.encoded_routes.
 */
PDPSolution geneticAlgorithmPDP(const PDPData& data,
                                const std::vector<SolutionEncoding>& initialPopulation,
                                int populationSize,
                                int maxGenerations,
                                double mutationRate,
                                int runNumber,
                                bool isSmallScale = false,
                                std::ostream* logStream = nullptr,
                                int logEvery = 1);

#endif // PDP_GA_H

