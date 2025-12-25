#ifndef PDP_INIT_H
#define PDP_INIT_H

#include "pdp_types.h"
#include <vector>
#include <random>

using namespace std;

// === DISTANCE ACCESSOR FUNCTIONS ===

/**
 * @brief Retrieve truck distance between two nodes from Manhattan matrix.
 * @param data PDP instance with distance matrices
 * @param nodeA_id, nodeB_id 0-based node identifiers
 * @return Distance value from truck distance matrix
 */
double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id);

/**
 * @brief Retrieve drone distance between two nodes from Euclidean matrix.
 * @param data PDP instance with distance matrices
 * @param nodeA_id, nodeB_id 0-based node identifiers
 * @return Distance value from drone distance matrix
 */
double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id);

/**
 * @brief Convert a permutation sequence (chromosome) into multi-truck routes.
 * Splits sequence based on separator nodes to assign customers to trucks.
 * @param seq Customer sequence (chromosome representation)
 * @param data PDP instance
 * @return Vector of routes, one per truck
 */
vector<vector<int>> decodeSeq(const vector<int>& seq, const PDPData& data);

// === REPAIR OPERATORS ===

/**
 * @brief Ensure chromosome validity by removing duplicates and adding missing customers.
 * Repairs infeasible chromosomes that may arise from genetic operators.
 * @param[in,out] seq Chromosome to repair
 * @param data PDP instance
 * @param gen Random number generator for stochastic repair decisions
 */
void quickRepairPDP(vector<int>& seq, const PDPData& data, mt19937& gen);

// === INITIALIZATION STRATEGIES ===

/**
 * @brief Create diverse initial population using multiple construction heuristics.
 * Combines Random, Greedy Time, Sweep, and Nearest Neighbor strategies
 * to maximize population diversity for genetic search.
 * @param populationSize Desired number of individuals
 * @param data PDP instance
 * @param runNumber Seed parameter for reproducibility
 * @return Vector of initial chromosome solutions
 */
vector<vector<int>> initStructuredPopulationPDP(int populationSize, const PDPData& data, int runNumber = 1);

/**
 * @brief Generate population with completely random sequences.
 * @param populationSize Number of individuals
 * @param data PDP instance
 * @return Vector of random chromosomes
 */
vector<vector<int>> initRandomPDP(int populationSize, const PDPData& data);

/**
 * @brief Generate population using Sweep algorithm (polar angle ordering).
 * Orders customers by angle from depot center for structured solution diversity.
 * @param populationSize Number of individuals
 * @param data PDP instance
 * @return Vector of sweep-based chromosomes
 */
vector<vector<int>> initSweepPDP(int populationSize, const PDPData& data);

/**
 * @brief Generate population using Greedy Time heuristic.
 * Prioritizes customers with earliest ready times for time-window feasibility.
 * @param populationSize Number of individuals
 * @param data PDP instance
 * @return Vector of greedy-constructed chromosomes
 */
vector<vector<int>> initGreedyTimePDP(int populationSize, const PDPData& data);

/**
 * @brief Generate population using Nearest Neighbor constructive heuristic.
 * Each individual built by repeatedly selecting nearest unserved customer.
 * @param populationSize Number of individuals
 * @param data PDP instance
 * @return Vector of nearest-neighbor chromosomes
 */
vector<vector<int>> initNearestNeighborPDP(int populationSize, const PDPData& data);

#endif
