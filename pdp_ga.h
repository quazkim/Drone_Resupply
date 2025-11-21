#ifndef PDP_GA_H
#define PDP_GA_H

#include "pdp_types.h"
#include <vector>
#include <random>

// ============ GA OPERATORS ============

// Crossover operators
std::vector<int> orderCrossover(const std::vector<int>& parent1, const std::vector<int>& parent2, std::mt19937& gen);
std::vector<int> pmxCrossover(const std::vector<int>& parent1, const std::vector<int>& parent2, std::mt19937& gen);
std::vector<int> cycleCrossover(const std::vector<int>& parent1, const std::vector<int>& parent2, std::mt19937& gen);

// Mutation operators
void swapMutation(std::vector<int>& seq, std::mt19937& gen);
void inversionMutation(std::vector<int>& seq, std::mt19937& gen);
void scrambleMutation(std::vector<int>& seq, std::mt19937& gen);

// Selection
std::vector<int> tournamentSelection(const std::vector<std::vector<int>>& population,
                                     const std::vector<double>& fitness,
                                     int tournamentSize,
                                     std::mt19937& gen);

// Repair để đảm bảo tất cả khách hàng có trong sequence
void repairSequence(std::vector<int>& seq, const PDPData& data, std::mt19937& gen);

// ============ MAIN GA ALGORITHM ============

PDPSolution geneticAlgorithmPDP(const PDPData& data,
                               int populationSize,
                               int maxGenerations,
                               double mutationRate,
                               int runNumber);

#endif // PDP_GA_H
