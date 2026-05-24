#ifndef PDP_GA_H
#define PDP_GA_H

#include "pdp_types.h"
#include <vector>
#include <random>

// ============================================================
// === GA OPERATOR SIGNATURES (Chromosome = vector<Gene>) =====
// ============================================================

// ---------- Crossover operators ----------
// Mỗi hàm trả về Chromosome (vector<Gene>)
Chromosome onePointCrossover(const Chromosome& parent1, const Chromosome& parent2, std::mt19937& gen);
Chromosome orderCrossover(const Chromosome& parent1, const Chromosome& parent2, std::mt19937& gen);
Chromosome pmxCrossover  (const Chromosome& parent1, const Chromosome& parent2, std::mt19937& gen);
Chromosome cycleCrossover(const Chromosome& parent1, const Chromosome& parent2, std::mt19937& gen);

// ---------- Perturbation operators ----------
Chromosome doubleBridgePerturbation(const Chromosome& seq, std::mt19937& gen);
Chromosome ruinRecreatePerturbation(const Chromosome& seq, std::mt19937& gen, double ruinRatio = 0.3);

// ---------- Mutation operators ----------
// Swap, Inversion, Scramble, Insertion, Displacement thao tác trên toàn bộ Gene
// (được phép di chuyển cả node 0 và -1 để GA tự do điều chỉnh ranh giới xe/depot)
void swapMutation       (Chromosome& seq, std::mt19937& gen);
void inversionMutation  (Chromosome& seq, std::mt19937& gen);
void scrambleMutation   (Chromosome& seq, std::mt19937& gen);
void insertionMutation  (Chromosome& seq, std::mt19937& gen);
void displacementMutation(Chromosome& seq, std::mt19937& gen);

// Đột biến mới: di chuyển ngẫu nhiên một gói hàng giữa các resupply_vector
void droneResupplyMutation(Chromosome& seq, const PDPData& data, std::mt19937& gen);

// ---------- Selection ----------
Chromosome tournamentSelection(
    const std::vector<Chromosome>& population,
    const std::vector<double>& fitness,
    int tournamentSize,
    std::mt19937& gen);

// ---------- Repair ----------
// Đảm bảo: node_id>0 xuất hiện đúng 1 lần | node_id==0 đúng 1 lần | node_id==-1 không chạm
void repairSequence(Chromosome& seq, const PDPData& data, std::mt19937& gen);

// ============================================================
// === MAIN GA ALGORITHM ======================================
// ============================================================

PDPSolution geneticAlgorithmPDP(const PDPData& data,
                                int populationSize,
                                int maxGenerations,
                                double mutationRate,
                                int runNumber,
                                bool isSmallScale = false);

#endif // PDP_GA_H
