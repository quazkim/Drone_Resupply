#ifndef PDP_ALNS_H
#define PDP_ALNS_H

#include "pdp_types.h"
#include <random>
#include <vector>

// ============================================================
// ALNS escape mechanism for TS-ALNS-LSP
// 3 destroy operators + 2 repair operators
// Adaptive weights updated after each call (rho=0.1 decay)
// ============================================================

struct ALNSWeights {
    // Destroy operators: 0=random segment, 1=worst removal, 2=related removal
    double destroy_w[3] = {1.0, 1.0, 1.0};
    // Repair operators:  0=greedy insert, 1=regret-2 insert
    double repair_w[2]  = {1.0, 1.0};

    // Select operator by roulette wheel
    int selectDestroy(std::mt19937& rng) const;
    int selectRepair(std::mt19937& rng) const;

    // Update weights after an iteration
    // score: 0=no improvement, 1=improvement, 2=new global best
    void updateDestroy(int op, int score, double rho = 0.1);
    void updateRepair(int op, int score, double rho = 0.1);
};

// Destroy operators (return new VRPSolution with some customers removed)
// removed_customers: customers pulled out (must be re-inserted by repair)
struct DestroyResult {
    VRPSolution partial;              // routes with customers removed
    std::vector<int> removed;         // customers that need reinsertion
};

DestroyResult destroyRandomSegment(const PDPData& data,
                                   const VRPSolution& vrp,
                                   int segment_len,  // number to remove
                                   std::mt19937& rng);

DestroyResult destroyWorstRemoval(const PDPData& data,
                                  const VRPSolution& vrp,
                                  int n_remove,
                                  std::mt19937& rng);

DestroyResult destroyRelatedRemoval(const PDPData& data,
                                    const VRPSolution& vrp,
                                    int n_remove,
                                    std::mt19937& rng);

// Repair operators (reinsert removed customers into routes)
// Must maintain C2 pair integrity (P before DL, same truck)
VRPSolution repairGreedyInsert(const PDPData& data,
                                const DestroyResult& dr,
                                std::mt19937& rng);

VRPSolution repairRegret2Insert(const PDPData& data,
                                 const DestroyResult& dr,
                                 std::mt19937& rng);

// Apply one ALNS iteration: destroy + repair
// Returns new VRPSolution candidate
VRPSolution alnsIteration(const PDPData& data,
                           const VRPSolution& vrp,
                           ALNSWeights& weights,
                           int destroy_op, int repair_op,
                           int n_remove,
                           std::mt19937& rng);

#endif
