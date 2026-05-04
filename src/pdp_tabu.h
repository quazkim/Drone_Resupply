#ifndef PDP_TABU_H
#define PDP_TABU_H

#include "pdp_types.h"
#include "pdp_cache.h"
#include <vector>
#include <string>
#include <map>

// ============ TABU SEARCH WITH ADAPTIVE WEIGHTS & 6 MOVES ============

struct TabuMove {
    int type;  // 0: swap, 1: insert, 2: 2-opt, 3: 2-opt*, 4: Or-opt, 5: relocate-pair
    int i, j;
    int param;  // For block_size (Or-opt) or other parameters
    std::string key() const;
};

class TabuSearchPDP {
public:
    TabuSearchPDP(const PDPData& data, int maxIterations, SolutionCache& cache);
    
    // Main tabu search
    Chromosome run(const Chromosome& initial);
    
private:
    const PDPData& data;
    int maxIterations;
    int tabuTenure;
    std::map<std::string, int> tabuList;
    SolutionCache& cache;  // Reference to shared solution cache
    
    // Adaptive weights for move selection
    std::vector<double> weights;
    std::vector<double> scores;
    std::vector<int> usedCount;
    
    // Move selection
    int selectMoveIndex();
    void updateWeights(int segmentLength);
    
    // Move generation functions (6 types)
    bool findBestSwapMove(const Chromosome& current, double currentCost,
                         double bestCost, int iter, TabuMove& bestMove,
                         Chromosome& bestCandidate, double& bestDelta);
    
    bool findBestInsertMove(const Chromosome& current, double currentCost,
                           double bestCost, int iter, TabuMove& bestMove,
                           Chromosome& bestCandidate, double& bestDelta);
    
    bool findBest2OptMove(const Chromosome& current, double currentCost,
                         double bestCost, int iter, TabuMove& bestMove,
                         Chromosome& bestCandidate, double& bestDelta);
    
    bool findBest2OptStarMove(const Chromosome& current, double currentCost,
                             double bestCost, int iter, TabuMove& bestMove,
                             Chromosome& bestCandidate, double& bestDelta);
    
    bool findBestOrOptMove(const Chromosome& current, double currentCost,
                          double bestCost, int iter, TabuMove& bestMove,
                          Chromosome& bestCandidate, double& bestDelta);
    
    bool findBestRelocatePairMove(const Chromosome& current, double currentCost,
                                 double bestCost, int iter, TabuMove& bestMove,
                                 Chromosome& bestCandidate, double& bestDelta);
    
    // Tabu management
    void addTabu(const TabuMove& move, int currentIter);
    bool isTabu(const TabuMove& move, int currentIter) const;
    
    // Helper: apply move and get candidate
    Chromosome applyMove(const Chromosome& chromo, const TabuMove& move) const;
};

// Simple interface function
Chromosome tabuSearchPDP(const Chromosome& initial,
                         const PDPData& data,
                         int maxIterations,
                         SolutionCache& cache);

#endif // PDP_TABU_H
