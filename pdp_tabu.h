#ifndef PDP_TABU_H
#define PDP_TABU_H

#include "pdp_types.h"
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
    TabuSearchPDP(const PDPData& data, int maxIterations);
    
    // Main tabu search
    std::vector<int> run(const std::vector<int>& initialSeq);
    
private:
    const PDPData& data;
    int maxIterations;
    int tabuTenure;
    std::map<std::string, int> tabuList;
    
    // Adaptive weights for move selection
    std::vector<double> weights;
    std::vector<double> scores;
    std::vector<int> usedCount;
    
    // Move selection
    int selectMoveIndex();
    void updateWeights(int segmentLength);
    
    // Move generation functions (6 types)
    bool findBestSwapMove(const std::vector<int>& currentSeq, double currentCost, 
                         double bestCost, int iter, TabuMove& bestMove, 
                         std::vector<int>& bestCandidate, double& bestDelta);
    
    bool findBestInsertMove(const std::vector<int>& currentSeq, double currentCost,
                           double bestCost, int iter, TabuMove& bestMove,
                           std::vector<int>& bestCandidate, double& bestDelta);
    
    bool findBest2OptMove(const std::vector<int>& currentSeq, double currentCost,
                         double bestCost, int iter, TabuMove& bestMove,
                         std::vector<int>& bestCandidate, double& bestDelta);
    
    bool findBest2OptStarMove(const std::vector<int>& currentSeq, double currentCost,
                             double bestCost, int iter, TabuMove& bestMove,
                             std::vector<int>& bestCandidate, double& bestDelta);
    
    bool findBestOrOptMove(const std::vector<int>& currentSeq, double currentCost,
                          double bestCost, int iter, TabuMove& bestMove,
                          std::vector<int>& bestCandidate, double& bestDelta);
    
    bool findBestRelocatePairMove(const std::vector<int>& currentSeq, double currentCost,
                                 double bestCost, int iter, TabuMove& bestMove,
                                 std::vector<int>& bestCandidate, double& bestDelta);
    
    // Tabu management
    void addTabu(const TabuMove& move, int currentIter);
    bool isTabu(const TabuMove& move, int currentIter) const;
    
    // Helper: apply move and get candidate
    std::vector<int> applyMove(const std::vector<int>& seq, const TabuMove& move) const;
};

// Simple interface function
std::vector<int> tabuSearchPDP(const std::vector<int>& initialSeq,
                               const PDPData& data,
                               int maxIterations);

#endif // PDP_TABU_H
