#ifndef PDP_TABU_H
#define PDP_TABU_H

#include "pdp_types.h"
#include "pdp_cache.h"
#include <vector>
#include <string>
#include <map>

// ============================================================
// === TABU SEARCH — hoạt động trên Chromosome (vector<Gene>)
// === TabuMove dựa trên index (i, j) — không phụ thuộc node_id
// ============================================================

struct TabuMove {
    int type;   // 0: swap | 1: insert | 2: 2-opt | 3: 2-opt* | 4: Or-opt | 5: relocate-pair
    int i, j;
    int param;  // block_size cho Or-opt
    std::string key() const;
};

class TabuSearchPDP {
public:
    TabuSearchPDP(const PDPData& data, int maxIterations, SolutionCache& cache);

    /// Main tabu search: nhận và trả về Chromosome (vector<Gene>)
    Chromosome run(const Chromosome& initialSeq);

private:
    const PDPData& data;
    int maxIterations;
    int tabuTenure;
    std::map<std::string, int> tabuList;
    SolutionCache& cache;

    // Adaptive weights (6 move types)
    std::vector<double> weights;
    std::vector<double> scores;
    std::vector<int>    usedCount;

    int  selectMoveIndex();
    void updateWeights(int segmentLength);

    // Move generation — nhận/trả về Chromosome
    bool findBestSwapMove(
        const Chromosome& cur, double curCost, double bestCost,
        int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta);

    bool findBestInsertMove(
        const Chromosome& cur, double curCost, double bestCost,
        int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta);

    bool findBest2OptMove(
        const Chromosome& cur, double curCost, double bestCost,
        int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta);

    bool findBest2OptStarMove(
        const Chromosome& cur, double curCost, double bestCost,
        int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta);

    bool findBestOrOptMove(
        const Chromosome& cur, double curCost, double bestCost,
        int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta);

    bool findBestRelocatePairMove(
        const Chromosome& cur, double curCost, double bestCost,
        int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta);

    void addTabu (const TabuMove& move, int currentIter);
    bool isTabu  (const TabuMove& move, int currentIter) const;

    /// Apply move: di chuyển nguyên khối Gene (mang theo resupply_vector)
    Chromosome applyMove(const Chromosome& seq, const TabuMove& move) const;
};

// Simple interface
Chromosome tabuSearchPDP(const Chromosome& initialSeq,
                         const PDPData& data,
                         int maxIterations,
                         SolutionCache& cache);

#endif // PDP_TABU_H
