/**
 * @file pdp_tabu.cpp
 * @brief Tabu Search hoạt động trên Chromosome (vector<Gene>).
 *
 * applyMove() di chuyển nguyên khối Gene (mang theo resupply_vector).
 * TabuMove dựa trên chỉ số (i, j) — không phụ thuộc node_id → logic không đổi.
 *
 * Bridge sang evaluateWithCache: chrToIds() trích xuất node_id để dùng cache cũ.
 */

#include "pdp_tabu.h"
#include "pdp_fitness.h"
#include "pdp_cache.h"
#include <algorithm>
#include <random>
#include <iostream>
#include <limits>
#include <cmath>
#include <numeric>

using namespace std;

// ============================================================
// === BRIDGE UTILITY =========================================
// ============================================================

/// Trích xuất vector<int> node_id từ Chromosome để dùng với evaluateWithCache
static vector<int> chrToIds(const Chromosome& seq) {
    vector<int> ids;
    ids.reserve(seq.size());
    for (const Gene& g : seq) ids.push_back(g.node_id);
    return ids;
}

/// Đánh giá Chromosome thông qua evaluateWithCache (dùng bridge)
static PDPSolution evalChromosome(const Chromosome& seq,
                                  const PDPData& data,
                                  SolutionCache& cache) {
    return evaluateWithCache(chrToIds(seq), data, cache);
}

// ============================================================
// === TABUMOVE ===============================================
// ============================================================

string TabuMove::key() const {
    return to_string(type) + "_" + to_string(i) + "_"
         + to_string(j)    + "_" + to_string(param);
}

// ============================================================
// === CONSTRUCTOR & WEIGHT MANAGEMENT ========================
// ============================================================

TabuSearchPDP::TabuSearchPDP(const PDPData& data, int maxIterations, SolutionCache& cache)
    : data(data), maxIterations(maxIterations), cache(cache)
{
    int n = data.numCustomers;
    tabuTenure = (int)(0.2 * n) + (rand() % 11);

    weights  = vector<double>(6, 1.0);
    scores   = vector<double>(6, 0.0);
    usedCount = vector<int>(6, 0);
}

int TabuSearchPDP::selectMoveIndex() {
    double total = 0.0;
    for (double w : weights) total += w;
    double rnd = ((double)rand() / RAND_MAX) * total;
    double acc = 0.0;
    for (int i = 0; i < (int)weights.size(); ++i) {
        acc += weights[i];
        if (rnd <= acc) return i;
    }
    return (int)weights.size() - 1;
}

void TabuSearchPDP::updateWeights(int /*segmentLength*/) {
    const double delta4 = 0.5;
    for (int i = 0; i < 6; ++i) {
        if (usedCount[i] > 0)
            weights[i] = (1 - delta4) * weights[i] + delta4 * (scores[i] / usedCount[i]);
        scores[i]    = 0.0;
        usedCount[i] = 0;
    }
}

void TabuSearchPDP::addTabu(const TabuMove& move, int currentIter) {
    tabuList[move.key()] = currentIter + tabuTenure;
}

bool TabuSearchPDP::isTabu(const TabuMove& move, int currentIter) const {
    auto it = tabuList.find(move.key());
    return (it != tabuList.end() && currentIter < it->second);
}

// ============================================================
// === APPLY MOVE — di chuyển nguyên khối Gene ================
// ============================================================

Chromosome TabuSearchPDP::applyMove(const Chromosome& seq, const TabuMove& move) const {
    Chromosome result = seq;

    switch (move.type) {
        case 0: // Swap
            swap(result[move.i], result[move.j]);
            break;

        case 1: { // Insert: di chuyển Gene[i] đến vị trí j
            Gene tmp = result[move.i];
            result.erase(result.begin() + move.i);
            int ins = move.j;
            if (ins > move.i) ins--;       // đã erase 1 phần tử
            ins = max(0, min(ins, (int)result.size()));
            result.insert(result.begin() + ins, tmp);
            break;
        }

        case 2:   // 2-opt: reverse segment [i, j]
        case 3: { // 2-opt*: same reversal logic
            if (move.i < move.j)
                reverse(result.begin() + move.i, result.begin() + move.j + 1);
            break;
        }

        case 4: { // Or-opt: relocate block of size param
            int blockSize = move.param;
            if (move.i + blockSize <= (int)result.size()) {
                Chromosome block(result.begin() + move.i,
                                 result.begin() + move.i + blockSize);
                result.erase(result.begin() + move.i,
                             result.begin() + move.i + blockSize);
                int ins = move.j > move.i ? move.j - blockSize : move.j;
                ins = max(0, min(ins, (int)result.size()));
                result.insert(result.begin() + ins, block.begin(), block.end());
            }
            break;
        }

        case 5: { // Relocate pair: move Gene[i] and Gene[i+1]
            if (move.i + 1 < (int)result.size()) {
                Gene g1 = result[move.i];
                Gene g2 = result[move.i + 1];
                result.erase(result.begin() + move.i,
                             result.begin() + move.i + 2);
                int ins = move.j > move.i ? move.j - 2 : move.j;
                ins = max(0, min(ins, (int)result.size()));
                result.insert(result.begin() + ins, g2);
                result.insert(result.begin() + ins, g1);
            }
            break;
        }
    }
    return result;
}

// ============================================================
// === MOVE GENERATION FUNCTIONS ==============================
// ============================================================

bool TabuSearchPDP::findBestSwapMove(
    const Chromosome& cur, double curCost, double bestCost,
    int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta)
{
    bool found = false;
    for (int i = 0; i < (int)cur.size(); ++i) {
        for (int j = i + 1; j < (int)cur.size(); ++j) {
            TabuMove move{0, i, j, 0};
            bool tabu = isTabu(move, iter);
            Chromosome cand = applyMove(cur, move);
            PDPSolution sol  = evalChromosome(cand, data, cache);
            double cost      = sol.totalCost + sol.totalPenalty;
            double delta     = cost - curCost;
            if (!tabu || cost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta; bestMove = move; bestCand = cand; found = true;
                }
            }
        }
    }
    return found;
}

bool TabuSearchPDP::findBestInsertMove(
    const Chromosome& cur, double curCost, double bestCost,
    int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta)
{
    bool found = false;
    int n = (int)cur.size();
    int maxTrials = min(50, n * n / 4);

    mt19937 gen(random_device{}());
    uniform_int_distribution<int> dist(0, n - 1);

    for (int t = 0; t < maxTrials; ++t) {
        int i = dist(gen), j = dist(gen);
        if (i == j) continue;
        TabuMove move{1, i, j, 0};
        bool tabu = isTabu(move, iter);
        Chromosome cand = applyMove(cur, move);
        PDPSolution sol  = evalChromosome(cand, data, cache);
        double cost      = sol.totalCost + sol.totalPenalty;
        double delta     = cost - curCost;
        if (!tabu || cost < bestCost) {
            if (delta < bestDelta) {
                bestDelta = delta; bestMove = move; bestCand = cand; found = true;
            }
        }
    }
    return found;
}

bool TabuSearchPDP::findBest2OptMove(
    const Chromosome& cur, double curCost, double bestCost,
    int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta)
{
    bool found = false;
    int n = (int)cur.size();
    for (int i = 0; i < n - 1; ++i) {
        for (int j = i + 1; j < min(n, i + 20); ++j) {
            TabuMove move{2, i, j, 0};
            bool tabu = isTabu(move, iter);
            Chromosome cand = applyMove(cur, move);
            PDPSolution sol  = evalChromosome(cand, data, cache);
            double cost      = sol.totalCost + sol.totalPenalty;
            double delta     = cost - curCost;
            if (!tabu || cost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta; bestMove = move; bestCand = cand; found = true;
                }
            }
        }
    }
    return found;
}

bool TabuSearchPDP::findBest2OptStarMove(
    const Chromosome& cur, double curCost, double bestCost,
    int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta)
{
    bool found = false;
    int n = (int)cur.size();
    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 2; j < min(n, i + 15); ++j) {
            TabuMove move{3, i, j, 0};
            bool tabu = isTabu(move, iter);
            Chromosome cand = applyMove(cur, move);
            PDPSolution sol  = evalChromosome(cand, data, cache);
            double cost      = sol.totalCost + sol.totalPenalty;
            double delta     = cost - curCost;
            if (!tabu || cost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta; bestMove = move; bestCand = cand; found = true;
                }
            }
        }
    }
    return found;
}

bool TabuSearchPDP::findBestOrOptMove(
    const Chromosome& cur, double curCost, double bestCost,
    int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta)
{
    bool found = false;
    int n = (int)cur.size();
    for (int bs = 1; bs <= 3; ++bs) {
        if (n < bs + 1) continue;
        for (int i = 0; i <= n - bs; ++i) {
            for (int j = 0; j <= n - bs; ++j) {
                if (abs(i - j) < bs) continue;
                TabuMove move{4, i, j, bs};
                bool tabu = isTabu(move, iter);
                Chromosome cand = applyMove(cur, move);
                PDPSolution sol  = evalChromosome(cand, data, cache);
                double cost      = sol.totalCost + sol.totalPenalty;
                double delta     = cost - curCost;
                if (!tabu || cost < bestCost) {
                    if (delta < bestDelta) {
                        bestDelta = delta; bestMove = move; bestCand = cand; found = true;
                    }
                }
            }
        }
    }
    return found;
}

bool TabuSearchPDP::findBestRelocatePairMove(
    const Chromosome& cur, double curCost, double bestCost,
    int iter, TabuMove& bestMove, Chromosome& bestCand, double& bestDelta)
{
    bool found = false;
    int n = (int)cur.size();
    if (n < 3) return false;
    for (int i = 0; i < n - 1; ++i) {
        for (int j = 0; j <= n - 2; ++j) {
            if (abs(i - j) < 2) continue;
            TabuMove move{5, i, j, 0};
            bool tabu = isTabu(move, iter);
            Chromosome cand = applyMove(cur, move);
            PDPSolution sol  = evalChromosome(cand, data, cache);
            double cost      = sol.totalCost + sol.totalPenalty;
            double delta     = cost - curCost;
            if (!tabu || cost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta; bestMove = move; bestCand = cand; found = true;
                }
            }
        }
    }
    return found;
}

// ============================================================
// === MAIN TABU SEARCH =======================================
// ============================================================

Chromosome TabuSearchPDP::run(const Chromosome& initialSeq) {
    Chromosome currentSeq = initialSeq;
    Chromosome bestSeq    = initialSeq;

    PDPSolution currentSol = evalChromosome(currentSeq, data, cache);
    PDPSolution bestSol    = currentSol;
    double currentCost     = currentSol.totalCost + currentSol.totalPenalty;
    double bestCost        = currentCost;

    const double delta1 = 0.5, delta2 = 0.3, delta3 = 0.2;
    int segmentLength  = 100;
    int noImprovement  = 0;
    const int maxNoImp = min(5000, maxIterations / 2);

    for (int iter = 0; iter < maxIterations; ++iter) {
        int moveIndex = selectMoveIndex();

        TabuMove   bestMove{-1, -1, -1, 0};
        Chromosome bestCand;
        double     bestDelta = numeric_limits<double>::infinity();
        bool       found     = false;

        switch (moveIndex) {
            case 0: found = findBestSwapMove        (currentSeq, currentCost, bestCost, iter, bestMove, bestCand, bestDelta); break;
            case 1: found = findBestInsertMove       (currentSeq, currentCost, bestCost, iter, bestMove, bestCand, bestDelta); break;
            case 2: found = findBest2OptMove         (currentSeq, currentCost, bestCost, iter, bestMove, bestCand, bestDelta); break;
            case 3: found = findBest2OptStarMove     (currentSeq, currentCost, bestCost, iter, bestMove, bestCand, bestDelta); break;
            case 4: found = findBestOrOptMove        (currentSeq, currentCost, bestCost, iter, bestMove, bestCand, bestDelta); break;
            case 5: found = findBestRelocatePairMove (currentSeq, currentCost, bestCost, iter, bestMove, bestCand, bestDelta); break;
        }

        if (found && bestMove.type != -1) {
            double prevCost = currentCost;
            currentSeq = bestCand;

            PDPSolution newSol = evalChromosome(currentSeq, data, cache);
            currentCost = newSol.totalCost + newSol.totalPenalty;
            currentSol  = newSol;

            if (currentCost < bestCost) {
                scores[moveIndex] += delta1;
                bestCost = currentCost; bestSeq = currentSeq; bestSol = newSol;
                noImprovement = 0;
            } else if (currentCost < prevCost) {
                scores[moveIndex] += delta2;
                noImprovement++;
            } else {
                scores[moveIndex] += delta3;
                noImprovement++;
            }
            usedCount[moveIndex]++;
            addTabu(bestMove, iter);
        } else {
            noImprovement++;
        }

        if ((iter + 1) % segmentLength == 0)
            updateWeights(segmentLength);

        if (noImprovement >= maxNoImp) break;

        if (iter % 50 == 0 && iter > 0)
            cout << "  Tabu iter " << iter
                 << ": best=" << bestCost
                 << " current=" << currentCost << "\n";
    }

    return bestSeq;
}

// ============================================================
// === SIMPLE INTERFACE =======================================
// ============================================================

Chromosome tabuSearchPDP(const Chromosome& initialSeq,
                         const PDPData& data,
                         int maxIterations,
                         SolutionCache& cache)
{
    TabuSearchPDP tabu(data, maxIterations, cache);
    return tabu.run(initialSeq);
}
