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

// ============ TABU MOVE ============

string TabuMove::key() const {
    return to_string(type) + "_" + to_string(i) + "_" + to_string(j) + "_" + to_string(param);
}

// ============ TABU SEARCH CLASS ============

TabuSearchPDP::TabuSearchPDP(const PDPData& data, int maxIterations, SolutionCache& cache)
    : data(data), maxIterations(maxIterations), cache(cache) {
    int n = data.numCustomers;
    double k = 0.2;  // 20% of customers
    int r = 10;      // random range
    
    tabuTenure = (int)(k * n) + (rand() % (r + 1));
    
    // Initialize 6 moves with equal weights
    weights = vector<double>(6, 1.0);
    scores = vector<double>(6, 0.0);
    usedCount = vector<int>(6, 0);
}

int TabuSearchPDP::selectMoveIndex() {
    double totalWeight = accumulate(weights.begin(), weights.end(), 0.0);
    double rnd = ((double)rand() / RAND_MAX) * totalWeight;
    double acc = 0.0;
    
    for (int i = 0; i < (int)weights.size(); ++i) {
        acc += weights[i];
        if (rnd <= acc) {
            return i;
        }
    }
    return weights.size() - 1;
}

void TabuSearchPDP::updateWeights(int segmentLength) {
    const double delta4 = 0.5;
    
    for (int i = 0; i < 6; ++i) {
        if (usedCount[i] > 0) {
            weights[i] = (1 - delta4) * weights[i] + delta4 * (scores[i] / usedCount[i]);
        }
        scores[i] = 0.0;
        usedCount[i] = 0;
    }
}

void TabuSearchPDP::addTabu(const TabuMove& move, int currentIter) {
    tabuList[move.key()] = currentIter + tabuTenure;
}

bool TabuSearchPDP::isTabu(const TabuMove& move, int currentIter) const {
    auto it = tabuList.find(move.key());
    if (it != tabuList.end() && currentIter < it->second) {
        return true;
    }
    return false;
}

Chromosome TabuSearchPDP::applyMove(const Chromosome& chromo, const TabuMove& move) const {
    Chromosome result = chromo;
    int n = (int)result.sequence.size();
    if ((int)result.truck_assign.size() != n) result.truck_assign.assign(n, 0);
    if ((int)result.drone_assign.size() != n) result.drone_assign.assign(n, 0);
    if ((int)result.break_bit.size() != n) result.break_bit.assign(n, 1);

    auto eraseInsert = [&](vector<int>& v, int from, int to) {
        int temp = v[from];
        v.erase(v.begin() + from);
        v.insert(v.begin() + to, temp);
    };

    auto reverseSeg = [&](vector<int>& v, int i, int j) {
        reverse(v.begin() + i, v.begin() + j + 1);
    };

    auto relocateBlock = [&](vector<int>& v, int start, int blockSize, int target) {
        if (start + blockSize > (int)v.size()) return;
        vector<int> block(v.begin() + start, v.begin() + start + blockSize);
        v.erase(v.begin() + start, v.begin() + start + blockSize);
        int insertPos = target;
        if (target > start) insertPos -= blockSize;
        insertPos = max(0, min(insertPos, (int)v.size()));
        v.insert(v.begin() + insertPos, block.begin(), block.end());
    };

    auto relocatePair = [&](vector<int>& v, int start, int target) {
        if (start + 1 >= (int)v.size()) return;
        int a = v[start];
        int b = v[start + 1];
        v.erase(v.begin() + start, v.begin() + start + 2);
        int insertPos = target;
        if (target > start) insertPos -= 2;
        insertPos = max(0, min(insertPos, (int)v.size()));
        v.insert(v.begin() + insertPos, a);
        v.insert(v.begin() + insertPos + 1, b);
    };
    
    switch (move.type) {
        case 0: // Swap
            swap(result.sequence[move.i], result.sequence[move.j]);
            swap(result.truck_assign[move.i], result.truck_assign[move.j]);
            swap(result.drone_assign[move.i], result.drone_assign[move.j]);
            swap(result.break_bit[move.i], result.break_bit[move.j]);
            break;
            
        case 1: { // Insert
            eraseInsert(result.sequence, move.i, move.j);
            eraseInsert(result.truck_assign, move.i, move.j);
            eraseInsert(result.drone_assign, move.i, move.j);
            eraseInsert(result.break_bit, move.i, move.j);
            break;
        }
        
        case 2: { // 2-opt (reverse segment)
            reverseSeg(result.sequence, move.i, move.j);
            reverseSeg(result.truck_assign, move.i, move.j);
            reverseSeg(result.drone_assign, move.i, move.j);
            reverseSeg(result.break_bit, move.i, move.j);
            break;
        }
        
        case 3: { // 2-opt* (reverse segment between i and j)
            if (move.i < move.j) {
                reverseSeg(result.sequence, move.i, move.j);
                reverseSeg(result.truck_assign, move.i, move.j);
                reverseSeg(result.drone_assign, move.i, move.j);
                reverseSeg(result.break_bit, move.i, move.j);
            }
            break;
        }
        
        case 4: { // Or-opt (relocate block)
            int blockSize = move.param;
            relocateBlock(result.sequence, move.i, blockSize, move.j);
            relocateBlock(result.truck_assign, move.i, blockSize, move.j);
            relocateBlock(result.drone_assign, move.i, blockSize, move.j);
            relocateBlock(result.break_bit, move.i, blockSize, move.j);
            break;
        }
        
        case 5: { // Relocate pair
            relocatePair(result.sequence, move.i, move.j);
            relocatePair(result.truck_assign, move.i, move.j);
            relocatePair(result.drone_assign, move.i, move.j);
            relocatePair(result.break_bit, move.i, move.j);
            break;
        }
    }
    
    return result;
}

// ============ MOVE GENERATION FUNCTIONS ============

bool TabuSearchPDP::findBestSwapMove(const Chromosome& current, double currentCost,
                                     double bestCost, int iter, TabuMove& bestMove,
                                     Chromosome& bestCandidate, double& bestDelta) {
    bool found = false;
    
    for (int i = 0; i < (int)current.sequence.size(); ++i) {
        for (int j = i + 1; j < (int)current.sequence.size(); ++j) {
            TabuMove move{0, i, j, 0};
            
            bool isTabuMove = isTabu(move, iter);
            
            Chromosome candidate = applyMove(current, move);
            PDPSolution candidateSol = evaluateWithCache(candidate, data, cache);
            double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
            double delta = candidateCost - currentCost;
            
            // Aspiration criterion: accept tabu if better than best
            if (!isTabuMove || candidateCost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestMove = move;
                    bestCandidate = candidate;
                    found = true;
                }
            }
        }
    }
    
    return found;
}

bool TabuSearchPDP::findBestInsertMove(const Chromosome& current, double currentCost,
                                       double bestCost, int iter, TabuMove& bestMove,
                                       Chromosome& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = (int)current.sequence.size();
    
    // Limit search for large instances
    int maxTrials = min(50, n * n / 4);
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dist(0, n - 1);
    
    for (int trial = 0; trial < maxTrials; ++trial) {
        int i = dist(gen);
        int j = dist(gen);
        
        if (i == j) continue;
        
        TabuMove move{1, i, j, 0};
        bool isTabuMove = isTabu(move, iter);
        
        Chromosome candidate = applyMove(current, move);
        PDPSolution candidateSol = evaluateWithCache(candidate, data, cache);
        double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
        double delta = candidateCost - currentCost;
        
        if (!isTabuMove || candidateCost < bestCost) {
            if (delta < bestDelta) {
                bestDelta = delta;
                bestMove = move;
                bestCandidate = candidate;
                found = true;
            }
        }
    }
    
    return found;
}

bool TabuSearchPDP::findBest2OptMove(const Chromosome& current, double currentCost,
                                     double bestCost, int iter, TabuMove& bestMove,
                                     Chromosome& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = (int)current.sequence.size();
    
    // Try 2-opt on segments
    for (int i = 0; i < n - 1; ++i) {
        for (int j = i + 1; j < min(n, i + 20); ++j) { // Limit segment size
            TabuMove move{2, i, j, 0};
            bool isTabuMove = isTabu(move, iter);
            
            Chromosome candidate = applyMove(current, move);
            PDPSolution candidateSol = evaluateWithCache(candidate, data, cache);
            double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
            double delta = candidateCost - currentCost;
            
            if (!isTabuMove || candidateCost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestMove = move;
                    bestCandidate = candidate;
                    found = true;
                }
            }
        }
    }
    
    return found;
}

bool TabuSearchPDP::findBest2OptStarMove(const Chromosome& current, double currentCost,
                                         double bestCost, int iter, TabuMove& bestMove,
                                         Chromosome& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = (int)current.sequence.size();
    
    // Similar to 2-opt but different reversal strategy
    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 2; j < min(n, i + 15); ++j) {
            TabuMove move{3, i, j, 0};
            bool isTabuMove = isTabu(move, iter);
            
            Chromosome candidate = applyMove(current, move);
            PDPSolution candidateSol = evaluateWithCache(candidate, data, cache);
            double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
            double delta = candidateCost - currentCost;
            
            if (!isTabuMove || candidateCost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestMove = move;
                    bestCandidate = candidate;
                    found = true;
                }
            }
        }
    }
    
    return found;
}

bool TabuSearchPDP::findBestOrOptMove(const Chromosome& current, double currentCost,
                                      double bestCost, int iter, TabuMove& bestMove,
                                      Chromosome& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = (int)current.sequence.size();
    
    // Try block sizes 1, 2, 3
    for (int blockSize = 1; blockSize <= 3; ++blockSize) {
        if (n < blockSize + 1) continue;
        
        for (int i = 0; i <= n - blockSize; ++i) {
            for (int j = 0; j <= n - blockSize; ++j) {
                if (abs(i - j) < blockSize) continue; // Skip overlapping
                
                TabuMove move{4, i, j, blockSize};
                bool isTabuMove = isTabu(move, iter);
                
                Chromosome candidate = applyMove(current, move);
                PDPSolution candidateSol = evaluateWithCache(candidate, data, cache);
                double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
                double delta = candidateCost - currentCost;
                
                if (!isTabuMove || candidateCost < bestCost) {
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestMove = move;
                        bestCandidate = candidate;
                        found = true;
                    }
                }
            }
        }
    }
    
    return found;
}

bool TabuSearchPDP::findBestRelocatePairMove(const Chromosome& current, double currentCost,
                                             double bestCost, int iter, TabuMove& bestMove,
                                             Chromosome& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = (int)current.sequence.size();
    
    if (n < 3) return false;
    
    for (int i = 0; i < n - 1; ++i) {
        for (int j = 0; j <= n - 2; ++j) {
            if (abs(i - j) < 2) continue; // Skip overlapping
            
            TabuMove move{5, i, j, 0};
            bool isTabuMove = isTabu(move, iter);
            
            Chromosome candidate = applyMove(current, move);
            PDPSolution candidateSol = evaluateWithCache(candidate, data, cache);
            double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
            double delta = candidateCost - currentCost;
            
            if (!isTabuMove || candidateCost < bestCost) {
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestMove = move;
                    bestCandidate = candidate;
                    found = true;
                }
            }
        }
    }
    
    return found;
}

// ============ MAIN TABU SEARCH ============

Chromosome TabuSearchPDP::run(const Chromosome& initial) {
    Chromosome current = initial;
    Chromosome best = initial;

    // Ensure encoding vectors exist (Tabu moves operate on aligned gene arrays).
    int n = (int)current.sequence.size();
    if ((int)current.truck_assign.size() != n) current.truck_assign.assign(n, 0);
    if ((int)current.drone_assign.size() != n) current.drone_assign.assign(n, 0);
    if ((int)current.break_bit.size() != n) current.break_bit.assign(n, 1);

    best = current;

    PDPSolution currentSol = evaluateWithCache(current, data, cache);
    PDPSolution bestSol = currentSol;
    double currentCost = currentSol.totalCost + currentSol.totalPenalty;
    double bestCost = currentCost;
    
    const double delta1 = 0.5, delta2 = 0.3, delta3 = 0.2;
    int segmentLength = 100;
    int noImprovement = 0;
    const int maxNoImprovement = min(5000, maxIterations / 2);
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Select move type using adaptive weights
        int moveIndex = selectMoveIndex();
        
        TabuMove bestMove{-1, -1, -1, 0};
        Chromosome bestCandidate;
        double bestDelta = numeric_limits<double>::infinity();
        bool moveFound = false;
        
        // Find best move of selected type
        switch (moveIndex) {
            case 0:
                moveFound = findBestSwapMove(current, currentCost, bestCost, iter,
                                            bestMove, bestCandidate, bestDelta);
                break;
            case 1:
                moveFound = findBestInsertMove(current, currentCost, bestCost, iter,
                                              bestMove, bestCandidate, bestDelta);
                break;
            case 2:
                moveFound = findBest2OptMove(current, currentCost, bestCost, iter,
                                            bestMove, bestCandidate, bestDelta);
                break;
            case 3:
                moveFound = findBest2OptStarMove(current, currentCost, bestCost, iter,
                                                bestMove, bestCandidate, bestDelta);
                break;
            case 4:
                moveFound = findBestOrOptMove(current, currentCost, bestCost, iter,
                                             bestMove, bestCandidate, bestDelta);
                break;
            case 5:
                moveFound = findBestRelocatePairMove(current, currentCost, bestCost, iter,
                                                     bestMove, bestCandidate, bestDelta);
                break;
        }
        
        // Apply best move if found
        if (moveFound && bestMove.type != -1) {
            double previousCost = currentCost;
            current = bestCandidate;
            
            PDPSolution newSol = evaluateWithCache(current, data, cache);
            currentCost = newSol.totalCost + newSol.totalPenalty;
            currentSol = newSol;
            
            // Update adaptive weights
            if (currentCost < bestCost) {
                scores[moveIndex] += delta1; // Best solution found
                bestCost = currentCost;
                best = current;
                bestSol = newSol;
                noImprovement = 0;
            } else if (currentCost < previousCost) {
                scores[moveIndex] += delta2; // Improved current
                noImprovement++;
            } else {
                scores[moveIndex] += delta3; // Accepted worse
                noImprovement++;
            }
            
            usedCount[moveIndex]++;
            addTabu(bestMove, iter);
        } else {
            noImprovement++;
        }
        
        // Update weights periodically
        if ((iter + 1) % segmentLength == 0) {
            updateWeights(segmentLength);
        }
        
        // Early stopping
        if (noImprovement >= maxNoImprovement) {
            break;
        }
        
        // Progress output
        if (iter % 50 == 0 && iter > 0) {
            cout << "  Tabu iter " << iter << ": best=" << bestCost 
                 << " current=" << currentCost << endl;
        }
    }
    
    return best;
}


Chromosome tabuSearchPDP(const Chromosome& initial, const PDPData& data,
                         int maxIterations, SolutionCache& cache) {
    TabuSearchPDP tabu(data, maxIterations, cache);
    return tabu.run(initial);
}
