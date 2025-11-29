#include "pdp_tabu.h"
#include "pdp_fitness.h"
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

TabuSearchPDP::TabuSearchPDP(const PDPData& data, int maxIterations)
    : data(data), maxIterations(maxIterations) {
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

vector<int> TabuSearchPDP::applyMove(const vector<int>& seq, const TabuMove& move) const {
    vector<int> result = seq;
    
    switch (move.type) {
        case 0: // Swap
            swap(result[move.i], result[move.j]);
            break;
            
        case 1: { // Insert
            int temp = result[move.i];
            result.erase(result.begin() + move.i);
            result.insert(result.begin() + move.j, temp);
            break;
        }
        
        case 2: { // 2-opt (reverse segment)
            reverse(result.begin() + move.i, result.begin() + move.j + 1);
            break;
        }
        
        case 3: { // 2-opt* (reverse segment between i and j)
            if (move.i < move.j) {
                reverse(result.begin() + move.i, result.begin() + move.j + 1);
            }
            break;
        }
        
        case 4: { // Or-opt (relocate block)
            int blockSize = move.param;
            if (move.i + blockSize <= (int)result.size()) {
                vector<int> block(result.begin() + move.i, result.begin() + move.i + blockSize);
                result.erase(result.begin() + move.i, result.begin() + move.i + blockSize);
                
                int insertPos = move.j;
                if (move.j > move.i) insertPos -= blockSize;
                
                // Safety check: ensure insertPos is valid
                insertPos = max(0, min(insertPos, (int)result.size()));
                
                result.insert(result.begin() + insertPos, block.begin(), block.end());
            }
            break;
        }
        
        case 5: { // Relocate pair
            if (move.i + 1 < (int)result.size()) {
                int cust1 = result[move.i];
                int cust2 = result[move.i + 1];
                result.erase(result.begin() + move.i, result.begin() + move.i + 2);
                
                int insertPos = move.j;
                if (move.j > move.i) insertPos -= 2;
                
                // Safety check: ensure insertPos is valid
                insertPos = max(0, min(insertPos, (int)result.size()));
                
                // Insert both elements safely
                if (insertPos < (int)result.size()) {
                    result.insert(result.begin() + insertPos, cust1);
                    result.insert(result.begin() + insertPos + 1, cust2);
                } else {
                    // Insert at end
                    result.push_back(cust1);
                    result.push_back(cust2);
                }
            }
            break;
        }
    }
    
    return result;
}

// ============ MOVE GENERATION FUNCTIONS ============

bool TabuSearchPDP::findBestSwapMove(const vector<int>& currentSeq, double currentCost,
                                     double bestCost, int iter, TabuMove& bestMove,
                                     vector<int>& bestCandidate, double& bestDelta) {
    bool found = false;
    
    for (int i = 0; i < (int)currentSeq.size(); ++i) {
        for (int j = i + 1; j < (int)currentSeq.size(); ++j) {
            TabuMove move{0, i, j, 0};
            
            bool isTabuMove = isTabu(move, iter);
            
            vector<int> candidate = applyMove(currentSeq, move);
            PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
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

bool TabuSearchPDP::findBestInsertMove(const vector<int>& currentSeq, double currentCost,
                                       double bestCost, int iter, TabuMove& bestMove,
                                       vector<int>& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = currentSeq.size();
    
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
        
        vector<int> candidate = applyMove(currentSeq, move);
        PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
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

bool TabuSearchPDP::findBest2OptMove(const vector<int>& currentSeq, double currentCost,
                                     double bestCost, int iter, TabuMove& bestMove,
                                     vector<int>& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = currentSeq.size();
    
    // Try 2-opt on segments
    for (int i = 0; i < n - 1; ++i) {
        for (int j = i + 1; j < min(n, i + 20); ++j) { // Limit segment size
            TabuMove move{2, i, j, 0};
            bool isTabuMove = isTabu(move, iter);
            
            vector<int> candidate = applyMove(currentSeq, move);
            PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
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

bool TabuSearchPDP::findBest2OptStarMove(const vector<int>& currentSeq, double currentCost,
                                         double bestCost, int iter, TabuMove& bestMove,
                                         vector<int>& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = currentSeq.size();
    
    // Similar to 2-opt but different reversal strategy
    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 2; j < min(n, i + 15); ++j) {
            TabuMove move{3, i, j, 0};
            bool isTabuMove = isTabu(move, iter);
            
            vector<int> candidate = applyMove(currentSeq, move);
            PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
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

bool TabuSearchPDP::findBestOrOptMove(const vector<int>& currentSeq, double currentCost,
                                      double bestCost, int iter, TabuMove& bestMove,
                                      vector<int>& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = currentSeq.size();
    
    // Try block sizes 1, 2, 3
    for (int blockSize = 1; blockSize <= 3; ++blockSize) {
        if (n < blockSize + 1) continue;
        
        for (int i = 0; i <= n - blockSize; ++i) {
            for (int j = 0; j <= n - blockSize; ++j) {
                if (abs(i - j) < blockSize) continue; // Skip overlapping
                
                TabuMove move{4, i, j, blockSize};
                bool isTabuMove = isTabu(move, iter);
                
                vector<int> candidate = applyMove(currentSeq, move);
                PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
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

bool TabuSearchPDP::findBestRelocatePairMove(const vector<int>& currentSeq, double currentCost,
                                             double bestCost, int iter, TabuMove& bestMove,
                                             vector<int>& bestCandidate, double& bestDelta) {
    bool found = false;
    int n = currentSeq.size();
    
    if (n < 3) return false;
    
    for (int i = 0; i < n - 1; ++i) {
        for (int j = 0; j <= n - 2; ++j) {
            if (abs(i - j) < 2) continue; // Skip overlapping
            
            TabuMove move{5, i, j, 0};
            bool isTabuMove = isTabu(move, iter);
            
            vector<int> candidate = applyMove(currentSeq, move);
            PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
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

vector<int> TabuSearchPDP::run(const vector<int>& initialSeq) {
    vector<int> currentSeq = initialSeq;
    vector<int> bestSeq = initialSeq;
    
    PDPSolution currentSol = decodeAndEvaluate(currentSeq, data);
    PDPSolution bestSol = currentSol;
    double currentCost = currentSol.totalCost + currentSol.totalPenalty;
    double bestCost = currentCost;
    
    const double delta1 = 0.5, delta2 = 0.3, delta3 = 0.2;
    int segmentLength = 100;
    int noImprovement = 0;
    const int maxNoImprovement = min(1000, maxIterations / 2);
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Select move type using adaptive weights
        int moveIndex = selectMoveIndex();
        
        TabuMove bestMove{-1, -1, -1, 0};
        vector<int> bestCandidate;
        double bestDelta = numeric_limits<double>::infinity();
        bool moveFound = false;
        
        // Find best move of selected type
        switch (moveIndex) {
            case 0:
                moveFound = findBestSwapMove(currentSeq, currentCost, bestCost, iter, 
                                            bestMove, bestCandidate, bestDelta);
                break;
            case 1:
                moveFound = findBestInsertMove(currentSeq, currentCost, bestCost, iter,
                                              bestMove, bestCandidate, bestDelta);
                break;
            case 2:
                moveFound = findBest2OptMove(currentSeq, currentCost, bestCost, iter,
                                            bestMove, bestCandidate, bestDelta);
                break;
            case 3:
                moveFound = findBest2OptStarMove(currentSeq, currentCost, bestCost, iter,
                                                bestMove, bestCandidate, bestDelta);
                break;
            case 4:
                moveFound = findBestOrOptMove(currentSeq, currentCost, bestCost, iter,
                                             bestMove, bestCandidate, bestDelta);
                break;
            case 5:
                moveFound = findBestRelocatePairMove(currentSeq, currentCost, bestCost, iter,
                                                     bestMove, bestCandidate, bestDelta);
                break;
        }
        
        // Apply best move if found
        if (moveFound && bestMove.type != -1) {
            double previousCost = currentCost;
            currentSeq = bestCandidate;
            
            PDPSolution newSol = decodeAndEvaluate(currentSeq, data);
            currentCost = newSol.totalCost + newSol.totalPenalty;
            currentSol = newSol;
            
            // Update adaptive weights
            if (currentCost < bestCost) {
                scores[moveIndex] += delta1; // Best solution found
                bestCost = currentCost;
                bestSeq = currentSeq;
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
    
    return bestSeq;
}

// ============ SIMPLE INTERFACE ============

vector<int> tabuSearchPDP(const vector<int>& initialSeq, const PDPData& data, int maxIterations) {
    TabuSearchPDP tabu(data, maxIterations);
    return tabu.run(initialSeq);
}
