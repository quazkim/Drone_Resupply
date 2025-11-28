#include "pdp_ga.h"
#include "pdp_tabu.h"
#include "pdp_types.h"
#include "pdp_fitness.h"
#include "pdp_init.h"
#include <algorithm>
#include <random>
#include <map>
#include <set>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
#include <climits>

using namespace std;

// ============ ADAPTIVE PARAMETERS ============
struct AdaptiveParams {
    // Crossover success rates
    vector<double> crossoverSuccess;
    vector<int> crossoverUsage;
    vector<double> crossoverRates;
    
    // Mutation success rates
    vector<double> mutationSuccess;
    vector<int> mutationUsage;
    vector<double> mutationRates;
    
    // Adaptive mutation rate
    double currentMutationRate;
    double baseMutationRate;
    int noImprovementCount;
    
    AdaptiveParams(double baseRate) : 
        crossoverSuccess(4, 0.0), crossoverUsage(4, 0), crossoverRates(4, 0.25),
        mutationSuccess(5, 0.0), mutationUsage(5, 0), mutationRates(5, 0.2),
        currentMutationRate(baseRate), baseMutationRate(baseRate), noImprovementCount(0) {}
    
    // Cập nhật success rate cho crossover
    void updateCrossoverSuccess(int type, bool improved) {
        crossoverUsage[type]++;
        if (improved) crossoverSuccess[type]++;
    }
    
    // Cập nhật success rate cho mutation
    void updateMutationSuccess(int type, bool improved) {
        mutationUsage[type]++;
        if (improved) mutationSuccess[type]++;
    }
    
    // Điều chỉnh crossover rates dựa trên success rate
    void adaptCrossoverRates() {
        double totalSuccess = 0;
        for (int i = 0; i < 4; ++i) {
            if (crossoverUsage[i] > 0) {
                totalSuccess += crossoverSuccess[i] / crossoverUsage[i];
            }
        }
        
        if (totalSuccess > 0) {
            for (int i = 0; i < 4; ++i) {
                if (crossoverUsage[i] > 0) {
                    double successRate = crossoverSuccess[i] / crossoverUsage[i];
                    crossoverRates[i] = 0.1 + 0.8 * (successRate / totalSuccess);
                } else {
                    crossoverRates[i] = 0.25;
                }
            }
        }
    }
    
    // Điều chỉnh mutation rates dựa trên success rate
    void adaptMutationRates() {
        double totalSuccess = 0;
        for (int i = 0; i < 5; ++i) {
            if (mutationUsage[i] > 0) {
                totalSuccess += mutationSuccess[i] / mutationUsage[i];
            }
        }
        
        if (totalSuccess > 0) {
            for (int i = 0; i < 5; ++i) {
                if (mutationUsage[i] > 0) {
                    double successRate = mutationSuccess[i] / mutationUsage[i];
                    mutationRates[i] = 0.05 + 0.9 * (successRate / totalSuccess);
                } else {
                    mutationRates[i] = 0.2;
                }
            }
        }
    }
    
    // Điều chỉnh mutation rate tổng thể
    void adaptMutationRate() {
        if (noImprovementCount > 5) {
            currentMutationRate = min(0.5, currentMutationRate * 1.2);
        } else if (noImprovementCount < 2) {
            currentMutationRate = max(baseMutationRate * 0.5, currentMutationRate * 0.9);
        }
    }
    
    // Chọn crossover type dựa trên rates
    int selectCrossoverType(mt19937& gen) {
        discrete_distribution<> dist(crossoverRates.begin(), crossoverRates.end());
        return dist(gen);
    }
    
    // Chọn mutation type dựa trên rates
    int selectMutationType(mt19937& gen) {
        discrete_distribution<> dist(mutationRates.begin(), mutationRates.end());
        return dist(gen);
    }
};

// ============ CROSSOVER OPERATORS ============

// Order Crossover (OX)
vector<int> orderCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child(n, -1);
    
    uniform_int_distribution<> dist(0, n - 1);
    int cut1 = dist(gen);
    int cut2 = dist(gen);
    if (cut1 > cut2) swap(cut1, cut2);
    
    // Copy segment from parent1
    for (int i = cut1; i <= cut2; ++i) {
        child[i] = parent1[i];
    }
    
    // Fill remaining from parent2
    set<int> used;
    for (int i = cut1; i <= cut2; ++i) {
        used.insert(parent1[i]);
    }
    
    int childIdx = (cut2 + 1) % n;
    int parentIdx = (cut2 + 1) % n;
    
    while (childIdx != cut1) {
        if (used.find(parent2[parentIdx]) == used.end()) {
            child[childIdx] = parent2[parentIdx];
            used.insert(parent2[parentIdx]);
            childIdx = (childIdx + 1) % n;
        }
        parentIdx = (parentIdx + 1) % n;
    }
    
    return child;
}

// Partially Mapped Crossover (PMX)
vector<int> pmxCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child = parent1;
    
    uniform_int_distribution<> dist(0, n - 1);
    int cut1 = dist(gen);
    int cut2 = dist(gen);
    if (cut1 > cut2) swap(cut1, cut2);
    
    map<int, int> mapping;
    for (int i = cut1; i <= cut2; ++i) {
        mapping[parent1[i]] = parent2[i];
        child[i] = parent2[i];
    }
    
    for (int i = 0; i < n; ++i) {
        if (i >= cut1 && i <= cut2) continue;
        
        int value = parent1[i];
        while (mapping.find(value) != mapping.end()) {
            value = mapping[value];
        }
        child[i] = value;
    }
    
    return child;
}

// Cycle Crossover (CX)
vector<int> cycleCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child(n, -1);
    vector<bool> visited(n, false);
    
    uniform_int_distribution<> coin(0, 1);
    bool useParent1 = coin(gen);
    
    for (int start = 0; start < n; ++start) {
        if (visited[start]) continue;
        
        int idx = start;
        do {
            visited[idx] = true;
            child[idx] = useParent1 ? parent1[idx] : parent2[idx];
            
            int value = useParent1 ? parent2[idx] : parent1[idx];
            idx = find(parent1.begin(), parent1.end(), value) - parent1.begin();
        } while (idx != start);
        
        useParent1 = !useParent1;
    }
    
    return child;
}

// Edge Recombination Crossover (ERX) - Bảo toàn các cạnh từ cha mẹ
vector<int> edgeCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child;
    
    // Tạo bảng adjacency từ cả hai cha mẹ
    map<int, set<int>> edges;
    
    // Thêm edges từ parent1
    for (int i = 0; i < n; ++i) {
        int current = parent1[i];
        int prev = parent1[(i - 1 + n) % n];
        int next = parent1[(i + 1) % n];
        edges[current].insert(prev);
        edges[current].insert(next);
    }
    
    // Thêm edges từ parent2
    for (int i = 0; i < n; ++i) {
        int current = parent2[i];
        int prev = parent2[(i - 1 + n) % n];
        int next = parent2[(i + 1) % n];
        edges[current].insert(prev);
        edges[current].insert(next);
    }
    
    // Bắt đầu với node ngẫu nhiên
    uniform_int_distribution<> dist(0, n - 1);
    int current = parent1[dist(gen)];
    child.push_back(current);
    
    set<int> used;
    used.insert(current);
    
    while (child.size() < n) {
        // Xóa current khỏi tất cả adjacency lists
        for (auto& pair : edges) {
            pair.second.erase(current);
        }
        
        int next = -1;
        
        // Tìm neighbor với ít connections nhất
        int minConnections = INT_MAX;
        for (int neighbor : edges[current]) {
            if (used.find(neighbor) == used.end()) {
                int connections = edges[neighbor].size();
                if (connections < minConnections) {
                    minConnections = connections;
                    next = neighbor;
                }
            }
        }
        
        // Nếu không tìm thấy, chọn node chưa dùng ngẫu nhiên
        if (next == -1) {
            vector<int> unused;
            for (int i = 0; i < n; ++i) {
                int node = parent1[i];
                if (used.find(node) == used.end()) {
                    unused.push_back(node);
                }
            }
            if (!unused.empty()) {
                uniform_int_distribution<> unusedDist(0, unused.size() - 1);
                next = unused[unusedDist(gen)];
            }
        }
        
        if (next != -1) {
            child.push_back(next);
            used.insert(next);
            current = next;
        } else {
            break; // Không thể tiếp tục
        }
    }
    
    // Đảm bảo child có đủ n phần tử
    if (child.size() < n) {
        for (int i = 0; i < n; ++i) {
            int node = parent1[i];
            if (used.find(node) == used.end()) {
                child.push_back(node);
                used.insert(node);
            }
        }
    }
    
    return child;
}

// ============ MUTATION OPERATORS ============

void swapMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    swap(seq[i], seq[j]);
}

void inversionMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    if (i > j) swap(i, j);
    
    reverse(seq.begin() + i, seq.begin() + j + 1);
}

void scrambleMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    if (i > j) swap(i, j);
    
    shuffle(seq.begin() + i, seq.begin() + j + 1, gen);
}

// Insertion Mutation - Di chuyển một phần tử đến vị trí khác
void insertionMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int fromPos = dist(gen);
    int toPos = dist(gen);
    
    if (fromPos == toPos) return;
    
    int element = seq[fromPos];
    if (fromPos < toPos) {
        for (int i = fromPos; i < toPos; ++i) {
            seq[i] = seq[i + 1];
        }
    } else {
        for (int i = fromPos; i > toPos; --i) {
            seq[i] = seq[i - 1];
        }
    }
    seq[toPos] = element;
}

// Displacement Mutation - Di chuyển một đoạn con đến vị trí khác
void displacementMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 3) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int start = dist(gen);
    int end = dist(gen);
    if (start > end) swap(start, end);
    
    // Đảm bảo có ít nhất 1 phần tử để di chuyển
    if (start == end) {
        if (end < seq.size() - 1) end++;
        else start--;
    }
    
    // Chọn vị trí đích (không trùng với đoạn hiện tại)
    vector<int> validPositions;
    for (int i = 0; i <= (int)seq.size() - (end - start + 1); ++i) {
        if (i < start || i > end - (end - start)) {
            validPositions.push_back(i);
        }
    }
    
    if (validPositions.empty()) return;
    
    uniform_int_distribution<> posDist(0, validPositions.size() - 1);
    int newPos = validPositions[posDist(gen)];
    
    // Lưu đoạn cần di chuyển
    vector<int> segment(seq.begin() + start, seq.begin() + end + 1);
    
    // Xóa đoạn cũ
    seq.erase(seq.begin() + start, seq.begin() + end + 1);
    
    // Chèn vào vị trí mới
    if (newPos > start) {
        newPos -= (end - start + 1);
    }
    seq.insert(seq.begin() + newPos, segment.begin(), segment.end());
}

// ============ REPAIR OPERATOR ============

void repairSequence(vector<int>& seq, const PDPData& data, mt19937& gen) {
    // Đếm số lần xuất hiện của mỗi customer
    map<int, int> count;
    for (int id : seq) {
        if (data.isCustomer(id)) {
            count[id]++;
        }
    }
    
    // Tìm customer bị thiếu
    vector<int> missing;
    for (int i = 0; i < data.numNodes; ++i) {
        if (data.isCustomer(i) && count[i] == 0) {
            missing.push_back(i);
        }
    }
    
    // Loại bỏ duplicate và thay bằng missing
    shuffle(missing.begin(), missing.end(), gen);
    int missingIdx = 0;
    
    for (int& id : seq) {
        if (data.isCustomer(id) && count[id] > 1) {
            if (missingIdx < (int)missing.size()) {
                int oldId = id;
                id = missing[missingIdx++];
                count[oldId]--;
                count[id]++;
            }
        }
    }
    
    // Thêm missing còn lại
    for (int i = missingIdx; i < (int)missing.size(); ++i) {
        seq.push_back(missing[i]);
    }
}

// ============ SELECTION ============

vector<int> tournamentSelection(const vector<vector<int>>& population,
                                const vector<double>& fitness,
                                int tournamentSize,
                                mt19937& gen) {
    uniform_int_distribution<> dist(0, population.size() - 1);
    
    int bestIdx = dist(gen);
    double bestFitness = fitness[bestIdx];
    
    for (int i = 1; i < tournamentSize; ++i) {
        int idx = dist(gen);
        if (fitness[idx] < bestFitness) {
            bestIdx = idx;
            bestFitness = fitness[idx];
        }
    }
    
    return population[bestIdx];
}

// ============ ADAPTIVE EVALUATION ============

// Đánh giá xem offspring có tốt hơn parents không
bool evaluateImprovement(const vector<int>& offspring, const vector<int>& parent1, 
                        const vector<int>& parent2, const PDPData& data) {
    PDPSolution offspringSol = decodeAndEvaluate(offspring, data);
    PDPSolution parent1Sol = decodeAndEvaluate(parent1, data);
    PDPSolution parent2Sol = decodeAndEvaluate(parent2, data);
    
    double offspringFit = offspringSol.totalCost + offspringSol.totalPenalty;
    double parent1Fit = parent1Sol.totalCost + parent1Sol.totalPenalty;
    double parent2Fit = parent2Sol.totalCost + parent2Sol.totalPenalty;
    
    double bestParentFit = min(parent1Fit, parent2Fit);
    
    return offspringFit < bestParentFit;
}

// Đánh giá xem mutated solution có tốt hơn original không
bool evaluateMutationImprovement(const vector<int>& mutated, const vector<int>& original, 
                               const PDPData& data) {
    PDPSolution mutatedSol = decodeAndEvaluate(mutated, data);
    PDPSolution originalSol = decodeAndEvaluate(original, data);
    
    double mutatedFit = mutatedSol.totalCost + mutatedSol.totalPenalty;
    double originalFit = originalSol.totalCost + originalSol.totalPenalty;
    
    return mutatedFit < originalFit;
}

// ============ MAIN GA ALGORITHM ============

PDPSolution geneticAlgorithmPDP(const PDPData& data, int populationSize, 
                               int maxGenerations, double mutationRate, int runNumber) {
    random_device rd;
    mt19937 rng(rd() + runNumber * 12345);
    
    cout << "\n=========================================" << endl;
    cout << "  GENETIC ALGORITHM + TABU SEARCH (PDP)" << endl;
    cout << "=========================================" << endl;
    cout << "Population size: " << populationSize << endl;
    cout << "Max generations: " << maxGenerations << endl;
    cout << "Base mutation rate: " << mutationRate << " (adaptive)" << endl;
    cout << "Tabu threshold: 10% of max gen without improvement" << endl;
    cout << "Adaptive operators: ENABLED" << endl;
    
    // Initialize adaptive parameters
    AdaptiveParams adaptiveParams(mutationRate);
    
    // STEP 1: Initialize population
    cout << "\n[1] Initializing population..." << endl;
    vector<vector<int>> population = initStructuredPopulationPDP(populationSize, data, runNumber);
    
    // Evaluate initial population
    vector<double> fitness(populationSize);
    PDPSolution bestSolution;
    bestSolution.totalCost = numeric_limits<double>::infinity();
    vector<int> bestSequence;
    
    for (int i = 0; i < populationSize; ++i) {
        PDPSolution sol = decodeAndEvaluate(population[i], data);
        fitness[i] = sol.totalCost + sol.totalPenalty;
        
        if (fitness[i] < bestSolution.totalCost + bestSolution.totalPenalty) {
            bestSolution = sol;
            bestSequence = population[i];
        }
    }
    
    cout << "Initial best cost: " << fixed << setprecision(2) 
         << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
    
    int noImprovementCounter = 0;
    int tabuThreshold = maxGenerations / 10;
    bool tabuApplied = false;
    int adaptationInterval = max(5, maxGenerations / 20);
    
    // STEP 2: GA Loop
    for (int generation = 0; generation < maxGenerations; ++generation) {
        // 2.1: Create offspring using adaptive crossover
        vector<vector<int>> offspring;
        vector<int> crossoverTypes;
        vector<pair<vector<int>, vector<int>>> parentPairs;
        
        int numOffspring = populationSize;
        for (int i = 0; i < numOffspring; ++i) {
            vector<int> parent1 = tournamentSelection(population, fitness, 3, rng);
            vector<int> parent2 = tournamentSelection(population, fitness, 3, rng);
            parentPairs.push_back({parent1, parent2});
            
            int crossoverType = adaptiveParams.selectCrossoverType(rng);
            crossoverTypes.push_back(crossoverType);
            
            vector<int> child;
            if (crossoverType == 0) {
                child = orderCrossover(parent1, parent2, rng);
            } else if (crossoverType == 1) {
                child = pmxCrossover(parent1, parent2, rng);
            } else if (crossoverType == 2) {
                child = cycleCrossover(parent1, parent2, rng);
            } else {
                child = edgeCrossover(parent1, parent2, rng);
            }
            
            // Repair to ensure all customers present
            repairSequence(child, data, rng);
            offspring.push_back(child);
            
            // Đánh giá hiệu suất crossover
            bool improved = evaluateImprovement(child, parent1, parent2, data);
            adaptiveParams.updateCrossoverSuccess(crossoverType, improved);
        }
        
        // 2.2: Adaptive Mutation
        int mutationCount = (int)(offspring.size() * adaptiveParams.currentMutationRate);
        uniform_int_distribution<> offspringDist(0, offspring.size() - 1);
        
        for (int i = 0; i < mutationCount; ++i) {
            int idx = offspringDist(rng);
            vector<int> original = offspring[idx];
            
            int mutationType = adaptiveParams.selectMutationType(rng);
            
            if (mutationType == 0) {
                swapMutation(offspring[idx], rng);
            } else if (mutationType == 1) {
                inversionMutation(offspring[idx], rng);
            } else if (mutationType == 2) {
                scrambleMutation(offspring[idx], rng);
            } else if (mutationType == 3) {
                insertionMutation(offspring[idx], rng);
            } else {
                displacementMutation(offspring[idx], rng);
            }
            
            repairSequence(offspring[idx], data, rng);
            
            // Đánh giá hiệu suất mutation
            bool improved = evaluateMutationImprovement(offspring[idx], original, data);
            adaptiveParams.updateMutationSuccess(mutationType, improved);
        }
        
        // 2.3: Evaluate offspring
        vector<double> offspringFitness(offspring.size());
        for (size_t i = 0; i < offspring.size(); ++i) {
            PDPSolution sol = decodeAndEvaluate(offspring[i], data);
            offspringFitness[i] = sol.totalCost + sol.totalPenalty;
        }
        
        // 2.4: Selection - 70% best offspring + 30% best parents
        int numBestOffspring = (int)(populationSize * 0.7);
        int numBestParents = populationSize - numBestOffspring;
        
        // Sort offspring
        vector<int> offspringIndices(offspring.size());
        iota(offspringIndices.begin(), offspringIndices.end(), 0);
        sort(offspringIndices.begin(), offspringIndices.end(),
             [&](int a, int b) { return offspringFitness[a] < offspringFitness[b]; });
        
        // Sort parents
        vector<int> parentIndices(population.size());
        iota(parentIndices.begin(), parentIndices.end(), 0);
        sort(parentIndices.begin(), parentIndices.end(),
             [&](int a, int b) { return fitness[a] < fitness[b]; });
        
        // Create new population
        vector<vector<int>> newPopulation;
        vector<double> newFitness;
        
        for (int i = 0; i < numBestOffspring; ++i) {
            newPopulation.push_back(offspring[offspringIndices[i]]);
            newFitness.push_back(offspringFitness[offspringIndices[i]]);
        }
        
        for (int i = 0; i < numBestParents; ++i) {
            newPopulation.push_back(population[parentIndices[i]]);
            newFitness.push_back(fitness[parentIndices[i]]);
        }
        
        population = newPopulation;
        fitness = newFitness;
        
        // Update best solution
        double currentBestCost = bestSolution.totalCost + bestSolution.totalPenalty;
        if (fitness[0] < currentBestCost) {
            PDPSolution sol = decodeAndEvaluate(population[0], data);
            bestSolution = sol;
            bestSequence = population[0];
            noImprovementCounter = 0;
            adaptiveParams.noImprovementCount = 0;
            
            cout << "Gen " << generation << ": New best = " << fixed << setprecision(2)
                 << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty 
                 << ", mut_rate: " << setprecision(3) << adaptiveParams.currentMutationRate << ")" << endl;
        } else {
            noImprovementCounter++;
            adaptiveParams.noImprovementCount++;
        }
        
        // 2.6: Adaptive parameter updates
        if (generation > 0 && generation % adaptationInterval == 0) {
            adaptiveParams.adaptCrossoverRates();
            adaptiveParams.adaptMutationRates();
            adaptiveParams.adaptMutationRate();
            
            // In thống kê adaptive (mỗi 10 generations)
            if (generation % (adaptationInterval * 2) == 0) {
                cout << "[ADAPT] Gen " << generation << " - Crossover rates: ";
                for (int i = 0; i < 4; ++i) {
                    cout << fixed << setprecision(2) << adaptiveParams.crossoverRates[i] << " ";
                }
                cout << ", Mutation rate: " << setprecision(3) << adaptiveParams.currentMutationRate << endl;
            }
        }
        
        // 2.5: Apply Tabu Search after 10% generations without improvement
        if (noImprovementCounter >= tabuThreshold && !tabuApplied) {
            cout << "\n[TABU] No improvement for " << tabuThreshold 
                 << " generations. Applying Tabu Search..." << endl;
            
            // Apply tabu to best sequence
            bestSequence = tabuSearchPDP(bestSequence, data, 100);
            bestSolution = decodeAndEvaluate(bestSequence, data);
            
            // Replace worst individual with tabu result
            population[populationSize - 1] = bestSequence;
            fitness[populationSize - 1] = bestSolution.totalCost + bestSolution.totalPenalty;
            
            cout << "[TABU] After Tabu Search: " << fixed << setprecision(2)
                 << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
            
            tabuApplied = true;
            noImprovementCounter = 0;
        }
    }
    
    cout << "\n=========================================" << endl;
    cout << "  GA + TABU COMPLETED" << endl;
    cout << "=========================================" << endl;
    cout << "Final best cost: " << fixed << setprecision(2)
         << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
    
    // In thống kê adaptive cuối cùng
    cout << "\n[ADAPTIVE STATS]" << endl;
    cout << "Final mutation rate: " << fixed << setprecision(3) << adaptiveParams.currentMutationRate << endl;
    cout << "Crossover success rates: ";
    for (int i = 0; i < 4; ++i) {
        if (adaptiveParams.crossoverUsage[i] > 0) {
            double rate = adaptiveParams.crossoverSuccess[i] / adaptiveParams.crossoverUsage[i];
            cout << "[" << i << "]:" << setprecision(2) << rate << " ";
        }
    }
    cout << endl;
    cout << "Mutation success rates: ";
    for (int i = 0; i < 5; ++i) {
        if (adaptiveParams.mutationUsage[i] > 0) {
            double rate = adaptiveParams.mutationSuccess[i] / adaptiveParams.mutationUsage[i];
            cout << "[" << i << "]:" << setprecision(2) << rate << " ";
        }
    }
    cout << endl;
    
    return bestSolution;
}
