#include "pdp_ga_tabu.h"
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

using namespace std;

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

// ============ TABU SEARCH ============

string TabuMove::key() const {
    return to_string(type) + "_" + to_string(i) + "_" + to_string(j);
}

vector<int> tabuSearchPDP(const vector<int>& initialSeq, const PDPData& data, int maxIterations) {
    vector<int> currentSeq = initialSeq;
    vector<int> bestSeq = initialSeq;
    
    PDPSolution currentSol = decodeAndEvaluate(currentSeq, data);
    PDPSolution bestSol = currentSol;
    double bestCost = bestSol.totalCost + bestSol.totalPenalty;
    
    map<string, int> tabuList;
    int tabuTenure = (int)sqrt(data.numCustomers);
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        double bestDelta = numeric_limits<double>::infinity();
        TabuMove bestMove{-1, -1, -1};
        vector<int> bestCandidate;
        
        // Try swap moves
        for (int i = 0; i < (int)currentSeq.size(); ++i) {
            for (int j = i + 1; j < (int)currentSeq.size(); ++j) {
                vector<int> candidate = currentSeq;
                swap(candidate[i], candidate[j]);
                
                TabuMove move{0, i, j};
                bool isTabu = (tabuList.find(move.key()) != tabuList.end() && 
                              tabuList[move.key()] > iter);
                
                PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
                double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
                double currentCost = currentSol.totalCost + currentSol.totalPenalty;
                double delta = candidateCost - currentCost;
                
                // Aspiration criterion
                if (!isTabu || candidateCost < bestCost) {
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestMove = move;
                        bestCandidate = candidate;
                    }
                }
            }
        }
        
        // Try insert moves (limited)
        if ((int)currentSeq.size() > 10) {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> dist(0, currentSeq.size() - 1);
            
            for (int trial = 0; trial < min(50, (int)currentSeq.size()); ++trial) {
                int i = dist(gen);
                int j = dist(gen);
                
                vector<int> candidate = currentSeq;
                int temp = candidate[i];
                candidate.erase(candidate.begin() + i);
                candidate.insert(candidate.begin() + j, temp);
                
                TabuMove move{1, i, j};
                bool isTabu = (tabuList.find(move.key()) != tabuList.end() && 
                              tabuList[move.key()] > iter);
                
                PDPSolution candidateSol = decodeAndEvaluate(candidate, data);
                double candidateCost = candidateSol.totalCost + candidateSol.totalPenalty;
                double currentCost = currentSol.totalCost + currentSol.totalPenalty;
                double delta = candidateCost - currentCost;
                
                if (!isTabu || candidateCost < bestCost) {
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestMove = move;
                        bestCandidate = candidate;
                    }
                }
            }
        }
        
        // Apply best move
        if (bestMove.type != -1 && !bestCandidate.empty()) {
            currentSeq = bestCandidate;
            currentSol = decodeAndEvaluate(currentSeq, data);
            tabuList[bestMove.key()] = iter + tabuTenure;
            
            double currentCost = currentSol.totalCost + currentSol.totalPenalty;
            if (currentCost < bestCost) {
                bestCost = currentCost;
                bestSol = currentSol;
                bestSeq = currentSeq;
            }
        }
    }
    
    return bestSeq;
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
    cout << "Mutation rate: " << mutationRate << endl;
    cout << "Tabu threshold: 10% of max gen without improvement" << endl;
    
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
    
    // STEP 2: GA Loop
    for (int generation = 0; generation < maxGenerations; ++generation) {
        // 2.1: Create offspring using 3 crossovers
        vector<vector<int>> offspring;
        
        int numOffspring = populationSize;
        for (int i = 0; i < numOffspring; ++i) {
            vector<int> parent1 = tournamentSelection(population, fitness, 3, rng);
            vector<int> parent2 = tournamentSelection(population, fitness, 3, rng);
            
            uniform_int_distribution<> crossoverChoice(0, 2);
            int choice = crossoverChoice(rng);
            
            vector<int> child;
            if (choice == 0) {
                child = orderCrossover(parent1, parent2, rng);
            } else if (choice == 1) {
                child = pmxCrossover(parent1, parent2, rng);
            } else {
                child = cycleCrossover(parent1, parent2, rng);
            }
            
            // Repair to ensure all customers present
            repairSequence(child, data, rng);
            offspring.push_back(child);
        }
        
        // 2.2: Mutation (10% of offspring)
        int mutationCount = (int)(offspring.size() * mutationRate);
        uniform_int_distribution<> offspringDist(0, offspring.size() - 1);
        uniform_int_distribution<> mutationType(0, 2);
        
        for (int i = 0; i < mutationCount; ++i) {
            int idx = offspringDist(rng);
            int type = mutationType(rng);
            
            if (type == 0) {
                swapMutation(offspring[idx], rng);
            } else if (type == 1) {
                inversionMutation(offspring[idx], rng);
            } else {
                scrambleMutation(offspring[idx], rng);
            }
            
            repairSequence(offspring[idx], data, rng);
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
            
            cout << "Gen " << generation << ": New best = " << fixed << setprecision(2)
                 << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
        } else {
            noImprovementCounter++;
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
    
    return bestSolution;
}
