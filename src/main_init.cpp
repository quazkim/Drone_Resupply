/**
 * @file main_init.cpp
 * @brief Test harness for population initialization.
 *        Sử dụng kiến trúc mã hóa vector<Gene> (Chromosome).
 *
 * Usage: ./main_init <instance_file> [pop_size]
 */

#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <iomanip>
#include <chrono>

#include "pdp_types.h"
#include "pdp_reader.h"
#include "pdp_init.h"
#include "pdp_fitness.h"
#include "pdp_utils.h"   // printSolution, printChromosome

using namespace std;

int main(int argc, char* argv[]) {
    cout << "==========================================================\n"
         << "   PDP SOLVER - INITIALIZATION TEST (Gene-based encoding)\n"
         << "==========================================================\n";

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <filename> [pop_size]\n";
        return 1;
    }
    const string filename = argv[1];
    const int    popSize  = (argc > 2) ? stoi(argv[2]) : 100;

    try {
        // ---- [1] LOAD DATA ----
        PDPData data;
        cout << "\n--- [1] LOADING DATA ---\n";
        if (!readPDPFile(filename, data)) return 1;
        showPDPInfo(data);

        cout << "Loaded: " << data.numNodes    << " nodes ("
             << data.numCustomers << " customers)\n"
             << "Trucks: " << data.numTrucks   << "  Drones: " << data.numDrones << "\n";

        // ---- [2] INITIALIZE POPULATION (returns vector<Chromosome>) ----
        cout << "\n--- [2] RUNNING INITIALIZATION ALGORITHMS ---\n";
        auto t0 = chrono::high_resolution_clock::now();

        // initStructuredPopulationPDP returns vector<Chromosome> (= vector<vector<Gene>>)
        vector<Chromosome> population = initStructuredPopulationPDP(popSize, data, 1);

        double elapsed = chrono::duration<double>(
            chrono::high_resolution_clock::now() - t0).count();

        cout << ">> Generated " << population.size()
             << " individuals in " << fixed << setprecision(3)
             << elapsed << "s.\n";

        // ---- [3] EVALUATE INITIAL POPULATION ----
        cout << "\n--- [3] EVALUATING INITIAL POPULATION ---\n";

        PDPSolution bestSol;
        bestSol.totalCost = numeric_limits<double>::max();
        Chromosome  bestSeq;  // vector<Gene> — không còn vector<int>

        int    feasibleCount = 0;
        double sumCost       = 0.0;

        for (size_t i = 0; i < population.size(); ++i) {
            // Evaluate the full Chromosome (which includes resupply_vector) directly
            PDPSolution sol = decode_sequence(population[i], data, false);
            sol.original_sequence = population[i];

            double fitness = sol.totalCost + sol.totalPenalty;
            sumCost += sol.totalCost;
            if (sol.isFeasible) feasibleCount++;

            // Chọn tốt nhất: ưu tiên feasible, rồi đến fitness thấp nhất
            bool isNewBest = false;
            if (sol.isFeasible) {
                if (!bestSol.isFeasible || sol.totalCost < bestSol.totalCost)
                    isNewBest = true;
            } else if (!bestSol.isFeasible &&
                       fitness < bestSol.totalCost + bestSol.totalPenalty) {
                isNewBest = true;
            }

            if (isNewBest) {
                bestSol = sol;
                bestSeq = population[i];   // vector<Gene>
            }
        }

        // ---- [4] RESULTS ----
        cout << "\n--- [4] RESULTS ---\n"
             << "Population Size:    " << population.size() << "\n"
             << "Feasible Solutions: " << feasibleCount << " ("
             << fixed << setprecision(1)
             << (population.size() > 0
                 ? feasibleCount * 100.0 / population.size() : 0.0)
             << "%)\n"
             << "Average C_max:      "
             << (population.size() > 0 ? sumCost / population.size() : 0.0) << "\n";

        cout << "\n BEST INITIAL SOLUTION FOUND:\n";
        cout << "  C_max:   " << fixed << setprecision(2) << bestSol.totalCost << " min\n"
             << "  Penalty: " << bestSol.totalPenalty << "\n"
             << "  Feasible: " << (bestSol.isFeasible ? "YES" : "NO") << "\n";

        // ---- In chromosome bằng hàm tiện ích (thay vòng lặp vector<int> cũ) ----
        cout << "\n CHROMOSOME (best individual):\n";
        printChromosome(bestSeq);   // vector<Gene> → định dạng [Node:X | Resupply:{...}]

        // ---- In chi tiết lời giải đã decode ----
        printSolution(bestSol, data);

    } catch (const exception& e) {
        cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}