#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include "pdp_types.h"
#include "pdp_reader.h"
#include "pdp_utils.h"
#include "pdp_ga.h"
#include "pdp_tabu.h"
#include "pdp_fitness.h"
#include "pdp_localsearch.h"
#include "pdp_validation.h"

using namespace std;

int main(int argc, char* argv[]) {
    // Start total timer
    auto startTotal = chrono::high_resolution_clock::now();
    
    // Configuration parameters - modify these values directly
    const int POPULATION_SIZE = 200;       // Balanced: quality vs speed
    const int MAX_GENERATIONS = 500;       // Standard iterations
    const double MUTATION_RATE = 0.15;     // Slightly higher diversity
    const int RUN_NUMBER = 1;
    
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance_file> [--depot MODE]" << endl;
        cerr << "Depot modes:" << endl;
        cerr << "  0 = center (default)" << endl;
        cerr << "  1 = border" << endl;
        cerr << "  2 = outside" << endl;
        cerr << "Examples:" << endl;
        cerr << "  " << argv[0] << " Instance/U_10_0.5_Num_1.txt" << endl;
        cerr << "  " << argv[0] << " Instance/U_30_0.5_Num_1.txt --depot 1" << endl;
        cerr << "  " << argv[0] << " Instance/U_50_1.0_Num_1.txt --depot 2" << endl;
        cerr << "\nCurrent parameters (edit in main_ga_tabu.cpp):" << endl;
        cerr << "  Population Size: " << POPULATION_SIZE << endl;
        cerr << "  Max Generations: " << MAX_GENERATIONS << endl;
        cerr << "  Mutation Rate: " << MUTATION_RATE << endl;
        cerr << "  Run Number: " << RUN_NUMBER << endl;
        return 1;
    }
    string instanceFile = argv[1];
    int populationSize = POPULATION_SIZE;
    int maxGenerations = MAX_GENERATIONS;
    double mutationRate = MUTATION_RATE;
    int runNumber = RUN_NUMBER;
    
    // Parse optional --depot MODE argument (0=center, 1=border, 2=outside)
    int depotMode = 0;  // default: center
    for (int i = 2; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--depot" && i + 1 < argc) {
            istringstream modeStream(argv[i + 1]);
            if (modeStream >> depotMode && depotMode >= 0 && depotMode <= 2) {
                // valid mode
            } else {
                cerr << "Error: --depot MODE must be 0 (center), 1 (border), or 2 (outside)" << endl;
                return 1;
            }
            i++;
        }
    }

    cout << "\n+========================================================+" << endl;
    cout << "|     PDP SOLVER - GA + TABU SEARCH                  |" << endl;
    cout << "+========================================================+" << endl;

    // Read instance
    cout << "\nReading instance: " << instanceFile << endl;
    PDPData data;
    data.depotMode = depotMode;
    string depotName[] = {"center", "border", "outside"};
    cout << "Using depot: " << depotName[depotMode] << endl;
    
    if (!readPDPFile(instanceFile, data)) {
        cerr << "Error: Failed to read instance file!" << endl;
        return 1;
    }
    
    cout << "\nInstance details:" << endl;
    cout << "  Customers: " << data.numCustomers << endl;
    cout << "  Trucks: " << data.numTrucks << endl;
    cout << "  Drones: " << data.numDrones << endl;
    cout << "  Drone endurance: " << data.droneEndurance << " minutes" << endl;
    cout << "  Depot: (" << data.coordinates[data.depotIndex].first 
         << ", " << data.coordinates[data.depotIndex].second << ")" << endl;
    
    // Detect scale
    const int TOTAL_NODES = data.numCustomers + 1;  // +1 for depot
    bool isSmallScale = (TOTAL_NODES <= 15);
    if (isSmallScale) {
        populationSize = 40;
        maxGenerations = 30;
        cout << "\n[SCALE DETECTION] Small scale mode (nodes <= 15)" << endl;
    }
    
    // Run GA + Tabu
    PDPSolution solution = geneticAlgorithmPDP(data, populationSize, maxGenerations, mutationRate, runNumber, isSmallScale);
    
    double costBeforeLS = solution.totalCost;
    
    // Recalculate actual C_max using LocalSearch helper
    IntegratedLocalSearch ils(data, 50);
    
    cout << "\n+========================================================+" << endl;
    cout << "|               GA + TABU RESULT                       |" << endl;
    cout << "+========================================================+" << endl;
    cout << "Final cost: " << fixed << setprecision(2) << costBeforeLS << " min" << endl;
    
    double costAfterLS = solution.totalCost;
    
    // Print final solution
    cout << "\n+========================================================+" << endl;
    cout << "|               FINAL SOLUTION                       |" << endl;
    cout << "+========================================================+" << endl;
    printSolution(solution, data);
    
    // ========== VALIDATION ==========
    validateSolution(solution, data, true);
    
    // ========== COST SUMMARY ==========
    cout << "\n=========================================\n";
    cout << "Cost before Local Search: " << fixed << setprecision(2) << costBeforeLS << " minutes\n";
    cout << "=========================================\n";
    
    // End total timer and print
    auto endTotal = chrono::high_resolution_clock::now();
    double totalTimeSec = chrono::duration<double>(endTotal - startTotal).count();
    
    cout << "\n[TOTAL RUNTIME] " << fixed << setprecision(2) << totalTimeSec << " seconds\n" << endl;

    return 0;
}
