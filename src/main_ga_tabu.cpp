#include <iostream>
#include <string>
#include <iomanip>
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
    // Configuration parameters - modify these values directly
    const int POPULATION_SIZE = 200;
    const int MAX_GENERATIONS = 100;
    const double MUTATION_RATE = 0.1;
    const int RUN_NUMBER = 1;
    
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance_file>" << endl;
        cerr << "Example: " << argv[0] << " Instance /U_10_0.5_Num_1_pd.txt" << endl;
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

    cout << "\n+========================================================+" << endl;
    cout << "|     PDP SOLVER - GA + TABU SEARCH                  |" << endl;
    cout << "+========================================================+" << endl;

    // Read instance
    cout << "\nReading instance: " << instanceFile << endl;
    PDPData data;
    if (!readPDPFile(instanceFile, data)) {
        cerr << "Error: Failed to read instance file!" << endl;
        return 1;
    }
    
    cout << "\nInstance details:" << endl;
    cout << "  Customers: " << data.numCustomers << endl;
    cout << "  Trucks: " << data.numTrucks << endl;
    cout << "  Drones: " << data.numDrones << endl;
    cout << "  Drone endurance: " << data.droneEndurance << " minutes" << endl;
    
    // Run GA + Tabu
    PDPSolution solution = geneticAlgorithmPDP(data, populationSize, maxGenerations, mutationRate, runNumber);
    
    double costBeforeLS = solution.totalCost;
    
    // Recalculate actual C_max using LocalSearch helper
    IntegratedLocalSearch ils(data, 100);
    
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

    return 0;
}
