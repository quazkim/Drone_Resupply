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

using namespace std;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance_file> [pop_size] [max_gen] [mutation_rate] [run_number]" << endl;
        cerr << "Example: " << argv[0] << " U_10_0.5_Num_1_pd.txt 50 100 0.1 1" << endl;
        return 1;
    }
    string instanceFile = argv[1];
    int populationSize = (argc >= 3) ? stoi(argv[2]) : 50;
    int maxGenerations = (argc >= 4) ? stoi(argv[3]) : 100;
    double mutationRate = (argc >= 5) ? stod(argv[4]) : 0.1;
    int runNumber = (argc >= 6) ? stoi(argv[5]) : 1;

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
    
    // Apply Solution-based Local Search
    cout << "\n+========================================================+" << endl;
    cout << "|     INTEGRATED LOCAL SEARCH (Truck + Drone)         |" << endl;
    cout << "+========================================================+" << endl;
    
    IntegratedLocalSearch ils(data, 50);  // 500 iterations max
    solution = ils.run(solution);
    
    double costAfterLS = solution.totalCost;
    
    // Print final solution
    cout << "\n+========================================================+" << endl;
    cout << "|               FINAL SOLUTION                       |" << endl;
    cout << "+========================================================+" << endl;
    printSolution(solution, data);
    
    cout << "\n=========================================" << endl;
    cout << "SUMMARY:" << endl;
    cout << "  Cost before Local Search: " << fixed << setprecision(2) << costBeforeLS << " minutes" << endl;
    cout << "  Cost after Local Search: " << fixed << setprecision(2) << costAfterLS << " minutes" << endl;
    if (costAfterLS < costBeforeLS) {
        cout << "  Improvement by LS: " << (costBeforeLS - costAfterLS) << " minutes ("
             << fixed << setprecision(1) << ((costBeforeLS - costAfterLS) / costBeforeLS * 100) << "%)" << endl;
    }
    cout << "  Total Penalty: " << solution.totalPenalty << endl;
    cout << "  Feasible: " << (solution.isFeasible ? "YES" : "NO") << endl;
    cout << "  Resupply Events: " << solution.resupply_events.size() << endl;
    cout << "=========================================" << endl;

    return 0;
}
