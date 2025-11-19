#include "pdp_reader.h"
#include "pdp_init.h"
#include "pdp_utils.h"
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>

using namespace std;

int main() {
    cout << "DRONE RESUPPLY - PDP SOLVER TESTING" << endl;
    cout << "================================================" << endl;

    // 1. TEST READING DATA
    cout << "\n1. TESTING DATA READER..." << endl;
    PDPData data;
    string filename = "U_10_0.5_Num_1_pd.txt"; // Test with smaller file
    
    if (!readPDPFile(filename, data)) {
        cerr << " Failed to read file: " << filename << endl;
        return -1;
    }
    
    showPDPInfo(data);
    
    // HIỂN THỊ THÔNG TIN CHI TIẾT DEPOT (THEO README)
    cout << "\n=== DEPOT INFORMATION (FROM README) ===" << endl;
    cout << "Depot configuration: " << (data.useDepotCenter ? "CENTER" : "BORDER") << endl;
    cout << "Fixed depot coordinates: (" << (data.useDepotCenter ? data.depotCenter.first : data.depotBorder.first) 
         << ", " << (data.useDepotCenter ? data.depotCenter.second : data.depotBorder.second) << ")" << endl;
    cout << "Depot array index: " << data.depotIndex << " (0-based)" << endl;
    cout << "Service area: 20x20 km" << endl;
    
    cout << "\n=== ALL NODE COORDINATES ===" << endl;
    cout << "Node 0 (DEPOT): (" << data.coordinates[0].first << ", " << data.coordinates[0].second 
         << ") Type=" << data.nodeTypes[0] << " ReadyTime=" << data.readyTimes[0] << " [FIXED DEPOT FROM README]" << endl;
    for (int i = 1; i < data.numNodes; i++) {
        cout << "Node " << i << " (Customer): (" << data.coordinates[i].first << ", " << data.coordinates[i].second 
             << ") Type=" << data.nodeTypes[i] << " ReadyTime=" << data.readyTimes[i] << endl;
    }

    // 2. TEST DISTANCE MATRICES
    cout << "\n2. TESTING DISTANCE MATRICES..." << endl;
    if (data.distMatrix.empty()) {
        cerr << " Distance matrix not built!" << endl;
        return -1;
    }
    
    cout << "Distance matrix size: " << data.distMatrix.size() << "x" << data.distMatrix[0].size() << endl;
    
    // Sample distance check
    if (data.numNodes >= 3) {
        double truckDist = getTruckDistance(data, 1, 2);
        double droneDist = getDroneDistance(data, 1, 2);
        cout << "Distance (1->2): Truck=" << truckDist << " km, Drone=" << droneDist << " km" << endl;
    }

    // 3. TEST SIMPLE DECODING
    cout << "\n3. TESTING SEQUENCE DECODING..." << endl;
    
    // Test chromosome với encoding ĐÚNG: separator = numCustomers + 1 = 10 + 1 = 11
    vector<int> testChromosome = {1, 4, 3, 2, 9, 11, 7, 5, 6, 8, 10};
    cout << "\nTest chromosome: ";
    for (int id : testChromosome) cout << id << " ";
    cout << " (length: " << testChromosome.size() << ")" << endl;
    cout << "Separator should be: " << data.getSeparatorStart() << " (numCustomers+1 = " << data.numCustomers << "+1)" << endl;
    cout << "Number of customers: " << data.numCustomers << ", Total nodes: " << data.numNodes << endl;
    
    auto routes = decodeSeq(testChromosome, data);
    cout << "Decoded " << routes.size() << " routes:" << endl;
    for (int i = 0; i < routes.size(); i++) {
        cout << "   Route " << (i+1) << ": ";
        for (int nodeId : routes[i]) cout << nodeId << " ";
        cout << " (length: " << routes[i].size() << ")" << endl;
    }
    
    // 4. TEST EVALUATION & MAKESPAN CALCULATION
    cout << "\n4. TESTING EVALUATION & MAKESPAN..." << endl;
    
    cout << "\nTesting Evaluation Function..." << endl;
    auto evalStart = chrono::high_resolution_clock::now();
    PDPSolution solution = decodeAndEvaluate(testChromosome, data);
    auto evalEnd = chrono::high_resolution_clock::now();
    auto evalDuration = chrono::duration_cast<chrono::microseconds>(evalEnd - evalStart);
    
    cout << "Evaluation completed in " << evalDuration.count() << " microseconds" << endl;
    cout << "\n=== SOLUTION RESULTS ===" << endl;
    cout << "   - Makespan (C_max): " << fixed << setprecision(2) << solution.totalCost << " minutes" << endl;
    cout << "   - Total Penalty: " << solution.totalPenalty << endl;
    cout << "   - Feasible: " << (solution.isFeasible ? "YES" : "NO") << endl;
    
    printSolution(solution, data);

    cout << "\nENCODING TEST COMPLETED!" << endl;
    cout << "================================================" << endl;
    
    return 0;
}