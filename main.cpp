#include "pdp_reader.h"
#include "pdp_init.h"
#include "pdp_utils.h"
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

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
    if (data.numNodes >= 2) {
        double truckDist = getTruckDistance(data, 1, 2);
        double droneDist = getDroneDistance(data, 1, 2);
        cout << "Distance (1->2): Truck=" << truckDist << " km, Drone=" << droneDist << " km" << endl;
    }

    // 3. TEST SIMPLE DECODING FIRST (Comment initialization để test decoding)
    cout << "\n3. TESTING SEQUENCE DECODING..." << endl;
    
    // Test chromosome với encoding mới: customers 1-10, separator 11  
    vector<int> testChromosome = {1, 4, 3, 2, 9, 11, 7, 5, 6, 8, 10};
    cout << "\nTest chromosome: ";
    for (int id : testChromosome) cout << id << " ";
    cout << " (length: " << testChromosome.size() << ")" << endl;
    
    auto routes = decodeSeq(testChromosome, data);
    cout << "Decoded " << routes.size() << " routes:" << endl;
    for (int i = 0; i < routes.size(); i++) {
        cout << "   Route " << (i+1) << ": ";
        for (int nodeId : routes[i]) cout << nodeId << " ";
        cout << " (length: " << routes[i].size() << ")" << endl;
    }
    
    cout << "\nSIMPLE DECODING TEST COMPLETED!" << endl;
    cout << "================================================" << endl;
    
    return 0;
}
    
/* COMMENT TẠM THỜI - SẼ UNCOMMENT SAU KHI DEBUG XONG

    auto start = chrono::high_resolution_clock::now();
    
    // Test Random initialization
    cout << "\n Testing Random Initialization..." << endl;
    auto randomPop = initRandomPDP(populationSize, data);
    cout << "Random initialization created " << randomPop.size() << " individuals" << endl;
    
    // Test Sweep initialization  
    cout << "\n Testing Sweep Initialization..." << endl;
    auto sweepPop = initSweepPDP(populationSize, data);
    cout << "Sweep initialization created " << sweepPop.size() << " individuals" << endl;
    
    // Test Greedy Time initialization
    cout << "\n Testing Greedy Time Initialization..." << endl;
    auto greedyPop = initGreedyTimePDP(populationSize, data);
    cout << "Greedy Time initialization created " << greedyPop.size() << " individuals" << endl;
    
    // Test Nearest Neighbor initialization
    cout << "\nTesting Nearest Neighbor Initialization..." << endl;
    auto nnPop = initNearestNeighborPDP(populationSize, data);
    cout << "Nearest Neighbor initialization created " << nnPop.size() << " individuals" << endl;
    
    // Test Structured Population
    cout << "\n Testing Structured Population Initialization..." << endl;
    auto structuredPop = initStructuredPopulationPDP(populationSize, data, 1);
    cout << "Structured initialization created " << structuredPop.size() << " individuals" << endl;

    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    cout << "\nTotal initialization time: " << duration.count() << " ms" << endl;

    // 4. TEST DECODING AND EVALUATION
    cout << "\n TESTING SEQUENCE DECODING & EVALUATION..." << endl;
    
    if (!randomPop.empty()) {
        vector<int> testSeq = randomPop[0]; // Lấy cá thể đầu tiên để test
        
        cout << "\n Test chromosome: ";
        for (int i = 0; i < min(20, (int)testSeq.size()); i++) {
            cout << testSeq[i] << " ";
            if (i == 19 && testSeq.size() > 20) cout << "...";
        }
        cout << " (length: " << testSeq.size() << ")" << endl;
        
        // Test decoding
        auto routes = decodeSeq(testSeq, data);
        cout << "Decoded " << routes.size() << " routes:" << endl;
        for (int i = 0; i < routes.size(); i++) {
            cout << "   Route " << (i+1) << ": ";
            for (int j = 0; j < min(10, (int)routes[i].size()); j++) {
                cout << routes[i][j] << " ";
                if (j == 9 && routes[i].size() > 10) cout << "...";
            }
            cout << " (length: " << routes[i].size() << ")" << endl;
        }
        
        // Test evaluation
        cout << "\nTesting Evaluation Function..." << endl;
        auto startEval = chrono::high_resolution_clock::now();
        PDPSolution solution = decodeAndEvaluate(testSeq, data);
        auto endEval = chrono::high_resolution_clock::now();
        auto evalDuration = chrono::duration_cast<chrono::microseconds>(endEval - startEval);
        
        cout << "Evaluation completed in " << evalDuration.count() << " " << endl;
        cout << "Results:" << endl;
        cout << "   - Makespan (C_max): " << solution.totalCost << " minutes" << endl;
        cout << "   - Total Penalty: " << solution.totalPenalty << endl;
        cout << "   - Feasible: " << (solution.isFeasible ? "YES" : "NO") << endl;
        
        // Print detailed solution
        printSolution(solution, data);
    }

    // 5. PERFORMANCE TESTING
    cout << "\n PERFORMANCE TESTING..." << endl;
    if (structuredPop.size() >= 3) {
        cout << "Evaluating first 3 individuals for performance test..." << endl;
        
        auto perfStart = chrono::high_resolution_clock::now();
        double totalCost = 0.0;
        int feasibleCount = 0;
        
        for (int i = 0; i < 3; i++) {
            PDPSolution sol = decodeAndEvaluate(structuredPop[i], data);
            totalCost += sol.totalCost;
            if (sol.isFeasible) feasibleCount++;
            cout << "   Individual " << (i+1) << ": C_max=" << sol.totalCost 
                 << ", Feasible=" << (sol.isFeasible ? "YES" : "NO") << endl;
        }
        
        auto perfEnd = chrono::high_resolution_clock::now();
        auto perfDuration = chrono::duration_cast<chrono::milliseconds>(perfEnd - perfStart);
        
        cout << "Performance Summary:" << endl;
        cout << "   - Average C_max: " << (totalCost / 3) << " minutes" << endl;
        cout << "   - Feasible solutions: " << feasibleCount << "/3" << endl;
        cout << "   - Total evaluation time: " << perfDuration.count() << " ms" << endl;
        cout << "   - Average per evaluation: " << (perfDuration.count() / 3.0) << " ms" << endl;
    }

    // 6. VALIDATION TESTING
    cout << "\nCONSTRAINT VALIDATION TESTING..." << endl;
    if (!structuredPop.empty()) {
        PDPSolution testSol = decodeAndEvaluate(structuredPop[0], data);
        bool isValid = validatePDPConstraints(testSol, data);
        cout << "PDP constraint validation: " << (isValid ? "PASSED" : "FAILED") << endl;
    }

    cout << "\nALL TESTS COMPLETED!" << endl;
    cout << "================================================" << endl;
    
    // Optional: Test with different file sizes
    cout << "\nTo test with different file sizes, modify filename in main.cpp:" << endl;
    cout << "   - Small: U_10_*.txt (10 customers)" << endl;
    cout << "   - Large: U_100_*.txt (100 customers)" << endl;
    
    return 0;
}