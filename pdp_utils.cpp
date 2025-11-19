#include "pdp_utils.h"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <map>
using namespace std;

double euclideanDistance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

bool validatePDPConstraints(const PDPSolution& solution, const PDPData& data) {
    // Simple validation: check if all P comes before corresponding DL
    for (const auto& route : solution.routes) {
        map<int, int> pickupPos, deliveryPos;
        
        for (int i = 0; i < route.size(); i++) {
            int nodeIdx = route[i] - 1; // Convert to 0-based
            if (nodeIdx >= 0 && nodeIdx < data.nodeTypes.size()) {
                string type = data.nodeTypes[nodeIdx];
                int pairId = data.pairIds[nodeIdx];
                
                if (type == "P") {
                    pickupPos[pairId] = i;
                } else if (type == "DL") {
                    deliveryPos[pairId] = i;
                }
            }
        }
        
        // Check precedence constraint
        for (const auto& pickup : pickupPos) {
            int pairId = pickup.first;
            int pickupPosition = pickup.second;
            
            if (deliveryPos.find(pairId) != deliveryPos.end()) {
                int deliveryPosition = deliveryPos[pairId];
                if (pickupPosition >= deliveryPosition) {
                    return false; // P must come before DL
                }
            }
        }
    }
    
    return true;
}

void printSolution(const PDPSolution& solution, const PDPData& data) {
    cout << "\nSolution Routes:" << endl;
    cout << "Total cost: " << fixed << setprecision(2) << solution.totalCost << endl;
    cout << "Feasible: " << (solution.isFeasible ? "YES" : "NO") << endl;
    
    for (int i = 0; i < solution.routes.size(); i++) {
        cout << "Route " << (i + 1) << ": ";
        for (int j = 0; j < solution.routes[i].size(); j++) {
            int nodeId = solution.routes[i][j];
            cout << nodeId;
            if (nodeId > 0 && nodeId <= data.nodeTypes.size()) {
                cout << "(" << data.nodeTypes[nodeId - 1] << ")";
            }
            if (j < solution.routes[i].size() - 1) cout << " -> ";
        }
        cout << endl;
    }
}

double calculateSolutionCost(const PDPSolution& solution, const vector<vector<double>>& distMatrix) {
    double totalCost = 0.0;
    
    for (const auto& route : solution.routes) {
        for (int i = 0; i < route.size() - 1; i++) {
            int from = route[i] - 1; // Convert to 0-based
            int to = route[i + 1] - 1;
            
            if (from >= 0 && from < distMatrix.size() && 
                to >= 0 && to < distMatrix[from].size()) {
                totalCost += distMatrix[from][to];
            }
        }
    }
    
    return totalCost;
}