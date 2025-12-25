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
    cout << "\n+=============================================================+" << endl;
    cout << "|              SOLUTION DETAILS (Thoi gian thuc te)          |" << endl;
    cout << "+=============================================================+" << endl;
    
    cout << "\n TONG QUAN:" << endl;
    cout << "   Total Cost (C_max): " << fixed << setprecision(2) << solution.totalCost << " phut" << endl;
    cout << "   Feasible: " << (solution.isFeasible ? "YES" : " NO") << endl;
    
    // In chi tiet tung xe
    cout << "\nCHI TIET CAC XE TAI:" << endl;
    for (const auto& truck_info : solution.truck_details) {
        cout << "\n   +-- Xe " << truck_info.truck_id << " --------------------------------------" << endl;
        cout << "   | Hoan thanh: " << fixed << setprecision(2) << truck_info.completion_time << " phut" << endl;
        cout << "   | Route: ";
        
        for (size_t i = 0; i < truck_info.route.size(); i++) {
            int nodeIdx = truck_info.route[i];
            if (nodeIdx >= 0 && nodeIdx < data.nodeTypes.size()) {
                cout << nodeIdx << "(" << data.nodeTypes[nodeIdx] << ")";
            } else {
                cout << nodeIdx;
            }
            if (i < truck_info.route.size() - 1) cout << " -> ";
        }
        cout << endl;
        
        cout << "   | Timeline:" << endl;
        for (size_t i = 0; i < truck_info.route.size(); i++) {
            int nodeIdx = truck_info.route[i];
            string nodeType = (nodeIdx >= 0 && nodeIdx < data.nodeTypes.size()) ? data.nodeTypes[nodeIdx] : "?";
            
            cout << "   |   " << setw(2) << i << ". Node " << setw(2) << nodeIdx 
                 << " (" << setw(2) << nodeType << "): ";
            cout << "Den=" << setw(6) << fixed << setprecision(1) << truck_info.arrival_times[i] << "' ";
            cout << "Roi=" << setw(6) << fixed << setprecision(1) << truck_info.departure_times[i] << "'" << endl;
        }
        cout << "   +-----------------------------------------------------" << endl;
    }
    
    // In chi tiet drone resupply
    if (!solution.resupply_events.empty()) {
        cout << "\nCHI TIET DRONE RESUPPLY (WITH CONSOLIDATION):" << endl;
        for (size_t i = 0; i < solution.resupply_events.size(); i++) {
            const auto& event = solution.resupply_events[i];
            cout << "\n   +-- Resupply Trip #" << (i + 1) << " ----------------------------------" << endl;
            cout << "   | Drone: Drone " << event.drone_id << " | Xe: Xe " << event.truck_id << endl;
            cout << "   | So khach hang: " << event.customer_ids.size() << " customers" << endl;
            cout << "   | Customers: ";
            for (size_t j = 0; j < event.customer_ids.size(); j++) {
                cout << "Node " << event.customer_ids[j];
                if (j < event.customer_ids.size() - 1) cout << ", ";
            }
            cout << endl;
            cout << "   | Total flight time: " << fixed << setprecision(1) << event.total_flight_time << " phut" << endl;
            cout << "   | Timeline:" << endl;
            cout << "   |   - Drone roi depot:     " << setw(6) << fixed << setprecision(1) << event.drone_depart_time << " phut" << endl;
            
            // ONE RENDEZVOUS: Chi 1 diem hen duy nhat
            cout << "   |   [Resupply Point: Customer " << event.resupply_point << "]" << endl;
            cout << "   |     - Drone den:        " << setw(6) << event.drone_arrive_time << " phut" << endl;
            cout << "   |     - Xe den:           " << setw(6) << event.truck_arrive_time << " phut" << endl;
            cout << "   |     - Bat dau resupply: " << setw(6) << event.resupply_start_time << " phut" << endl;
            cout << "   |     - Ket thuc:         " << setw(6) << event.resupply_end_time << " phut" << endl;
            cout << "   |     - Truck giao xong:  " << setw(6) << event.truck_delivery_end << " phut" << endl;
            
            cout << "   |   - Drone ve depot:      " << setw(6) << event.drone_return_time << " phut" << endl;
            cout << "   +-----------------------------------------------------" << endl;
        }
    }
    
    // In thoi gian hoan thanh cua tung drone
    if (!solution.drone_completion_times.empty()) {
        cout << "\n THOI GIAN HOAN THANH CAC DRONE:" << endl;
        for (size_t i = 0; i < solution.drone_completion_times.size(); i++) {
            cout << "   Drone " << i << ": " << fixed << setprecision(2) 
                 << solution.drone_completion_times[i] << " phut" << endl;
        }
    }
    
    cout << "\n=============================================================" << endl;
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
