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
    cout << "\n╔══════════════════════════════════════════════════════════════╗" << endl;
    cout << "║              SOLUTION DETAILS (Thời gian thực tế)           ║" << endl;
    cout << "╚══════════════════════════════════════════════════════════════╝" << endl;
    
    cout << "\n TỔNG QUAN:" << endl;
    cout << "   Total Cost (C_max): " << fixed << setprecision(2) << solution.totalCost << " phút" << endl;
    cout << "   Feasible: " << (solution.isFeasible ? "YES" : " NO") << endl;
    
    // In chi tiết từng xe
    cout << "\nCHI TIẾT CÁC XE TẢI:" << endl;
    for (const auto& truck_info : solution.truck_details) {
        cout << "\n   ┌─ Xe " << truck_info.truck_id << " ─────────────────────────────────" << endl;
        cout << "   │ Hoàn thành: " << fixed << setprecision(2) << truck_info.completion_time << " phút" << endl;
        cout << "   │ Route: ";
        
        for (size_t i = 0; i < truck_info.route.size(); i++) {
            int nodeIdx = truck_info.route[i];
            if (nodeIdx >= 0 && nodeIdx < data.nodeTypes.size()) {
                cout << nodeIdx << "(" << data.nodeTypes[nodeIdx] << ")";
            } else {
                cout << nodeIdx;
            }
            if (i < truck_info.route.size() - 1) cout << " → ";
        }
        cout << endl;
        
        cout << "   │ Timeline:" << endl;
        for (size_t i = 0; i < truck_info.route.size(); i++) {
            int nodeIdx = truck_info.route[i];
            string nodeType = (nodeIdx >= 0 && nodeIdx < data.nodeTypes.size()) ? data.nodeTypes[nodeIdx] : "?";
            
            cout << "   │   " << setw(2) << i << ". Node " << setw(2) << nodeIdx 
                 << " (" << setw(2) << nodeType << "): ";
            cout << "Đến=" << setw(6) << fixed << setprecision(1) << truck_info.arrival_times[i] << "' ";
            cout << "Rời=" << setw(6) << fixed << setprecision(1) << truck_info.departure_times[i] << "'" << endl;
        }
        cout << "   └─────────────────────────────────────────────────" << endl;
    }
    
    // In chi tiết drone resupply
    if (!solution.resupply_events.empty()) {
        cout << "\nCHI TIẾT DRONE RESUPPLY:" << endl;
        for (size_t i = 0; i < solution.resupply_events.size(); i++) {
            const auto& event = solution.resupply_events[i];
            cout << "\n   ┌─ Resupply #" << (i + 1) << " ─────────────────────────────────" << endl;
            cout << "   │ Khách hàng: Node " << event.customer_id << endl;
            cout << "   │ Drone: Drone " << event.drone_id << endl;
            cout << "   │ Gặp xe: Xe " << event.truck_id << endl;
            cout << "   │ Timeline:" << endl;
            cout << "   │   - Drone rời depot:     " << setw(6) << fixed << setprecision(1) << event.drone_depart_time << " phút" << endl;
            cout << "   │   - Drone đến khách:     " << setw(6) << event.drone_arrive_time << " phút" << endl;
            cout << "   │   - Xe đến khách:        " << setw(6) << event.truck_arrive_time << " phút" << endl;
            cout << "   │   - Bắt đầu resupply:    " << setw(6) << event.resupply_start << " phút" << endl;
            cout << "   │   - Kết thúc resupply:   " << setw(6) << event.resupply_end << " phút" << endl;
            cout << "   │   - Drone về depot:      " << setw(6) << event.drone_return_time << " phút" << endl;
            cout << "   └─────────────────────────────────────────────────" << endl;
        }
    }
    
    // In thời gian hoàn thành của từng drone
    if (!solution.drone_completion_times.empty()) {
        cout << "\n THỜI GIAN HOÀN THÀNH CÁC DRONE:" << endl;
        for (size_t i = 0; i < solution.drone_completion_times.size(); i++) {
            cout << "   Drone " << i << ": " << fixed << setprecision(2) 
                 << solution.drone_completion_times[i] << " phút" << endl;
        }
    }
    
    cout << "\n═══════════════════════════════════════════════════════════════" << endl;
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