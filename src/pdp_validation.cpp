#include "pdp_validation.h"
#include <iostream>
#include <iomanip>
#include <set>
#include <cmath>

using namespace std;

bool validateSolution(const PDPSolution& solution, const PDPData& data, bool verbose) {
    bool valid = true;
    
    if (verbose) {
        cout << "\n+========================================================+" << endl;
        cout << "|               SOLUTION VALIDATION                    |" << endl;
        cout << "+========================================================+" << endl;
    }
    
    // 1. Kiểm tra tất cả customers được phục vụ
    set<int> served_customers;
    for (const auto& truck : solution.truck_details) {
        for (int node : truck.route) {
            if (data.isCustomer(node)) {
                served_customers.insert(node);
            }
        }
    }
    
    if (verbose) {
        cout << "\n[1] CUSTOMER COVERAGE:" << endl;
        cout << "    Customers served: " << served_customers.size() << "/" << data.numCustomers << endl;
    }
    if ((int)served_customers.size() != data.numCustomers) {
        if (verbose) cout << "    ❌ MISSING CUSTOMERS!" << endl;
        valid = false;
    } else {
        if (verbose) cout << "    ✓ All customers served" << endl;
    }
    
    // 2. Kiểm tra drone endurance
    if (verbose) {
        cout << "\n[2] DRONE ENDURANCE (max " << fixed << setprecision(2) 
             << data.droneEndurance << " min):" << endl;
    }
    for (const auto& event : solution.resupply_events) {
        if (verbose) {
            cout << "    Trip to Node " << event.resupply_point 
                 << ": flight=" << fixed << setprecision(1) << event.total_flight_time << " min";
        }
        if (event.total_flight_time > data.droneEndurance) {
            if (verbose) cout << " EXCEEDED!" << endl;
            valid = false;
        } else {
            if (verbose) cout << " ✓" << endl;
        }
    }
    
    // 3. Kiểm tra timeline consistency
    if (verbose) cout << "\n[3] TIMELINE CONSISTENCY:" << endl;
    for (const auto& truck : solution.truck_details) {
        bool truck_valid = true;
        for (size_t i = 1; i < truck.arrival_times.size(); i++) {
            if (truck.arrival_times[i] < truck.departure_times[i-1] - 0.01) {
                truck_valid = false;
                break;
            }
        }
        if (verbose) {
            cout << "    Truck " << truck.truck_id << ": ";
            if (truck_valid) {
                cout << "✓ Timeline OK" << endl;
            } else {
                cout << "Timeline inconsistent!" << endl;
            }
        }
        if (!truck_valid) valid = false;
    }
    
    // 4. Kiểm tra drone-truck synchronization
    if (verbose) cout << "\n[4] DRONE-TRUCK SYNCHRONIZATION:" << endl;
    for (size_t i = 0; i < solution.resupply_events.size(); i++) {
        const auto& event = solution.resupply_events[i];
        if (verbose) {
            cout << "    Trip #" << (i+1) << " (Node " << event.resupply_point << "): ";
        }
        
        bool sync_ok = true;
        if (event.drone_arrive_time > event.resupply_start_time + 0.01) {
            if (verbose) cout << " Drone arrives AFTER resupply start!" << endl;
            sync_ok = false;
        } else if (event.truck_arrive_time > event.resupply_start_time + 0.01) {
            if (verbose) cout << " Truck arrives AFTER resupply start!" << endl;
            sync_ok = false;
        } else {
            if (verbose) cout << "✓ Sync OK" << endl;
        }
        if (!sync_ok) valid = false;
    }
    
    // 5. Kiểm tra C_max calculation
    if (verbose) cout << "\n[5] C_MAX VERIFICATION:" << endl;
    double calc_cmax = 0.0;
    for (const auto& truck : solution.truck_details) {
        calc_cmax = max(calc_cmax, truck.completion_time);
    }
    for (const auto& event : solution.resupply_events) {
        calc_cmax = max(calc_cmax, event.drone_return_time);
    }
    if (verbose) {
        cout << "    Reported C_max: " << fixed << setprecision(2) << solution.totalCost << " min" << endl;
        cout << "    Calculated C_max: " << calc_cmax << " min" << endl;
    }
    if (abs(calc_cmax - solution.totalCost) > 0.1) {
        if (verbose) cout << "    MISMATCH!" << endl;
        valid = false;
    } else {
        if (verbose) cout << " Match" << endl;
    }
    
    // 6. Đếm số lần truck quay về depot
    if (verbose) {
        cout << "\n[6] DEPOT RETURNS (efficiency check):" << endl;
        for (const auto& truck : solution.truck_details) {
            int depot_returns = 0;
            for (size_t i = 1; i < truck.route.size() - 1; i++) {
                if (truck.route[i] == data.depotIndex) {
                    depot_returns++;
                }
            }
            cout << "    Truck " << truck.truck_id << ": " << depot_returns << " depot returns";
            if (depot_returns > 0) {
                cout << " (could be optimized with more drone usage)";
            }
            cout << endl;
        }
    }
    
    if (verbose) {
        cout << "\n=========================================" << endl;
        cout << "VALIDATION RESULT: " << (valid ? "✓ VALID" : "INVALID") << endl;
        cout << "=========================================" << endl;
    }
    
    return valid;
}

void printSolutionSummary(const PDPSolution& solution, double costBeforeLS, double costAfterLS) {
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
}
