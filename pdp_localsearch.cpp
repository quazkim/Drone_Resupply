#include "pdp_localsearch.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>
#include <climits>
#include <set>
#include <map>
#include <numeric>

using namespace std;

// ============ CONSTRUCTOR ============

IntegratedLocalSearch::IntegratedLocalSearch(const PDPData& data, int maxIterations)
    : data(data), maxIterations(maxIterations) {
    random_device rd;
    rng = mt19937(rd());
    initOperatorStats();
}

// ============ ADAPTIVE OPERATOR SELECTION ============

void IntegratedLocalSearch::initOperatorStats() {
    // Initialize all operators with equal weight
    for (int i = 0; i < static_cast<int>(OperatorType::NUM_OPERATORS); ++i) {
        operatorStats[static_cast<OperatorType>(i)] = OperatorStats();
    }
}

void IntegratedLocalSearch::updateOperatorStats(OperatorType op, bool success, double improvement) {
    auto& stats = operatorStats[op];
    stats.attempts++;
    if (success) {
        stats.successes++;
        stats.total_improvement += improvement;
    }
}

void IntegratedLocalSearch::updateOperatorWeights() {
    // Adaptive weight update based on success rate and improvement
    // Formula: weight = base_weight + success_bonus + improvement_bonus
    
    const double base_weight = 0.5;
    const double success_factor = 2.0;
    const double improvement_factor = 0.1;
    const double min_weight = 0.1;
    const double max_weight = 5.0;
    
    for (auto& pair : operatorStats) {
        OperatorType op = pair.first;
        OperatorStats& stats = pair.second;
        if (stats.attempts > 0) {
            double success_rate = stats.getSuccessRate();
            double avg_improvement = stats.getAvgImprovement();
            
            // Calculate new weight
            double new_weight = base_weight 
                              + success_factor * success_rate 
                              + improvement_factor * avg_improvement;
            
            // Clamp to reasonable range
            stats.weight = max(min_weight, min(max_weight, new_weight));
        }
    }
}

OperatorType IntegratedLocalSearch::selectOperator(const vector<OperatorType>& operators) {
    // Roulette wheel selection based on weights
    double total_weight = 0.0;
    for (OperatorType op : operators) {
        total_weight += operatorStats[op].weight;
    }
    
    uniform_real_distribution<double> dist(0.0, total_weight);
    double r = dist(rng);
    
    double cumulative = 0.0;
    for (OperatorType op : operators) {
        cumulative += operatorStats[op].weight;
        if (r <= cumulative) {
            return op;
        }
    }
    
    return operators.back(); // Fallback
}

string IntegratedLocalSearch::getOperatorName(OperatorType op) const {
    switch (op) {
        case OperatorType::TRUCK_2OPT: return "Truck-2Opt";
        case OperatorType::TRUCK_SWAP: return "Truck-Swap";
        case OperatorType::TRUCK_RELOCATE: return "Truck-Relocate";
        case OperatorType::TRUCK_CROSS_EXCHANGE: return "Truck-CrossExchange";
        case OperatorType::DRONE_REORDER: return "Drone-Reorder";
        case OperatorType::DRONE_SWAP: return "Drone-Swap";
        case OperatorType::DRONE_MOVE: return "Drone-Move";
        case OperatorType::DRONE_MERGE: return "Drone-Merge";
        case OperatorType::DRONE_SPLIT: return "Drone-Split";
        case OperatorType::DRONE_REASSIGN: return "Drone-Reassign";
        case OperatorType::DRONE_INSERT_INTO_TRIP: return "Drone-InsertIntoTrip";
        default: return "Unknown";
    }
}

void IntegratedLocalSearch::printOperatorStats() const {
    cout << "\n[ADAPTIVE] Operator Performance Statistics:" << endl;
    cout << "-------------------------------------------------------------" << endl;
    cout << setw(22) << "Operator" << setw(10) << "Attempts" << setw(10) << "Success" 
         << setw(12) << "Rate(%)" << setw(12) << "AvgImprv" << setw(10) << "Weight" << endl;
    cout << "-------------------------------------------------------------" << endl;
    
    for (const auto& pair : operatorStats) {
        OperatorType op = pair.first;
        const OperatorStats& stats = pair.second;
        if (stats.attempts > 0) {
            cout << setw(22) << getOperatorName(op) 
                 << setw(10) << stats.attempts
                 << setw(10) << stats.successes
                 << setw(11) << fixed << setprecision(1) << (stats.getSuccessRate() * 100) << "%"
                 << setw(12) << setprecision(2) << stats.getAvgImprovement()
                 << setw(10) << setprecision(2) << stats.weight << endl;
        }
    }
    cout << "-------------------------------------------------------------" << endl;
}

// ============ HELPER FUNCTIONS ============

double IntegratedLocalSearch::getTruckTravelTime(int from, int to) const {
    if (from < 0 || from >= data.numNodes || to < 0 || to >= data.numNodes)
        return numeric_limits<double>::infinity();
    return data.truckDistMatrix[from][to] / data.truckSpeed * 60.0;
}

double IntegratedLocalSearch::getDroneTravelTime(int from, int to) const {
    if (from < 0 || from >= data.numNodes || to < 0 || to >= data.numNodes)
        return numeric_limits<double>::infinity();
    return data.droneDistMatrix[from][to] / data.droneSpeed * 60.0;
}

bool IntegratedLocalSearch::isDroneEligible(int node_id) const {
    if (node_id < 0 || node_id >= data.numNodes) return false;
    return (data.nodeTypes[node_id] == "D" && data.readyTimes[node_id] > 0);
}

double IntegratedLocalSearch::calculateCmax(const PDPSolution& sol) const {
    double cmax = 0.0;
    
    // Max truck completion time
    for (const auto& truck : sol.truck_details) {
        cmax = max(cmax, truck.completion_time);
    }
    
    // Max drone completion time
    for (const auto& event : sol.resupply_events) {
        cmax = max(cmax, event.drone_return_time);
    }
    
    return cmax;
}

void IntegratedLocalSearch::recalculateTruckTimes(PDPSolution& sol) {
    // NOTE: Truck times may need to wait for drone at resupply points
    // This is a preliminary calculation - will be updated in recalculateDroneTimes()
    // 
    // IMPORTANT: ready_time l├á thß╗¥i gian h├áng xuß║Ñt hiß╗çn tß║íi DEPOT (kh├┤ng phß║úi tß║íi customer)
    // Truck chß╗ë cß║ºn ─æß╗úi ready_time KHI Lß║ñY H├ÇNG Tß║áI DEPOT, kh├┤ng phß║úi khi ─æß║┐n customer
    
    for (auto& truck : sol.truck_details) {
        if (truck.route.size() < 2) continue;
        
        truck.arrival_times.clear();
        truck.departure_times.clear();
        
        double current_time = 0.0;
        int current_pos = data.depotIndex;
        
        truck.arrival_times.push_back(0.0);
        truck.departure_times.push_back(0.0);
        
        for (size_t i = 1; i < truck.route.size(); ++i) {
            int node = truck.route[i];
            
            // Travel to node
            double travel_time = getTruckTravelTime(current_pos, node);
            double arrival = current_time + travel_time;
            
            // Service time depends on node type
            double service_time = 0.0;
            
            if (node == data.depotIndex) {
                service_time = data.depotReceiveTime;
            } else if (data.isCustomer(node)) {
                // Check if this is a resupply point (type D customer)
                bool is_resupply = false;
                for (const auto& event : sol.resupply_events) {
                    for (int cust : event.customer_ids) {
                        if (cust == node && event.truck_id == truck.truck_id) {
                            is_resupply = true;
                            break;
                        }
                    }
                    if (is_resupply) break;
                }
                
                if (is_resupply) {
                    service_time = data.resupplyTime + data.truckServiceTime;
                } else {
                    service_time = data.truckServiceTime;
                }
            }
            
            // Truck kh├┤ng cß║ºn ─æß╗úi ready_time tß║íi customer
            // ready_time chß╗ë ß║únh h╞░ß╗ƒng khi lß║Ñy h├áng tß╗½ depot (xß╗¡ l├╜ ri├¬ng)
            double start_time = arrival;
            double depart_time = start_time + service_time;
            
            truck.arrival_times.push_back(arrival);
            truck.departure_times.push_back(depart_time);
            
            current_time = depart_time;
            current_pos = node;
        }
        
        truck.completion_time = current_time;
    }
}

void IntegratedLocalSearch::recalculateDroneTimes(PDPSolution& sol) {
    // First recalculate truck times (preliminary, may need adjustment)
    recalculateTruckTimes(sol);
    
    vector<double> drone_available(data.numDrones, 0.0);
    
    // Track truck wait times at resupply points
    map<pair<int,int>, double> truck_wait_times; // <truck_id, node> -> additional wait time
    
    for (auto& event : sol.resupply_events) {
        if (event.customer_ids.empty()) continue;
        
        int drone_id = event.drone_id;
        int truck_id = event.truck_id;
        
        if (truck_id < 0 || truck_id >= sol.truck_details.size()) continue;
        
        // Find truck state when it reaches first customer
        double truck_time = 0.0;
        int truck_pos = data.depotIndex;
        
        const auto& truck = sol.truck_details[truck_id];
        
        // Find when truck reaches first customer in this trip
        for (size_t i = 0; i < truck.route.size(); ++i) {
            if (truck.route[i] == event.customer_ids[0]) {
                truck_time = truck.arrival_times[i];
                truck_pos = (i > 0) ? truck.route[i-1] : data.depotIndex;
                break;
            }
        }
        
        // Calculate max ready time
        double max_ready = 0.0;
        for (int cust : event.customer_ids) {
            max_ready = max(max_ready, (double)data.readyTimes[cust]);
        }
        
        // Drone departure
        double drone_ready = max(drone_available[drone_id], max_ready);
        event.drone_depart_time = drone_ready + data.depotDroneLoadTime;
        
        // Clear and recalculate arrival times
        event.arrive_times.clear();
        event.truck_arrive_times.clear();
        event.resupply_starts.clear();
        event.resupply_ends.clear();
        
        double drone_time = event.drone_depart_time;
        double total_flight = 0.0;
        int current_pos = data.depotIndex;
        double current_truck_time = truck_time;
        int current_truck_pos = truck_pos;
        
        for (int cust : event.customer_ids) {
            // Drone flies to customer
            double fly_time = getDroneTravelTime(current_pos, cust);
            drone_time += fly_time;
            total_flight += fly_time;
            
            // Truck travels to customer
            double truck_travel = getTruckTravelTime(current_truck_pos, cust);
            double truck_arr = current_truck_time + truck_travel;
            
            // Rendezvous: Truck v├á drone gß║╖p nhau tß║íi customer
            // KH├öNG cß║ºn ─æß╗úi ready_time tß║íi customer (ready_time ─æ├ú ─æ╞░ß╗úc xß╗¡ l├╜ khi drone lß║Ñy h├áng tß╗½ depot)
            double resupply_start = max(drone_time, truck_arr);
            double wait_time = resupply_start - drone_time;
            total_flight += wait_time;
            
            double resupply_end = resupply_start + data.resupplyTime + data.truckServiceTime;
            
            event.arrive_times.push_back(drone_time);
            event.truck_arrive_times.push_back(truck_arr);
            event.resupply_starts.push_back(resupply_start);
            event.resupply_ends.push_back(resupply_end);
            
            drone_time = resupply_end;
            total_flight += data.resupplyTime;
            current_pos = cust;
            current_truck_time = resupply_end;
            current_truck_pos = cust;
        }
        
        // Drone returns to depot
        double return_time = getDroneTravelTime(current_pos, data.depotIndex);
        total_flight += return_time;
        event.drone_return_time = drone_time + return_time;
        event.total_flight_time = total_flight;
        
        drone_available[drone_id] = event.drone_return_time;
        
        // Update truck times if truck had to wait for drone
        if (truck_id >= 0 && truck_id < sol.truck_details.size()) {
            auto& truck = sol.truck_details[truck_id];
            
            for (size_t j = 0; j < event.customer_ids.size(); ++j) {
                int cust = event.customer_ids[j];
                double resupply_end = event.resupply_ends[j];
                
                // Find this customer in truck route and update times
                for (size_t k = 0; k < truck.route.size(); ++k) {
                    if (truck.route[k] == cust) {
                        // Truck must wait until resupply_end
                        if (resupply_end > truck.departure_times[k]) {
                            double delay = resupply_end - truck.departure_times[k];
                            truck.departure_times[k] = resupply_end;
                            
                            // Propagate delay to subsequent nodes
                            for (size_t m = k + 1; m < truck.route.size(); ++m) {
                                truck.arrival_times[m] += delay;
                                truck.departure_times[m] += delay;
                            }
                        }
                        break;
                    }
                }
            }
            
            // Update truck completion time
            if (!truck.departure_times.empty()) {
                truck.completion_time = truck.departure_times.back();
            }
        }
    }
}

bool IntegratedLocalSearch::isTruckRouteFeasible(const vector<int>& route, int truck_id) const {
    if (route.size() < 2) return true;
    
    double current_load = 0.0;
    set<int> picked_up_pairs;
    
    for (size_t i = 1; i < route.size() - 1; ++i) {
        int node = route[i];
        if (node == data.depotIndex) {
            current_load = 0.0;
            picked_up_pairs.clear();
            continue;
        }
        
        if (!data.isCustomer(node)) continue;
        
        string type = data.nodeTypes[node];
        int demand = data.demands[node];
        int pair_id = data.pairIds[node];
        
        // Check P-DL precedence
        if (type == "DL" && pair_id > 0) {
            if (picked_up_pairs.find(pair_id) == picked_up_pairs.end()) {
                return false; // DL before P
            }
            picked_up_pairs.erase(pair_id);
        }
        
        if (type == "P" && pair_id > 0) {
            picked_up_pairs.insert(pair_id);
        }
        
        current_load += demand;
        
        if (current_load > data.truckCapacity || current_load < -0.01) {
            return false;
        }
    }
    
    return true;
}

bool IntegratedLocalSearch::isDroneTripFeasible(const ResupplyEvent& trip, const PDPSolution& sol) const {
    if (trip.customer_ids.empty()) return true;
    
    // Check capacity
    int drone_capacity = data.getDroneCapacity();
    if ((int)trip.customer_ids.size() > drone_capacity) return false;
    
    // Check all customers are type D
    for (int cust : trip.customer_ids) {
        if (!isDroneEligible(cust)) return false;
    }
    
    // Check consolidation constraints (ready time within 30 min, distance < 10km)
    if (trip.customer_ids.size() > 1) {
        int first_ready = data.readyTimes[trip.customer_ids[0]];
        for (size_t i = 1; i < trip.customer_ids.size(); ++i) {
            int cust = trip.customer_ids[i];
            int ready = data.readyTimes[cust];
            
            // Ready time constraint
            if (abs(ready - first_ready) > 30) return false;
            
            // Distance constraint from first customer
            double dist = data.droneDistMatrix[trip.customer_ids[0]][cust];
            if (dist > 10.0) return false;
        }
    }
    
    // Check endurance
    if (trip.total_flight_time > data.droneEndurance) return false;
    
    return true;
}

double IntegratedLocalSearch::evaluateDroneTripTime(
    const vector<int>& customers,
    int drone_id,
    int truck_id,
    double truck_available_time,
    int truck_position) const 
{
    if (customers.empty()) return numeric_limits<double>::max();
    
    double max_ready = 0.0;
    for (int cust : customers) {
        max_ready = max(max_ready, (double)data.readyTimes[cust]);
    }
    
    double drone_time = max_ready + data.depotDroneLoadTime;
    double total_flight = 0.0;
    int current_pos = data.depotIndex;
    double truck_time = truck_available_time;
    int truck_pos = truck_position;
    
    for (int cust : customers) {
        double fly_time = getDroneTravelTime(current_pos, cust);
        drone_time += fly_time;
        total_flight += fly_time;
        
        double truck_travel = getTruckTravelTime(truck_pos, cust);
        double truck_arr = truck_time + truck_travel;
        
        // Rendezvous: chß╗ë ─æß╗úi drone v├á truck gß║╖p nhau
        // KH├öNG ─æß╗úi ready_time tß║íi customer (─æ├ú xß╗¡ l├╜ khi drone lß║Ñy h├áng tß╗½ depot)
        double start = max(drone_time, truck_arr);
        double wait = start - drone_time;
        total_flight += wait;
        
        drone_time = start + data.resupplyTime + data.truckServiceTime;
        total_flight += data.resupplyTime;
        
        current_pos = cust;
        truck_time = drone_time;
        truck_pos = cust;
    }
    
    double return_time = getDroneTravelTime(current_pos, data.depotIndex);
    total_flight += return_time;
    
    // Check endurance
    if (total_flight > data.droneEndurance) {
        return numeric_limits<double>::max();
    }
    
    return drone_time + return_time;
}

// ============ TRUCK LOCAL SEARCH OPERATORS ============

bool IntegratedLocalSearch::truck2Opt(PDPSolution& sol) {
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    
    for (auto& truck : sol.truck_details) {
        if (truck.route.size() < 4) continue; // Need at least depot-a-b-depot
        
        // Try all 2-opt moves (excluding depot)
        for (size_t i = 1; i < truck.route.size() - 2; ++i) {
            for (size_t j = i + 1; j < truck.route.size() - 1; ++j) {
                // Reverse segment [i, j]
                vector<int> new_route = truck.route;
                reverse(new_route.begin() + i, new_route.begin() + j + 1);
                
                // Check feasibility
                if (!isTruckRouteFeasible(new_route, truck.truck_id)) continue;
                
                // Evaluate
                vector<int> old_route = truck.route;
                truck.route = new_route;
                recalculateTruckTimes(sol);
                recalculateDroneTimes(sol);
                double new_cmax = calculateCmax(sol);
                
                if (new_cmax < best_cmax - 0.01) {
                    best_cmax = new_cmax;
                    improved = true;
                    return true; // First improvement
                } else {
                    truck.route = old_route;
                    recalculateTruckTimes(sol);
                    recalculateDroneTimes(sol);
                }
            }
        }
    }
    
    return improved;
}

bool IntegratedLocalSearch::truckOrOpt(PDPSolution& sol) {
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    
    for (auto& truck : sol.truck_details) {
        if (truck.route.size() < 4) continue;
        
        // Only try moving single nodes (seg_len = 1) for efficiency
        for (size_t i = 1; i < truck.route.size() - 1; ++i) {
            for (size_t j = 1; j < truck.route.size() - 1; ++j) {
                if (j == i || j == i - 1 || j == i + 1) continue;
                
                vector<int> new_route;
                int node = truck.route[i];
                
                // Build new route by moving node from i to after j
                for (size_t k = 0; k < truck.route.size(); ++k) {
                    if (k == i) continue;
                    new_route.push_back(truck.route[k]);
                    if ((j < i && new_route.size() == j + 1) || 
                        (j > i && new_route.size() == j)) {
                        new_route.push_back(node);
                    }
                }
                
                if (new_route.size() != truck.route.size()) continue;
                if (!isTruckRouteFeasible(new_route, truck.truck_id)) continue;
                
                vector<int> old_route = truck.route;
                truck.route = new_route;
                recalculateTruckTimes(sol);
                recalculateDroneTimes(sol);
                double new_cmax = calculateCmax(sol);
                
                if (new_cmax < best_cmax - 0.01) {
                    best_cmax = new_cmax;
                    improved = true;
                    // First improvement: return immediately
                    return true;
                } else {
                    truck.route = old_route;
                    recalculateTruckTimes(sol);
                    recalculateDroneTimes(sol);
                }
            }
        }
    }
    
    return improved;
}

bool IntegratedLocalSearch::truckSwap(PDPSolution& sol) {
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    
    for (auto& truck : sol.truck_details) {
        if (truck.route.size() < 4) continue;
        
        for (size_t i = 1; i < truck.route.size() - 1; ++i) {
            for (size_t j = i + 1; j < truck.route.size() - 1; ++j) {
                // Swap nodes at i and j
                vector<int> new_route = truck.route;
                swap(new_route[i], new_route[j]);
                
                if (!isTruckRouteFeasible(new_route, truck.truck_id)) continue;
                
                vector<int> old_route = truck.route;
                truck.route = new_route;
                recalculateTruckTimes(sol);
                recalculateDroneTimes(sol);
                double new_cmax = calculateCmax(sol);
                
                if (new_cmax < best_cmax - 0.01) {
                    best_cmax = new_cmax;
                    improved = true;
                    return true; // First improvement
                } else {
                    truck.route = old_route;
                    recalculateTruckTimes(sol);
                    recalculateDroneTimes(sol);
                }
            }
        }
    }
    
    return improved;
}

bool IntegratedLocalSearch::truckRelocate(PDPSolution& sol) {
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    
    for (auto& truck : sol.truck_details) {
        if (truck.route.size() < 4) continue;
        
        for (size_t i = 1; i < truck.route.size() - 1; ++i) {
            int node = truck.route[i];
            
            for (size_t j = 1; j < truck.route.size() - 1; ++j) {
                if (j == i || j == i - 1) continue;
                
                // Move node from position i to after position j
                vector<int> new_route;
                for (size_t k = 0; k < truck.route.size(); ++k) {
                    if (k == i) continue;
                    new_route.push_back(truck.route[k]);
                    if (new_route.size() == j + 1 || (j > i && new_route.size() == j)) {
                        new_route.push_back(node);
                    }
                }
                
                if (new_route.size() != truck.route.size()) continue;
                if (!isTruckRouteFeasible(new_route, truck.truck_id)) continue;
                
                vector<int> old_route = truck.route;
                truck.route = new_route;
                recalculateTruckTimes(sol);
                recalculateDroneTimes(sol);
                double new_cmax = calculateCmax(sol);
                
                if (new_cmax < best_cmax - 0.01) {
                    best_cmax = new_cmax;
                    improved = true;
                } else {
                    truck.route = old_route;
                    recalculateTruckTimes(sol);
                    recalculateDroneTimes(sol);
                }
            }
        }
    }
    
    return improved;
}

bool IntegratedLocalSearch::truckCrossExchange(PDPSolution& sol) {
    // Simplified: just try swapping first movable node
    if (sol.truck_details.size() < 2) return false;
    
    double best_cmax = calculateCmax(sol);
    
    for (size_t t1 = 0; t1 < sol.truck_details.size(); ++t1) {
        for (size_t t2 = t1 + 1; t2 < sol.truck_details.size(); ++t2) {
            auto& truck1 = sol.truck_details[t1];
            auto& truck2 = sol.truck_details[t2];
            
            if (truck1.route.size() < 3 || truck2.route.size() < 3) continue;
            
            // Just try first node swap
            vector<int> new_route1 = truck1.route;
            vector<int> new_route2 = truck2.route;
            
            swap(new_route1[1], new_route2[1]);
            
            if (!isTruckRouteFeasible(new_route1, truck1.truck_id)) continue;
            if (!isTruckRouteFeasible(new_route2, truck2.truck_id)) continue;
            
            vector<int> old_route1 = truck1.route;
            vector<int> old_route2 = truck2.route;
            
            truck1.route = new_route1;
            truck2.route = new_route2;
            recalculateTruckTimes(sol);
            recalculateDroneTimes(sol);
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                return true; // First improvement
            } else {
                truck1.route = old_route1;
                truck2.route = old_route2;
                recalculateTruckTimes(sol);
                recalculateDroneTimes(sol);
            }
        }
    }
    
    return false;
}

// ============ DRONE LOCAL SEARCH OPERATORS ============

bool IntegratedLocalSearch::droneMergeTrips(PDPSolution& sol) {
    if (sol.resupply_events.size() < 2) return false;
    
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    int drone_capacity = data.getDroneCapacity();
    
    // Try merging pairs of trips
    for (size_t i = 0; i < sol.resupply_events.size(); ++i) {
        for (size_t j = i + 1; j < sol.resupply_events.size(); ++j) {
            auto& trip1 = sol.resupply_events[i];
            auto& trip2 = sol.resupply_events[j];
            
            // Check if merge is possible (same truck, capacity ok)
            if (trip1.truck_id != trip2.truck_id) continue;
            if (trip1.customer_ids.size() + trip2.customer_ids.size() > drone_capacity) continue;
            
            // Create merged trip
            ResupplyEvent merged;
            merged.customer_ids = trip1.customer_ids;
            merged.customer_ids.insert(merged.customer_ids.end(), 
                                       trip2.customer_ids.begin(), 
                                       trip2.customer_ids.end());
            merged.drone_id = trip1.drone_id;
            merged.truck_id = trip1.truck_id;
            
            // Check consolidation constraints
            if (!isDroneTripFeasible(merged, sol)) continue;
            
            // Apply merge and evaluate
            vector<ResupplyEvent> old_events = sol.resupply_events;
            sol.resupply_events.erase(sol.resupply_events.begin() + j);
            sol.resupply_events[i] = merged;
            
            recalculateDroneTimes(sol);
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                best_cmax = new_cmax;
                improved = true;
                break; // Structure changed, restart
            } else {
                sol.resupply_events = old_events;
                recalculateDroneTimes(sol);
            }
        }
        if (improved) break;
    }
    
    return improved;
}

bool IntegratedLocalSearch::droneSplitTrip(PDPSolution& sol) {
    // Split: T├ích trip c├│ nhiß╗üu customers th├ánh nhiß╗üu trips nhß╗Å h╞ín
    // Mß╗Ñc ti├¬u: Giß║úm waiting time khi customers c├│ ready_time kh├íc nhau nhiß╗üu
    
    if (sol.resupply_events.empty()) return false;
    
    double best_cmax = calculateCmax(sol);
    
    for (size_t trip_idx = 0; trip_idx < sol.resupply_events.size(); ++trip_idx) {
        auto& trip = sol.resupply_events[trip_idx];
        
        // Chß╗ë split trip c├│ >= 2 customers
        if (trip.customer_ids.size() < 2) continue;
        
        // T├¡nh ─æß╗Ö ch├¬nh lß╗çch ready_time trong trip
        int min_ready = INT_MAX, max_ready = 0;
        for (int cust : trip.customer_ids) {
            min_ready = min(min_ready, data.readyTimes[cust]);
            max_ready = max(max_ready, data.readyTimes[cust]);
        }
        
        // Chß╗ë split nß║┐u ready_time ch├¬nh lß╗çch > 15 ph├║t (─æ├íng ─æß╗â t├ích)
        if (max_ready - min_ready < 15) continue;
        
        // Thß╗¡ t├ích theo ready_time: nh├│m early v├á nh├│m late
        vector<int> early_group, late_group;
        int threshold = (min_ready + max_ready) / 2;
        
        for (int cust : trip.customer_ids) {
            if (data.readyTimes[cust] <= threshold) {
                early_group.push_back(cust);
            } else {
                late_group.push_back(cust);
            }
        }
        
        // Cß║ºn cß║ú 2 nh├│m ─æß╗üu c├│ customers
        if (early_group.empty() || late_group.empty()) continue;
        
        // Backup v├á thß╗¡ split
        vector<ResupplyEvent> old_events = sol.resupply_events;
        
        // Tß║ío 2 trips mß╗¢i
        ResupplyEvent early_trip = trip;
        early_trip.customer_ids = early_group;
        
        ResupplyEvent late_trip = trip;
        late_trip.customer_ids = late_group;
        // Thß╗¡ g├ín late_trip cho drone kh├íc nß║┐u c├│
        if (data.numDrones > 1) {
            late_trip.drone_id = (trip.drone_id + 1) % data.numDrones;
        }
        
        // Kiß╗âm tra feasibility
        if (!isDroneTripFeasible(early_trip, sol) || !isDroneTripFeasible(late_trip, sol)) {
            continue;
        }
        
        // Thay thß║┐ trip c┼⌐ bß║▒ng 2 trips mß╗¢i
        sol.resupply_events[trip_idx] = early_trip;
        sol.resupply_events.push_back(late_trip);
        
        recalculateDroneTimes(sol);
        double new_cmax = calculateCmax(sol);
        
        if (new_cmax < best_cmax - 0.01) {
            return true; // Improvement found
        } else {
            // Restore
            sol.resupply_events = old_events;
            recalculateDroneTimes(sol);
        }
    }
    
    return false;
}

// ============ NEW OPERATOR: Insert Into Existing Trip ============

vector<int> IntegratedLocalSearch::findStandaloneTypeD(const PDPSolution& sol) const {
    // T├¼m c├íc type D customers ─æang ─æ╞░ß╗úc truck vß╗ü depot lß║Ñy h├áng (kh├┤ng d├╣ng drone)
    // ─É├óy l├á candidates c├│ thß╗â consolidate v├áo drone trip c├│ sß║╡n
    
    set<int> drone_served;
    for (const auto& event : sol.resupply_events) {
        for (int cust : event.customer_ids) {
            drone_served.insert(cust);
        }
    }
    
    vector<int> standalone;
    for (const auto& truck : sol.truck_details) {
        for (size_t i = 1; i < truck.route.size(); ++i) {
            int node = truck.route[i];
            if (node == data.depotIndex) continue;
            
            // Type D customer kh├┤ng ─æ╞░ß╗úc drone phß╗Ñc vß╗Ñ
            if (isDroneEligible(node) && drone_served.find(node) == drone_served.end()) {
                standalone.push_back(node);
            }
        }
    }
    
    return standalone;
}

bool IntegratedLocalSearch::droneInsertIntoTrip(PDPSolution& sol) {
    // Insert Into Trip: Th├¬m standalone type D customer v├áo drone trip c├│ sß║╡n
    // ─Éiß╗üu n├áy t─âng consolidation v├á c├│ thß╗â giß║úm total completion time
    
    if (sol.resupply_events.empty()) return false;
    
    double best_cmax = calculateCmax(sol);
    int drone_capacity = data.getDroneCapacity();
    
    // T├¼m standalone type D customers
    vector<int> standalone = findStandaloneTypeD(sol);
    if (standalone.empty()) return false;
    
    for (int candidate : standalone) {
        int cand_ready = data.readyTimes[candidate];
        
        // Thß╗¡ insert v├áo tß╗½ng trip hiß╗çn c├│
        for (size_t trip_idx = 0; trip_idx < sol.resupply_events.size(); ++trip_idx) {
            auto& trip = sol.resupply_events[trip_idx];
            
            // Kiß╗âm tra capacity
            if ((int)trip.customer_ids.size() >= drone_capacity) continue;
            
            // Kiß╗âm tra consolidation constraints
            bool can_consolidate = true;
            for (int existing : trip.customer_ids) {
                // Ready time constraint (within 30 min)
                if (abs(data.readyTimes[existing] - cand_ready) > 30) {
                    can_consolidate = false;
                    break;
                }
                // Distance constraint (< 10km from first customer)
                double dist = data.droneDistMatrix[trip.customer_ids[0]][candidate];
                if (dist > 10.0) {
                    can_consolidate = false;
                    break;
                }
            }
            
            if (!can_consolidate) continue;
            
            // Backup v├á thß╗¡ insert
            vector<ResupplyEvent> old_events = sol.resupply_events;
            vector<TruckRouteInfo> old_trucks = sol.truck_details;
            
            // Th├¬m customer v├áo trip
            trip.customer_ids.push_back(candidate);
            
            // Sß║»p xß║┐p lß║íi theo ready_time ─æß╗â tß╗æi ╞░u
            sort(trip.customer_ids.begin(), trip.customer_ids.end(), 
                 [this](int a, int b) { return data.readyTimes[a] < data.readyTimes[b]; });
            
            // Kiß╗âm tra feasibility sau khi th├¬m
            if (!isDroneTripFeasible(trip, sol)) {
                sol.resupply_events = old_events;
                continue;
            }
            
            // X├│a candidate khß╗Åi truck route (truck kh├┤ng cß║ºn vß╗ü depot lß║Ñy h├áng n├áy nß╗»a)
            for (auto& truck : sol.truck_details) {
                auto it = find(truck.route.begin(), truck.route.end(), candidate);
                if (it != truck.route.end()) {
                    // Kiß╗âm tra xem c├│ depot trip tr╞░ß╗¢c customer n├áy kh├┤ng
                    size_t idx = it - truck.route.begin();
                    if (idx > 0 && truck.route[idx-1] == data.depotIndex) {
                        // X├│a cß║ú depot visit nß║┐u ─æ├óy l├á chuyß║┐n vß╗ü depot ri├¬ng
                        truck.route.erase(it - 1, it + 1);
                    } else {
                        truck.route.erase(it);
                    }
                    break;
                }
            }
            
            // Recalculate times
            recalculateTruckTimes(sol);
            recalculateDroneTimes(sol);
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                return true; // Improvement found
            } else {
                // Restore
                sol.resupply_events = old_events;
                sol.truck_details = old_trucks;
                recalculateTruckTimes(sol);
                recalculateDroneTimes(sol);
            }
        }
    }
    
    return false;
}

bool IntegratedLocalSearch::droneMoveCustomer(PDPSolution& sol) {
    // Simplified: just try moving first customer of each trip
    if (sol.resupply_events.size() < 2) return false;
    
    double best_cmax = calculateCmax(sol);
    int drone_capacity = data.getDroneCapacity();
    
    for (size_t i = 0; i < sol.resupply_events.size(); ++i) {
        if (sol.resupply_events[i].customer_ids.size() <= 1) continue;
        
        int customer = sol.resupply_events[i].customer_ids[0];
        
        for (size_t j = 0; j < sol.resupply_events.size(); ++j) {
            if (i == j) continue;
            if ((int)sol.resupply_events[j].customer_ids.size() >= drone_capacity) continue;
            
            vector<ResupplyEvent> old_events = sol.resupply_events;
            
            sol.resupply_events[i].customer_ids.erase(sol.resupply_events[i].customer_ids.begin());
            sol.resupply_events[j].customer_ids.push_back(customer);
            
            if (!isDroneTripFeasible(sol.resupply_events[j], sol)) {
                sol.resupply_events = old_events;
                continue;
            }
            
            recalculateDroneTimes(sol);
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                return true; // First improvement
            } else {
                sol.resupply_events = old_events;
                recalculateDroneTimes(sol);
            }
        }
    }
    
    return false;
}
bool IntegratedLocalSearch::droneSwapCustomers(PDPSolution& sol) {
    if (sol.resupply_events.size() < 2) return false;
    
    double best_cmax = calculateCmax(sol);
    
    // Simplified: only try first customer of each trip
    for (size_t i = 0; i < sol.resupply_events.size(); ++i) {
        if (sol.resupply_events[i].customer_ids.empty()) continue;
        
        for (size_t j = i + 1; j < sol.resupply_events.size(); ++j) {
            if (sol.resupply_events[j].customer_ids.empty()) continue;
            
            vector<ResupplyEvent> old_events = sol.resupply_events;
            
            swap(sol.resupply_events[i].customer_ids[0], 
                 sol.resupply_events[j].customer_ids[0]);
            
            if (!isDroneTripFeasible(sol.resupply_events[i], sol) || 
                !isDroneTripFeasible(sol.resupply_events[j], sol)) {
                sol.resupply_events = old_events;
                continue;
            }
            
            recalculateDroneTimes(sol);
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                return true; // First improvement
            } else {
                sol.resupply_events = old_events;
                recalculateDroneTimes(sol);
            }
        }
    }
    
    return false;
}

bool IntegratedLocalSearch::droneReassign(PDPSolution& sol) {
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    
    for (auto& trip : sol.resupply_events) {
        int original_drone = trip.drone_id;
        
        for (int d = 0; d < data.numDrones; ++d) {
            if (d == original_drone) continue;
            
            trip.drone_id = d;
            recalculateDroneTimes(sol);
            
            if (!isDroneTripFeasible(trip, sol)) {
                trip.drone_id = original_drone;
                recalculateDroneTimes(sol);
                continue;
            }
            
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                best_cmax = new_cmax;
                improved = true;
                original_drone = d;
            } else {
                trip.drone_id = original_drone;
                recalculateDroneTimes(sol);
            }
        }
    }
    
    return improved;
}

bool IntegratedLocalSearch::droneReorderTrip(PDPSolution& sol) {
    bool improved = false;
    double best_cmax = calculateCmax(sol);
    
    for (auto& trip : sol.resupply_events) {
        if (trip.customer_ids.size() < 2) continue;
        // Only reorder trips with <= 3 customers (avoid factorial explosion)
        if (trip.customer_ids.size() > 3) continue;
        
        vector<int> original_order = trip.customer_ids;
        vector<int> best_order = original_order;
        
        // Try all permutations for small trips
        sort(trip.customer_ids.begin(), trip.customer_ids.end());
        
        do {
            if (trip.customer_ids == original_order) continue;
            
            if (!isDroneTripFeasible(trip, sol)) continue;
            
            recalculateDroneTimes(sol);
            double new_cmax = calculateCmax(sol);
            
            if (new_cmax < best_cmax - 0.01) {
                best_cmax = new_cmax;
                best_order = trip.customer_ids;
                improved = true;
            }
        } while (next_permutation(trip.customer_ids.begin(), trip.customer_ids.end()));
        
        trip.customer_ids = best_order;
        recalculateDroneTimes(sol);
    }
    
    return improved;
}

// ============ CROSS TRUCK-DRONE OPERATORS ============
// ============ PERTURBATION ============

void perturbSolution(PDPSolution& sol, mt19937& rng, const PDPData& data) {
    uniform_int_distribution<int> choice(0, 2);
    int perturbType = choice(rng);
    
    if (perturbType == 0 && sol.truck_details.size() >= 1) {
        // Perturb truck: random swap in route
        for (auto& truck : sol.truck_details) {
            if (truck.route.size() >= 4) {
                uniform_int_distribution<int> dist(1, truck.route.size() - 2);
                int i = dist(rng);
                int j = dist(rng);
                if (i != j) {
                    swap(truck.route[i], truck.route[j]);
                }
                break;
            }
        }
    } else if (perturbType == 1 && sol.resupply_events.size() >= 2) {
        // Perturb drone: swap customers between trips
        uniform_int_distribution<int> dist(0, sol.resupply_events.size() - 1);
        int t1 = dist(rng);
        int t2 = dist(rng);
        if (t1 != t2 && !sol.resupply_events[t1].customer_ids.empty() && 
            !sol.resupply_events[t2].customer_ids.empty()) {
            uniform_int_distribution<int> d1(0, sol.resupply_events[t1].customer_ids.size() - 1);
            uniform_int_distribution<int> d2(0, sol.resupply_events[t2].customer_ids.size() - 1);
            swap(sol.resupply_events[t1].customer_ids[d1(rng)], 
                 sol.resupply_events[t2].customer_ids[d2(rng)]);
        }
    } else if (sol.resupply_events.size() >= 1) {
        // Perturb drone: change drone assignment
        uniform_int_distribution<int> dist(0, sol.resupply_events.size() - 1);
        int t = dist(rng);
        uniform_int_distribution<int> droneDist(0, data.numDrones - 1);
        sol.resupply_events[t].drone_id = droneDist(rng);
    }
}

// ============ MAIN PHASES (with Adaptive Selection) ============

bool IntegratedLocalSearch::optimizeTruckRoutes(PDPSolution& sol) {
    double current_cmax = calculateCmax(sol);
    PDPSolution best_sol = sol;
    bool any_improved = false;
    
    // Define truck operators
    vector<OperatorType> truck_ops = {
        OperatorType::TRUCK_2OPT,
        OperatorType::TRUCK_SWAP,
        OperatorType::TRUCK_RELOCATE,
        OperatorType::TRUCK_CROSS_EXCHANGE
    };
    
    // Adaptive: Select operator based on weights
    OperatorType selected_op = selectOperator(truck_ops);
    
    // Try selected operator first
    PDPSolution candidate = sol;
    bool op_success = false;
    double improvement = 0.0;
    
    switch (selected_op) {
        case OperatorType::TRUCK_2OPT:
            op_success = truck2Opt(candidate);
            break;
        case OperatorType::TRUCK_SWAP:
            op_success = truckSwap(candidate);
            break;
        case OperatorType::TRUCK_RELOCATE:
            op_success = truckRelocate(candidate);
            break;
        case OperatorType::TRUCK_CROSS_EXCHANGE:
            op_success = truckCrossExchange(candidate);
            break;
        default:
            break;
    }
    
    if (op_success) {
        double new_cmax = calculateCmax(candidate);
        if (new_cmax < current_cmax - 0.01) {
            improvement = current_cmax - new_cmax;
            best_sol = candidate;
            current_cmax = new_cmax;
            any_improved = true;
        }
    }
    updateOperatorStats(selected_op, any_improved, improvement);
    
    // Also try other operators (but with lower priority)
    for (OperatorType op : truck_ops) {
        if (op == selected_op) continue;
        
        candidate = sol;
        bool success = false;
        
        switch (op) {
            case OperatorType::TRUCK_2OPT:
                success = truck2Opt(candidate);
                break;
            case OperatorType::TRUCK_SWAP:
                success = truckSwap(candidate);
                break;
            case OperatorType::TRUCK_RELOCATE:
                success = truckRelocate(candidate);
                break;
            case OperatorType::TRUCK_CROSS_EXCHANGE:
                success = truckCrossExchange(candidate);
                break;
            default:
                break;
        }
        
        if (success) {
            double new_cmax = calculateCmax(candidate);
            if (new_cmax < current_cmax - 0.01) {
                improvement = current_cmax - new_cmax;
                best_sol = candidate;
                current_cmax = new_cmax;
                any_improved = true;
                updateOperatorStats(op, true, improvement);
            } else {
                updateOperatorStats(op, false, 0.0);
            }
        } else {
            updateOperatorStats(op, false, 0.0);
        }
    }
    
    if (any_improved) {
        sol = best_sol;
    }
    return any_improved;
}

bool IntegratedLocalSearch::optimizeDroneTrips(PDPSolution& sol) {
    double current_cmax = calculateCmax(sol);
    PDPSolution best_sol = sol;
    bool any_improved = false;
    
    // Define drone operators (including new ones)
    vector<OperatorType> drone_ops = {
        OperatorType::DRONE_REORDER,
        OperatorType::DRONE_SWAP,
        OperatorType::DRONE_MOVE,
        OperatorType::DRONE_MERGE,
        OperatorType::DRONE_SPLIT,
        OperatorType::DRONE_REASSIGN,
        OperatorType::DRONE_INSERT_INTO_TRIP
    };
    
    // Adaptive: Select operator based on weights
    OperatorType selected_op = selectOperator(drone_ops);
    
    // Try selected operator first
    PDPSolution candidate = sol;
    bool op_success = false;
    double improvement = 0.0;
    
    switch (selected_op) {
        case OperatorType::DRONE_REORDER:
            op_success = droneReorderTrip(candidate);
            break;
        case OperatorType::DRONE_SWAP:
            op_success = droneSwapCustomers(candidate);
            break;
        case OperatorType::DRONE_MOVE:
            op_success = droneMoveCustomer(candidate);
            break;
        case OperatorType::DRONE_MERGE:
            op_success = droneMergeTrips(candidate);
            break;
        case OperatorType::DRONE_SPLIT:
            op_success = droneSplitTrip(candidate);
            break;
        case OperatorType::DRONE_REASSIGN:
            op_success = droneReassign(candidate);
            break;
        case OperatorType::DRONE_INSERT_INTO_TRIP:
            op_success = droneInsertIntoTrip(candidate);
            break;
        default:
            break;
    }
    
    if (op_success) {
        double new_cmax = calculateCmax(candidate);
        if (new_cmax < current_cmax - 0.01) {
            improvement = current_cmax - new_cmax;
            best_sol = candidate;
            current_cmax = new_cmax;
            any_improved = true;
        }
    }
    updateOperatorStats(selected_op, any_improved, improvement);
    
    // Also try other operators
    for (OperatorType op : drone_ops) {
        if (op == selected_op) continue;
        
        candidate = sol;
        bool success = false;
        
        switch (op) {
            case OperatorType::DRONE_REORDER:
                success = droneReorderTrip(candidate);
                break;
            case OperatorType::DRONE_SWAP:
                success = droneSwapCustomers(candidate);
                break;
            case OperatorType::DRONE_MOVE:
                success = droneMoveCustomer(candidate);
                break;
            case OperatorType::DRONE_MERGE:
                success = droneMergeTrips(candidate);
                break;
            case OperatorType::DRONE_SPLIT:
                success = droneSplitTrip(candidate);
                break;
            case OperatorType::DRONE_REASSIGN:
                success = droneReassign(candidate);
                break;
            case OperatorType::DRONE_INSERT_INTO_TRIP:
                success = droneInsertIntoTrip(candidate);
                break;
            default:
                break;
        }
        
        if (success) {
            double new_cmax = calculateCmax(candidate);
            if (new_cmax < current_cmax - 0.01) {
                improvement = current_cmax - new_cmax;
                best_sol = candidate;
                current_cmax = new_cmax;
                any_improved = true;
                updateOperatorStats(op, true, improvement);
            } else {
                updateOperatorStats(op, false, 0.0);
            }
        } else {
            updateOperatorStats(op, false, 0.0);
        }
    }
    
    if (any_improved) {
        sol = best_sol;
    }
    return any_improved;
}

// ============ MAIN ENTRY POINT ============

PDPSolution IntegratedLocalSearch::run(PDPSolution initialSolution) {
    cout << "\n[ADAPTIVE LS] Starting Adaptive Local Search with 11 operators..." << endl;
    cout << "[ADAPTIVE LS] Initial C_max: " << fixed << setprecision(2) 
         << initialSolution.totalCost << " minutes" << endl;
    cout << "[ADAPTIVE LS] Truck routes: " << initialSolution.truck_details.size() << endl;
    cout << "[ADAPTIVE LS] Drone trips: " << initialSolution.resupply_events.size() << endl;
    
    PDPSolution current = initialSolution;
    PDPSolution best = initialSolution;
    double best_cmax = initialSolution.totalCost;
    double initial_cmax = best_cmax;
    
    int iter = 0;
    int no_improve_count = 0;
    int perturbations = 0;
    const int max_no_improve = 25;  // Increased for more exploration
    const int max_perturbations = 8;  // Increased perturbation budget
    const int weight_update_freq = 10;  // Update weights every 10 iterations
    
    while (iter < maxIterations) {
        bool improved = false;
        
        // Update operator weights periodically
        if (iter > 0 && iter % weight_update_freq == 0) {
            updateOperatorWeights();
        }
        
        // Phase 1: Truck optimization
        PDPSolution candidate = current;
        if (optimizeTruckRoutes(candidate)) {
            double new_cmax = calculateCmax(candidate);
            if (new_cmax < best_cmax - 0.01) {
                current = candidate;
                best = candidate;
                best_cmax = new_cmax;
                best.totalCost = best_cmax;
                truck_improvements++;
                improved = true;
                cout << "[ADAPTIVE LS] Iter " << iter << ": Truck improved to " 
                     << fixed << setprecision(2) << best_cmax << " min" << endl;
            }
        }
        
        // Phase 2: Drone optimization (with new operators)
        candidate = current;
        if (optimizeDroneTrips(candidate)) {
            double new_cmax = calculateCmax(candidate);
            if (new_cmax < best_cmax - 0.01) {
                current = candidate;
                best = candidate;
                best_cmax = new_cmax;
                best.totalCost = best_cmax;
                drone_improvements++;
                improved = true;
                cout << "[ADAPTIVE LS] Iter " << iter << ": Drone improved to " 
                     << fixed << setprecision(2) << best_cmax << " min" << endl;
            }
        }
        
        if (!improved) {
            no_improve_count++;
            
            // Perturbation when stuck
            if (no_improve_count >= max_no_improve && perturbations < max_perturbations) {
                cout << "[ADAPTIVE LS] Perturbation #" << (perturbations + 1) 
                     << " (diversification)" << endl;
                
                PDPSolution perturbed = best;
                
                // Stronger perturbation as we go
                int perturb_strength = 3 + perturbations;  // Increasing strength
                for (int p = 0; p < perturb_strength; ++p) {
                    perturbSolution(perturbed, rng, data);
                }
                recalculateTruckTimes(perturbed);
                recalculateDroneTimes(perturbed);
                
                current = perturbed;
                perturbations++;
                no_improve_count = 0;
            }
        } else {
            no_improve_count = 0;
        }
        
        // Stop if exhausted
        if (no_improve_count >= max_no_improve && perturbations >= max_perturbations) {
            break;
        }
        
        iter++;
    }
    
    // Print final statistics
    cout << "\n[ADAPTIVE LS] ====== FINAL RESULTS ======" << endl;
    cout << "[ADAPTIVE LS] Completed after " << iter << " iterations" << endl;
    cout << "[ADAPTIVE LS] Final C_max: " << fixed << setprecision(2) 
         << best.totalCost << " minutes" << endl;
    cout << "[ADAPTIVE LS] Improvements: Truck=" << truck_improvements 
         << ", Drone=" << drone_improvements << endl;
    cout << "[ADAPTIVE LS] Perturbations used: " << perturbations << endl;
    
    double improvement = initial_cmax - best.totalCost;
    if (improvement > 0.01) {
        cout << "[ADAPTIVE LS] Total improvement: " << fixed << setprecision(2) 
             << improvement << " minutes (" 
             << setprecision(1) << (improvement / initial_cmax * 100) << "%)" << endl;
    } else {
        cout << "[ADAPTIVE LS] No improvement found (solution may be near optimal)" << endl;
    }
    
    // Print operator statistics
    printOperatorStats();
    
    return best;
}
