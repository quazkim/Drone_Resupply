#include "pdp_fitness.h"
#include "pdp_types.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <set>
#include <map>

using namespace std;

// ============ CONFIGURATION ============
static const bool ENABLE_MERGE_STRATEGIES = true;   // Set to false to disable merge
static const bool ENABLE_CONSOLIDATION = true;      // Allow multiple packages per trip
static const bool ENABLE_ASSIGNMENT_LS = false;      // Disabled: first-improvement LS inside fitness
static const int ASSIGNMENT_LS_MAX_ITER = 50;        // Max iterations for assignment LS

// ============ FORWARD DECLARATIONS ============

// Trang thai cua moi xe tai (dung chung cho nhieu functions)
struct TruckState {
    double available_time;      // Thoi diem xe ranh
    int current_position;       // Vi tri hien tai cua xe
    double current_load;        // Load hien tai
    vector<int> route;          // Route cua xe nay
    vector<double> arrival_times;   // Thoi gian den moi node
    vector<double> departure_times; // Thoi gian roi moi node
    set<int> picked_up_pairs;   // Cac cap P-DL da pickup (luu pairId)
    set<int> cargo_on_truck;    // Cac goi hang dang co tren xe (customer IDs)
};

// ============ UTILITY FUNCTIONS ============

// Ham tien ich cho truck (Manhattan)
static double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.truckDistMatrix[nodeA_id][nodeB_id];
}

// Ham tien ich cho drone (Euclidean)
static double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.droneDistMatrix[nodeA_id][nodeB_id];
}

// ============ DRONE CONSOLIDATION HELPER ============

// Forward declaration
static pair<double, bool> evaluateDroneTrip(
    const vector<int>& customers,
    int drone_id,
    const struct TruckState& truck,
    const PDPData& data,
    const vector<double>& Drone_Available_Time
);

/**
 * @brief Tim cac customers type D co the consolidate trong 1 drone trip
 * @param seq Sequence cua customers
 * @param start_idx Index bat dau tim kiem
 * @param data Du lieu bai toan
 * @param truck_current_pos Vi tri hien tai cua truck
 * @param max_consolidate So luong toi da customers trong 1 trip
 * @return Vector cac customer IDs co the consolidate
 */
static vector<int> findConsolidationCandidates(
    const vector<int>& seq,
    int start_idx,
    const PDPData& data,
    int truck_current_pos,
    int max_consolidate
) {
    vector<int> candidates;
    int current_customer = seq[start_idx];
    
    // Customer hien tai luon duoc them vao
    if (data.nodeTypes[current_customer] == "D" && data.readyTimes[current_customer] > 0) {
        candidates.push_back(current_customer);
    } else {
        return candidates; // Chi consolidate cho type D
    }
    
    // Tim them cac customers type D tiep theo trong sequence
    // Chi consolidate neu:
    // 1. Cung type D
    // 2. Ready time khong qua xa nhau (trong vong 30 phut)
    // 3. Khoang cach dia ly gan nhau (< 10km tu current customer)
    int current_ready = data.readyTimes[current_customer];
    
    for (int i = start_idx + 1; i < seq.size() && candidates.size() < max_consolidate; ++i) {
        int next_customer = seq[i];
        if (!data.isCustomer(next_customer)) continue;
        
        // Chß╗ë consolidate type D (C1 customers)
        if (data.nodeTypes[next_customer] == "D" && data.readyTimes[next_customer] > 0) {
            int next_ready = data.readyTimes[next_customer];
            double distance = getDroneDistance(data, current_customer, next_customer);
            
            // Kiem tra dieu kien consolidation
            // Ready time khong qua xa: trong vong 30 phut
            // Khoang cach khong qua xa: < 10km
            if (abs(next_ready - current_ready) <= 30 && distance <= 10.0) {
                candidates.push_back(next_customer);
            }
        }
    }
    
    return candidates;
}

/**
 * @brief Tinh toan chi phi va kiem tra feasibility cua drone trip
 * LOGIC CHINH XAC:
 *   1. Drone xuat phat tu DEPOT (mang nhieu goi hang - packages cho customers)
 *   2. Drone bay THANG den 1 DIEM GAP TRUCK (resupply point - la 1 customer nao do)
 *   3. Tai diem do, drone GIAO TAT CA packages cho truck
 *   4. Drone ve depot
 *   5. Truck se di giao hang cho cac customers
 * 
 * @param customers Danh sach customer IDs ma packages cua ho duoc chuyen cho truck (KHONG phai drone visit)
 * @param drone_id ID cua drone
 * @param truck Trang thai hien tai cua truck
 * @param data Du lieu bai toan
 * @param Drone_Available_Time Vector thoi gian ranh cua cac drones
 * @return pair<completion_time, feasible>
 */
static pair<double, bool> evaluateDroneTrip(
    const vector<int>& customers,
    int drone_id,
    const struct TruckState& truck,
    const PDPData& data,
    const vector<double>& Drone_Available_Time
) {
    if (customers.empty()) return {numeric_limits<double>::max(), false};
    
    // CRITICAL: TAT CA packages phai READY TAI DEPOT truoc khi drone xuat phat
    // Ready time cua moi customer la thoi diem package cua ho SAN SANG TAI DEPOT
    // Drone phai DOI package cuoi cung ready, roi moi load va xuat phat
    double max_ready_time = 0.0;
    for (int cust : customers) {
        max_ready_time = max(max_ready_time, (double)data.readyTimes[cust]);
    }
    
    // Drone chi xuat phat khi:
    // 1. Drone da ranh (available)
    // 2. TAT CA packages da ready tai depot (max_ready_time)
    double T_Drone_Ready = max(Drone_Available_Time[drone_id], max_ready_time);
    
    // Load packages tai depot
    double drone_depart_time = T_Drone_Ready + data.depotDroneLoadTime;
    
    // Chon diem resupply = customer DAU TIEN trong list
    // (truck se di den day de nhan packages, roi di giao cho tat ca customers)
    int resupply_point = customers[0];
    
    // VALIDATION: Kiem tra resupply point co hop le khong
    // Ready time tai resupply point phai <= thoi diem drone den
    // (vi packages da ready tai depot roi, nen ready time tai resupply point khong anh huong)
    // NHUNG ta van can dam bao truck co the den duoc
    
    // PHASE 1: Drone bay THANG tu depot den resupply point
    double t_fly_to_resupply = getDroneDistance(data, data.depotIndex, resupply_point) / data.droneSpeed * 60.0;
    double drone_arrive_time = drone_depart_time + t_fly_to_resupply;
    
    // PHASE 2: Truck di den resupply point
    double truck_travel_time = getTruckDistance(data, truck.current_position, resupply_point) / data.truckSpeed * 60.0;
    double truck_arrive_time = truck.available_time + truck_travel_time;
    
    // Thoi gian bat dau resupply = max(drone arrive, truck arrive)
    // KHONG can kiem tra ready time tai resupply point
    // Vi ready time chi la thoi diem package XUAT HIEN TAI DEPOT
    // Sau khi package ready, co the giao cho khach BAT CU LUC NAO
    double T_Start_Resupply = max(drone_arrive_time, truck_arrive_time);
    
    // Thoi gian cho cua drone (neu truck chua den)
    double wait_time = T_Start_Resupply - drone_arrive_time;
    
    // VALIDATION: Neu drone phai doi qua lau -> khong hieu qua
    // (co the bo qua neu muon linh hoat hon)
    // if (wait_time > 30.0) return {numeric_limits<double>::max(), false};
    
    // Resupply: Drone giao TAT CA packages cho truck
    double resupply_end = T_Start_Resupply + data.resupplyTime;
    
    // PHASE 3: Drone quay ve depot
    double t_return = getDroneDistance(data, resupply_point, data.depotIndex) / data.droneSpeed * 60.0;
    double drone_return_time = resupply_end + t_return;
    
    // Tinh tong thoi gian bay
    double total_flight_time = t_fly_to_resupply + wait_time + t_return;
    
    // Kiem tra endurance
    bool feasible = (total_flight_time <= data.droneEndurance);
    
    // PHASE 4: Truck di giao hang cho TAT CA customers
    // Sau khi nhan hang tu drone, truck phai di giao cho tung customer
    double truck_time = resupply_end;
    int truck_pos = resupply_point;
    
    for (int cust : customers) {
        // Truck di den customer
        double travel_time = getTruckDistance(data, truck_pos, cust) / data.truckSpeed * 60.0;
        double arrival_time = truck_time + travel_time;
        
        // Giao hang (service time)
        truck_time = arrival_time + data.truckServiceTime;
        truck_pos = cust;
    }
    
    // Completion time = truck finish delivery
    // C_max is defined as: max(time last vehicle returns to depot, time last customer is delivered)
    // Drone return time does not determine C_max; only truck delivery times matter here.
    double completion_time = truck_time;
    
    return {completion_time, feasible};
}

/**
 * @brief Merge 2 drone trips va kiem tra xem co tot hon khong
 * @return pair<merged_event, improvement> - improvement > 0 neu tot hon
 */
static pair<ResupplyEvent, double> tryMergeTrips(
    const ResupplyEvent& trip1,
    const ResupplyEvent& trip2,
    const PDPData& data,
    const vector<TruckState>& trucks,
    const vector<double>& Drone_Available_Time
) {
    ResupplyEvent merged;
    merged.customer_ids = trip1.customer_ids;
    merged.customer_ids.insert(merged.customer_ids.end(),
                               trip2.customer_ids.begin(),
                               trip2.customer_ids.end());
    merged.drone_id = trip1.drone_id;
    merged.truck_id = trip1.truck_id;
    
    // Sort by ready time
    sort(merged.customer_ids.begin(), merged.customer_ids.end(),
         [&data](int a, int b) { return data.readyTimes[a] < data.readyTimes[b]; });
    
    // Check capacity
    if ((int)merged.customer_ids.size() > data.getDroneCapacity()) {
        return {merged, -1e9}; // Not feasible
    }
    
    // Check consolidation constraints
    if (merged.customer_ids.size() > 1) {
        int first_ready = data.readyTimes[merged.customer_ids[0]];
        int max_ready = first_ready;
        int min_ready = first_ready;
        
        for (size_t i = 1; i < merged.customer_ids.size(); ++i) {
            int cust = merged.customer_ids[i];
            int ready = data.readyTimes[cust];
            
            max_ready = max(max_ready, ready);
            min_ready = min(min_ready, ready);
            
            // Ready time constraint (30 min - relaxed for feasibility)
            if (abs(ready - first_ready) > 30) {
                return {merged, -1e9};
            }
            
            // Distance constraint (10km from first)
            double dist = getDroneDistance(data, merged.customer_ids[0], cust);
            if (dist > 10.0) {
                return {merged, -1e9};
            }
        }
        
        // SMART CHECK: Nếu ready time chênh lệch > 15 phút
        // → Merge có thể GÂY WAITING TIME cao
        // Chỉ merge nếu có lợi ích rõ ràng về khoảng cách
        int ready_gap = max_ready - min_ready;
        if (ready_gap > 15) {
            // Tính avg distance giữa customers
            double total_dist = 0;
            int pair_count = 0;
            for (size_t i = 0; i < merged.customer_ids.size(); ++i) {
                for (size_t j = i + 1; j < merged.customer_ids.size(); ++j) {
                    total_dist += getDroneDistance(data, 
                                                   merged.customer_ids[i], 
                                                   merged.customer_ids[j]);
                    pair_count++;
                }
            }
            double avg_dist = total_dist / pair_count;
            
            // Nếu ready gap lớn VÀ customers xa nhau → KHÔNG merge
            if (avg_dist > 3.0) {  // 3km threshold
                return {merged, -1e9};
            }
        }
    }
    
    // Evaluate merged trip
    auto result = evaluateDroneTrip(
        merged.customer_ids,
        merged.drone_id,
        trucks[merged.truck_id],
        data,
        Drone_Available_Time
    );
    
    if (!result.second) { // Not feasible
        return {merged, -1e9};
    }
    
    double merged_completion = result.first;
    
    // Calculate improvement: compare with doing 2 separate trips
    // CRITICAL: Phải xét xem 2 trips có cùng drone hay không
    
    double trip1_completion = evaluateDroneTrip(
        trip1.customer_ids, trip1.drone_id, 
        trucks[trip1.truck_id], data, Drone_Available_Time).first;
    double trip2_completion = evaluateDroneTrip(
        trip2.customer_ids, trip2.drone_id,
        trucks[trip2.truck_id], data, Drone_Available_Time).first;
    
    double original_completion;
    
    if (trip1.drone_id == trip2.drone_id) {
        // CÙNG DRONE: Phải chạy tuần tự (sequential)
        // Trip 2 chỉ bắt đầu sau khi trip 1 hoàn thành
        
        // Giả sử trip1 chạy trước (có ready time sớm hơn)
        int trip1_max_ready = 0;
        for (int c : trip1.customer_ids) 
            trip1_max_ready = max(trip1_max_ready, data.readyTimes[c]);
        
        int trip2_max_ready = 0;
        for (int c : trip2.customer_ids)
            trip2_max_ready = max(trip2_max_ready, data.readyTimes[c]);
        
        if (trip1_max_ready <= trip2_max_ready) {
            // Trip 1 trước, Trip 2 sau
            // Trip 2 phải đợi trip 1 về depot + ready time của trip 2
            double trip2_start = max(trip1_completion, (double)trip2_max_ready);
            double trip2_adjusted = trip2_start - trip2_max_ready + trip2_completion;
            original_completion = trip2_adjusted;
        } else {
            // Trip 2 trước, Trip 1 sau
            double trip1_start = max(trip2_completion, (double)trip1_max_ready);
            double trip1_adjusted = trip1_start - trip1_max_ready + trip1_completion;
            original_completion = trip1_adjusted;
        }
    } else {
        // KHÁC DRONE: Chạy song song (parallel)
        original_completion = max(trip1_completion, trip2_completion);
    }
    
    // Calculate waiting time penalty for merged trip
    double merged_waiting_penalty = 0;
    if (merged.customer_ids.size() > 1) {
        int max_ready = 0;
        for (int cust : merged.customer_ids) {
            max_ready = max(max_ready, data.readyTimes[cust]);
        }
        
        // Các customers có ready time sớm hơn phải đợi
        for (int cust : merged.customer_ids) {
            int wait_time = max_ready - data.readyTimes[cust];
            merged_waiting_penalty += wait_time * 0.5; // 0.5 weight cho waiting
        }
    }
    
    // Adjusted improvement (trừ đi waiting penalty)
    double improvement = original_completion - merged_completion - merged_waiting_penalty;
    
    // CHỈ merge nếu improvement > threshold (ít nhất 1 phút)
    if (improvement < 1.0) {
        return {merged, -1.0}; // Not worth merging
    }
    
    // Update merged event with full details
    double max_ready = 0.0;
    for (int cust : merged.customer_ids) {
        max_ready = max(max_ready, (double)data.readyTimes[cust]);
    }
    
    double T_Drone_Ready = max(Drone_Available_Time[merged.drone_id], max_ready);
    merged.drone_depart_time = T_Drone_Ready + data.depotDroneLoadTime;
    
    const auto& truck = trucks[merged.truck_id];
    
    // LOGIC DUNG: Chi co 1 resupply point (customer dau tien)
    int resupply_point = merged.customer_ids[0];
    
    // Drone bay tu depot -> resupply point
    double t_fly_to_resupply = getDroneDistance(data, data.depotIndex, resupply_point) / data.droneSpeed * 60.0;
    double drone_arrive = merged.drone_depart_time + t_fly_to_resupply;
    
    // Truck den resupply point
    double truck_travel = getTruckDistance(data, truck.current_position, resupply_point) / data.truckSpeed * 60.0;
    double truck_arrive = truck.available_time + truck_travel;
    
    // Resupply
    double resupply_start = max(drone_arrive, truck_arrive);
    double wait = resupply_start - drone_arrive;
    double resupply_end = resupply_start + data.resupplyTime;
    
    // Drone ve depot
    double t_return = getDroneDistance(data, resupply_point, data.depotIndex) / data.droneSpeed * 60.0;
    merged.drone_return_time = resupply_end + t_return;
    merged.total_flight_time = t_fly_to_resupply + wait + t_return;
    
    // ONE RENDEZVOUS: Chi luu 1 resupply point
    merged.resupply_point = resupply_point;
    merged.drone_arrive_time = drone_arrive;
    merged.truck_arrive_time = truck_arrive;
    merged.resupply_start_time = resupply_start;
    merged.resupply_end_time = resupply_end;
    
    // Truck tu di giao hang cho cac customers
    double truck_delivery_time = resupply_end;
    int current_pos = resupply_point;
    for (int cust : merged.customer_ids) {
        double travel = getTruckDistance(data, current_pos, cust) / data.truckSpeed * 60.0;
        truck_delivery_time += travel + data.truckServiceTime;
        current_pos = cust;
    }
    merged.truck_delivery_end = truck_delivery_time;
    
    return {merged, improvement};
}

// =========================================================
// === ASSIGNMENT ENCODING + LOCAL SEARCH ===
// =========================================================

// AssignmentEncoding struct is defined in pdp_fitness.h

struct DroneTripGroup {
    int truck_id;
    int drone_id;  // 0-based
    vector<int> customer_ids;
};

// Build drone trips from encoding
static vector<DroneTripGroup> buildDroneTrips(
    const vector<int>& seq,
    const AssignmentEncoding& enc,
    const PDPData& data
) {
    vector<DroneTripGroup> trips;
    map<pair<int,int>, int> active_trip;  // (truck_id, drone_id) -> trip index

    for (int i = 0; i < (int)seq.size(); i++) {
        int c = seq[i];
        if (!data.isCustomer(c)) continue;
        if (data.nodeTypes[c] != "D" || data.readyTimes[c] <= 0) continue;
        if (enc.drone_assign[i] == 0) continue;

        int truck_id = enc.truck_assign[i];
        int drone_id = enc.drone_assign[i] - 1;
        auto key = make_pair(truck_id, drone_id);

        bool start_new = (enc.break_bit[i] == 1) || !active_trip.count(key);

        if (!start_new) {
            int tidx = active_trip[key];
            if ((int)trips[tidx].customer_ids.size() >= data.getDroneCapacity()) {
                start_new = true;  // capacity exceeded
            }
        }

        if (start_new) {
            DroneTripGroup trip;
            trip.truck_id = truck_id;
            trip.drone_id = drone_id;
            trip.customer_ids.push_back(c);
            trips.push_back(trip);
            active_trip[key] = (int)trips.size() - 1;
        } else {
            trips[active_trip[key]].customer_ids.push_back(c);
        }
    }
    return trips;
}

// Decode solution from explicit encoding (truck_assign + drone_assign + break_bit)
static PDPSolution decodeFromEncoding(
    const vector<int>& seq,
    const AssignmentEncoding& enc,
    const PDPData& data
) {
    PDPSolution sol;
    sol.totalCost = 0.0;
    sol.totalPenalty = 0.0;
    sol.isFeasible = true;
    sol.original_sequence = seq;

    if (seq.empty()) {
        sol.totalPenalty = 1e9;
        sol.isFeasible = false;
        return sol;
    }

    // Pre-build drone trips
    auto drone_trips = buildDroneTrips(seq, enc, data);

    // Map customers to trips
    map<int, int> customer_to_trip;
    set<int> trip_first_custs;
    map<int, int> first_cust_to_trip;

    for (int t = 0; t < (int)drone_trips.size(); t++) {
        for (int c : drone_trips[t].customer_ids)
            customer_to_trip[c] = t;
        if (!drone_trips[t].customer_ids.empty()) {
            int first = drone_trips[t].customer_ids[0];
            trip_first_custs.insert(first);
            first_cust_to_trip[first] = t;
        }
    }

    // Init trucks
    double C_max = 0.0;
    vector<TruckState> trucks(data.numTrucks);
    for (int i = 0; i < data.numTrucks; i++) {
        trucks[i].available_time = 0.0;
        trucks[i].current_position = data.depotIndex;
        trucks[i].current_load = 0.0;
        trucks[i].route.push_back(data.depotIndex);
        trucks[i].arrival_times.push_back(0.0);
        trucks[i].departure_times.push_back(0.0);
    }

    vector<double> Drone_Available_Time(data.numDrones, 0.0);
    sol.drone_completion_times.resize(data.numDrones, 0.0);

    set<int> processed_customers;

    for (int seq_idx = 0; seq_idx < (int)seq.size(); seq_idx++) {
        int v_id = seq[seq_idx];
        if (!data.isCustomer(v_id)) continue;
        if (processed_customers.count(v_id)) continue;

        string v_type = data.nodeTypes[v_id];
        int v_ready = data.readyTimes[v_id];
        int v_demand = data.demands[v_id];
        int v_pairId = data.pairIds[v_id];
        double e_v = (double)v_ready;

        // ===== DL: FORCED to pickup truck =====
        if (v_type == "DL" && v_pairId > 0) {
            int pickup_truck_id = -1;
            for (int t = 0; t < data.numTrucks; t++) {
                if (trucks[t].picked_up_pairs.count(v_pairId)) {
                    pickup_truck_id = t;
                    break;
                }
            }
            if (pickup_truck_id == -1) {
                sol.totalPenalty += 10000;
                sol.isFeasible = false;
                continue;
            }

            TruckState& truck = trucks[pickup_truck_id];
            double T_Arr = truck.available_time +
                getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
            truck.available_time = T_Arr + data.truckServiceTime;
            truck.current_position = v_id;
            truck.route.push_back(v_id);
            truck.arrival_times.push_back(T_Arr);
            truck.departure_times.push_back(truck.available_time);
            truck.current_load += v_demand;

            if (truck.current_load < -0.01) { sol.totalPenalty += 1000; sol.isFeasible = false; }
            if (truck.current_load > data.truckCapacity) { sol.totalPenalty += 1000; sol.isFeasible = false; }

            truck.picked_up_pairs.erase(v_pairId);
            C_max = max(C_max, truck.available_time);
            continue;
        }

        // ===== Get truck from encoding =====
        int truck_id = enc.truck_assign[seq_idx];
        if (truck_id < 0 || truck_id >= data.numTrucks) truck_id = 0;
        TruckState& truck = trucks[truck_id];

        // ===== Type D =====
        if (v_type == "D" && v_ready > 0) {
            // Cargo already on truck (from previous depot return)
            if (truck.cargo_on_truck.count(v_id) > 0) {
                double T_Arr = truck.available_time +
                    getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
                truck.available_time = T_Arr + data.truckServiceTime;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Arr);
                truck.departure_times.push_back(truck.available_time);
                truck.current_load -= v_demand;
                truck.cargo_on_truck.erase(v_id);
                C_max = max(C_max, truck.available_time);
                continue;
            }

            int drone_val = enc.drone_assign[seq_idx];

            if (drone_val > 0 && trip_first_custs.count(v_id)) {
                // DRONE RESUPPLY: This is the first customer of a trip
                int tidx = first_cust_to_trip[v_id];
                auto& trip = drone_trips[tidx];
                int drone_id = trip.drone_id;

                auto result = evaluateDroneTrip(
                    trip.customer_ids, drone_id, truck, data, Drone_Available_Time
                );

                if (result.second) {
                    // === FEASIBLE: schedule drone resupply ===
                    ResupplyEvent event;
                    event.customer_ids = trip.customer_ids;
                    event.drone_id = drone_id;
                    event.truck_id = truck_id;

                    double max_ready = 0.0;
                    for (int cust : trip.customer_ids)
                        max_ready = max(max_ready, (double)data.readyTimes[cust]);

                    double T_Drone_Ready = max(Drone_Available_Time[drone_id], max_ready);
                    event.drone_depart_time = T_Drone_Ready + data.depotDroneLoadTime;

                    int resupply_point = trip.customer_ids[0];
                    event.resupply_point = resupply_point;

                    double t_fly = getDroneDistance(data, data.depotIndex, resupply_point)
                                   / data.droneSpeed * 60.0;
                    event.drone_arrive_time = event.drone_depart_time + t_fly;

                    double truck_travel = getTruckDistance(data, truck.current_position, resupply_point)
                                          / data.truckSpeed * 60.0;
                    event.truck_arrive_time = truck.available_time + truck_travel;

                    double resupply_start = max(event.drone_arrive_time, event.truck_arrive_time);
                    double wait = resupply_start - event.drone_arrive_time;
                    event.resupply_start_time = resupply_start;
                    event.resupply_end_time = resupply_start + data.resupplyTime;

                    double t_return = getDroneDistance(data, resupply_point, data.depotIndex)
                                      / data.droneSpeed * 60.0;
                    event.drone_return_time = event.resupply_end_time + t_return;
                    event.total_flight_time = t_fly + wait + t_return;

                    // Truck route: go to resupply point, deliver first customer
                    double departure_from_resupply = event.resupply_end_time + data.truckServiceTime;
                    truck.route.push_back(resupply_point);
                    truck.arrival_times.push_back(event.truck_arrive_time);
                    truck.departure_times.push_back(departure_from_resupply);
                    truck.current_position = resupply_point;
                    truck.available_time = departure_from_resupply;

                    // Deliver remaining customers in trip
                    for (int cust_id : trip.customer_ids) {
                        if (cust_id == resupply_point) continue;
                        double travel = getTruckDistance(data, truck.current_position, cust_id)
                                        / data.truckSpeed * 60.0;
                        double arrival = truck.available_time + travel;
                        double departure = arrival + data.truckServiceTime;
                        truck.route.push_back(cust_id);
                        truck.arrival_times.push_back(arrival);
                        truck.departure_times.push_back(departure);
                        truck.current_position = cust_id;
                        truck.available_time = departure;
                    }

                    event.truck_delivery_end = truck.available_time;
                    sol.resupply_events.push_back(event);

                    Drone_Available_Time[drone_id] = event.drone_return_time;
                    sol.drone_completion_times[drone_id] =
                        max(sol.drone_completion_times[drone_id], event.drone_return_time);
                    C_max = max(C_max, event.drone_return_time);
                    C_max = max(C_max, truck.available_time);

                    for (int c : trip.customer_ids)
                        processed_customers.insert(c);
                    continue;
                } else {
                    // Infeasible drone trip -> penalty, fallback to depot return
                    sol.totalPenalty += 500;
                }
            } else if (drone_val > 0 && customer_to_trip.count(v_id)) {
                // Not first customer of trip -> already processed or will be
                continue;
            }

            // DEPOT RETURN (drone_val=0 or drone infeasible)
            if (truck.current_position != data.depotIndex) {
                double t_to_depot = getTruckDistance(data, truck.current_position, data.depotIndex)
                                    / data.truckSpeed * 60.0;
                double T_Arr_Depot = truck.available_time + t_to_depot;
                truck.route.push_back(data.depotIndex);
                truck.arrival_times.push_back(T_Arr_Depot);
                truck.departure_times.push_back(T_Arr_Depot + data.depotReceiveTime);
                truck.available_time = T_Arr_Depot + data.depotReceiveTime;
                truck.current_position = data.depotIndex;
                truck.current_load = 0.0;
                truck.cargo_on_truck.clear();
            }

            double T_Depart = max(truck.available_time, (double)v_ready);
            if (!truck.route.empty() && truck.route.back() == data.depotIndex)
                truck.departure_times.back() = T_Depart;
            truck.available_time = T_Depart;

            truck.current_load += v_demand;
            truck.cargo_on_truck.insert(v_id);

            double t_to_cust = getTruckDistance(data, data.depotIndex, v_id)
                               / data.truckSpeed * 60.0;
            double T_Arr = truck.available_time + t_to_cust;
            double T_Start = max(T_Arr, e_v);
            truck.available_time = T_Start + data.truckServiceTime;
            truck.current_position = v_id;
            truck.route.push_back(v_id);
            truck.arrival_times.push_back(T_Arr);
            truck.departure_times.push_back(truck.available_time);
            truck.current_load -= v_demand;
            truck.cargo_on_truck.erase(v_id);

            if (truck.current_load > data.truckCapacity) { sol.totalPenalty += 1000; sol.isFeasible = false; }
            if (truck.current_load < -0.01) { sol.totalPenalty += 1000; sol.isFeasible = false; }
        }
        // ===== Type P =====
        else if (v_type == "P") {
            if (truck.current_position == data.depotIndex &&
                !truck.route.empty() && truck.route.back() == data.depotIndex &&
                e_v > truck.available_time) {
                truck.available_time = e_v;
                truck.departure_times.back() = e_v;
            }

            double T_Arr = truck.available_time +
                getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
            double T_Start = max(T_Arr, e_v);
            truck.available_time = T_Start + data.truckServiceTime;
            truck.current_position = v_id;
            truck.route.push_back(v_id);
            truck.arrival_times.push_back(T_Arr);
            truck.departure_times.push_back(truck.available_time);

            truck.current_load += v_demand;
            truck.cargo_on_truck.insert(v_id);
            if (v_pairId > 0) truck.picked_up_pairs.insert(v_pairId);

            if (truck.current_load > data.truckCapacity) { sol.totalPenalty += 1000; sol.isFeasible = false; }
            if (truck.current_load < 0) { sol.totalPenalty += 1000; sol.isFeasible = false; }
        }

        C_max = max(C_max, truck.available_time);
    }

    // ===== PROPAGATE DELAY =====
    for (auto& event : sol.resupply_events) {
        if (event.customer_ids.empty()) continue;
        int tid = event.truck_id;
        if (tid < 0 || tid >= (int)trucks.size()) continue;
        int rp = event.resupply_point;
        if (rp <= 0) rp = event.customer_ids[0];
        auto& tr = trucks[tid];
        for (size_t k = 0; k < tr.route.size(); k++) {
            if (tr.route[k] == rp) {
                double required = event.resupply_end_time;
                if (required > tr.departure_times[k]) {
                    double delay = required - tr.departure_times[k];
                    tr.departure_times[k] = required;
                    for (size_t m = k + 1; m < tr.route.size(); m++) {
                        tr.arrival_times[m] += delay;
                        tr.departure_times[m] += delay;
                    }
                    tr.available_time = tr.departure_times.back();
                }
                break;
            }
        }
    }

    // Update C_max
    for (const auto& event : sol.resupply_events) {
        C_max = max(C_max, event.drone_return_time);
        C_max = max(C_max, event.truck_delivery_end);
    }
    for (int i = 0; i < data.numTrucks; i++)
        C_max = max(C_max, trucks[i].available_time);
    for (int d = 0; d < data.numDrones; d++)
        C_max = max(C_max, sol.drone_completion_times[d]);

    // Return to depot
    for (int i = 0; i < data.numTrucks; i++) {
        if (trucks[i].current_position != data.depotIndex) {
            double T_Return = trucks[i].available_time +
                getTruckDistance(data, trucks[i].current_position, data.depotIndex)
                / data.truckSpeed * 60.0;
            trucks[i].route.push_back(data.depotIndex);
            trucks[i].arrival_times.push_back(T_Return);
            trucks[i].departure_times.push_back(T_Return);
            trucks[i].available_time = T_Return;
            C_max = max(C_max, T_Return);
        }
        TruckRouteInfo info;
        info.truck_id = i;
        info.route = trucks[i].route;
        info.arrival_times = trucks[i].arrival_times;
        info.departure_times = trucks[i].departure_times;
        info.completion_time = trucks[i].available_time;
        sol.truck_details.push_back(info);
        if (trucks[i].route.size() > 2)
            sol.routes.push_back(trucks[i].route);
    }

    sol.totalCost = C_max;
    if (sol.totalPenalty > 1.0) sol.isFeasible = false;
    return sol;
}

// Extract encoding from a greedy-decoded solution
AssignmentEncoding initFromSolution(
    const vector<int>& seq,
    const PDPSolution& sol,
    const PDPData& data
) {
    int n = (int)seq.size();
    AssignmentEncoding enc;
    enc.truck_assign.resize(n, 0);
    enc.drone_assign.resize(n, 0);
    enc.break_bit.resize(n, 1);

    // Map customer_id -> truck_id from truck_details
    map<int, int> cust_truck;
    for (const auto& td : sol.truck_details)
        for (int node : td.route)
            if (data.isCustomer(node))
                cust_truck[node] = td.truck_id;

    // Map customer_id -> (drone_id+1, event_index) from resupply_events
    map<int, int> cust_drone;
    map<int, int> cust_event;
    for (int e = 0; e < (int)sol.resupply_events.size(); e++)
        for (int c : sol.resupply_events[e].customer_ids) {
            cust_drone[c] = sol.resupply_events[e].drone_id + 1;
            cust_event[c] = e;
        }

    int prev_event = -1, prev_drone = -1, prev_truck = -1;
    for (int i = 0; i < n; i++) {
        int c = seq[i];
        if (!data.isCustomer(c)) continue;

        if (cust_truck.count(c)) enc.truck_assign[i] = cust_truck[c];
        if (cust_drone.count(c)) enc.drone_assign[i] = cust_drone[c];

        // Break bit: 0 if same event+drone+truck as previous D customer
        if (cust_event.count(c)) {
            int ev = cust_event[c];
            int dr = enc.drone_assign[i];
            int tr = enc.truck_assign[i];
            enc.break_bit[i] = (ev == prev_event && dr == prev_drone && tr == prev_truck) ? 0 : 1;
            prev_event = ev;
            prev_drone = dr;
            prev_truck = tr;
        }
    }
    return enc;
}

// Assignment-based local search: flip truck_assign, drone_assign, break_bit
PDPSolution runAssignmentLS(
    const vector<int>& seq,
    AssignmentEncoding& enc,
    const PDPData& data,
    int max_iter
) {
    PDPSolution best_sol = decodeFromEncoding(seq, enc, data);
    double best_cost = best_sol.totalCost + best_sol.totalPenalty * 1000.0;
    int n = (int)seq.size();

    // Precompute DL partner indices for P nodes
    vector<int> dl_partner(n, -1);
    for (int i = 0; i < n; i++) {
        int c = seq[i];
        if (!data.isCustomer(c)) continue;
        if (data.nodeTypes[c] == "P" && data.pairIds[c] > 0) {
            int pair_id = data.pairIds[c];
            for (int j = 0; j < n; j++) {
                if (j != i && data.isCustomer(seq[j]) &&
                    data.nodeTypes[seq[j]] == "DL" &&
                    data.pairIds[seq[j]] == pair_id) {
                    dl_partner[i] = j;
                    break;
                }
            }
        }
    }

    for (int iter = 0; iter < max_iter; iter++) {
        // Best-improvement: find the best move across all operators
        double iter_best_cost = best_cost;
        int best_op = -1, best_i = -1, best_j = -1, best_val = -1;
        PDPSolution iter_best_sol;

        for (int i = 0; i < n; i++) {
            int c = seq[i];
            if (!data.isCustomer(c)) continue;
            string ctype = data.nodeTypes[c];

            // === OP1: Flip truck assignment (P and D only, skip DL) ===
            if (ctype != "DL") {
                int old_truck = enc.truck_assign[i];
                int di = dl_partner[i];
                int dl_old = (di >= 0) ? enc.truck_assign[di] : -1;

                for (int t = 0; t < data.numTrucks; t++) {
                    if (t == old_truck) continue;
                    enc.truck_assign[i] = t;
                    if (di >= 0) enc.truck_assign[di] = t;

                    PDPSolution sol = decodeFromEncoding(seq, enc, data);
                    double cost = sol.totalCost + sol.totalPenalty * 1000.0;
                    if (cost < iter_best_cost - 0.01) {
                        iter_best_cost = cost;
                        iter_best_sol = sol;
                        best_op = 1; best_i = i; best_val = t;
                    }

                    enc.truck_assign[i] = old_truck;
                    if (di >= 0) enc.truck_assign[di] = dl_old;
                }
            }

            // === OP2: Flip drone/depot (type D only) ===
            if (ctype == "D" && data.readyTimes[c] > 0) {
                int old_drone = enc.drone_assign[i];
                for (int d = 0; d <= data.numDrones; d++) {
                    if (d == old_drone) continue;
                    enc.drone_assign[i] = d;

                    PDPSolution sol = decodeFromEncoding(seq, enc, data);
                    double cost = sol.totalCost + sol.totalPenalty * 1000.0;
                    if (cost < iter_best_cost - 0.01) {
                        iter_best_cost = cost;
                        iter_best_sol = sol;
                        best_op = 2; best_i = i; best_val = d;
                    }

                    enc.drone_assign[i] = old_drone;
                }
            }

            // === OP3: Flip break bit (type D with drone > 0) ===
            if (ctype == "D" && data.readyTimes[c] > 0 && enc.drone_assign[i] > 0) {
                enc.break_bit[i] = 1 - enc.break_bit[i];

                PDPSolution sol = decodeFromEncoding(seq, enc, data);
                double cost = sol.totalCost + sol.totalPenalty * 1000.0;
                if (cost < iter_best_cost - 0.01) {
                    iter_best_cost = cost;
                    iter_best_sol = sol;
                    best_op = 3; best_i = i;
                }

                enc.break_bit[i] = 1 - enc.break_bit[i];  // revert
            }

            // === OP4: Swap truck assignment between two customers ===
            if (ctype != "DL") {
                for (int j = i + 1; j < n; j++) {
                    int c2 = seq[j];
                    if (!data.isCustomer(c2)) continue;
                    if (data.nodeTypes[c2] == "DL") continue;
                    if (enc.truck_assign[i] == enc.truck_assign[j]) continue;

                    // Swap trucks
                    int ti = enc.truck_assign[i], tj = enc.truck_assign[j];
                    int di = dl_partner[i], dj = dl_partner[j];
                    int di_old = (di >= 0) ? enc.truck_assign[di] : -1;
                    int dj_old = (dj >= 0) ? enc.truck_assign[dj] : -1;

                    enc.truck_assign[i] = tj;
                    enc.truck_assign[j] = ti;
                    if (di >= 0) enc.truck_assign[di] = tj;
                    if (dj >= 0) enc.truck_assign[dj] = ti;

                    PDPSolution sol = decodeFromEncoding(seq, enc, data);
                    double cost = sol.totalCost + sol.totalPenalty * 1000.0;
                    if (cost < iter_best_cost - 0.01) {
                        iter_best_cost = cost;
                        iter_best_sol = sol;
                        best_op = 4; best_i = i; best_j = j;
                    }

                    // Revert
                    enc.truck_assign[i] = ti;
                    enc.truck_assign[j] = tj;
                    if (di >= 0) enc.truck_assign[di] = di_old;
                    if (dj >= 0) enc.truck_assign[dj] = dj_old;
                }
            }

            // === OP5: Swap drone assignment between two D customers ===
            if (ctype == "D" && data.readyTimes[c] > 0) {
                for (int j = i + 1; j < n; j++) {
                    int c2 = seq[j];
                    if (!data.isCustomer(c2)) continue;
                    if (data.nodeTypes[c2] != "D" || data.readyTimes[c2] <= 0) continue;
                    if (enc.drone_assign[i] == enc.drone_assign[j]) continue;

                    int di_val = enc.drone_assign[i], dj_val = enc.drone_assign[j];
                    enc.drone_assign[i] = dj_val;
                    enc.drone_assign[j] = di_val;

                    PDPSolution sol = decodeFromEncoding(seq, enc, data);
                    double cost = sol.totalCost + sol.totalPenalty * 1000.0;
                    if (cost < iter_best_cost - 0.01) {
                        iter_best_cost = cost;
                        iter_best_sol = sol;
                        best_op = 5; best_i = i; best_j = j;
                    }

                    // Revert
                    enc.drone_assign[i] = di_val;
                    enc.drone_assign[j] = dj_val;
                }
            }
        }

        // Apply the best move found
        if (best_op < 0) break;  // no improving move found

        best_sol = iter_best_sol;
        best_cost = iter_best_cost;

        if (best_op == 1) {
            enc.truck_assign[best_i] = best_val;
            int di = dl_partner[best_i];
            if (di >= 0) enc.truck_assign[di] = best_val;
        } else if (best_op == 2) {
            enc.drone_assign[best_i] = best_val;
        } else if (best_op == 3) {
            enc.break_bit[best_i] = 1 - enc.break_bit[best_i];
        } else if (best_op == 4) {
            int ti = enc.truck_assign[best_i], tj = enc.truck_assign[best_j];
            enc.truck_assign[best_i] = tj;
            enc.truck_assign[best_j] = ti;
            int di = dl_partner[best_i], dj = dl_partner[best_j];
            if (di >= 0) enc.truck_assign[di] = tj;
            if (dj >= 0) enc.truck_assign[dj] = ti;
        } else if (best_op == 5) {
            int di_val = enc.drone_assign[best_i], dj_val = enc.drone_assign[best_j];
            enc.drone_assign[best_i] = dj_val;
            enc.drone_assign[best_j] = di_val;
        }
    }

    return best_sol;
}

// =========================================================
// === HAM DANH GIA (FITNESS FUNCTION) ===
// =========================================================

/**
 * @brief Ham danh gia fitness - GAN TUNG KHACH CHO XE RANH NHAT
 * @param seq Chromosome (chi thu tu khach hang, khong co separator)
 * @param data Du lieu bai toan
 * @return PDPSolution chua C_max (totalCost) va totalPenalty
 */
PDPSolution decodeAndEvaluate(const vector<int>& seq, const PDPData& data) {
    PDPSolution sol;
    sol.totalCost = 0.0;     
    sol.totalPenalty = 0.0;
    sol.isFeasible = true;
    sol.original_sequence = seq;  // Luu sequence goc

    if (seq.empty()) {
        sol.totalPenalty = 1e9;
        sol.isFeasible = false;
        return sol;
    }

    double C_max = 0.0;
    
    
    // Khoi tao trang thai cac xe tai (su dung TruckState da dinh nghia o dau file)
    vector<TruckState> trucks(data.numTrucks);
    for (int i = 0; i < data.numTrucks; ++i) {
        trucks[i].available_time = 0.0;
        trucks[i].current_position = data.depotIndex;
        trucks[i].current_load = 0.0;
        trucks[i].route.push_back(data.depotIndex); // Bat dau tu depot
        trucks[i].arrival_times.push_back(0.0);
        trucks[i].departure_times.push_back(0.0);
    }
    
    // Trang thai drone
    vector<double> Drone_Available_Time(data.numDrones, 0.0);
    sol.drone_completion_times.resize(data.numDrones, 0.0);

    // Tinh so khach toi da de cho cho batching
    int max_batch_size = max(3, (int)(data.numCustomers * 0.05));
    
    // Duyet tung khach hang trong sequence
    for (int seq_idx = 0; seq_idx < seq.size(); ++seq_idx) {
        int v_id = seq[seq_idx];
        if (!data.isCustomer(v_id)) continue;
        
        string v_type = data.nodeTypes[v_id];
        int v_ready = data.readyTimes[v_id];
        int v_demand = data.demands[v_id];
        int v_pairId = data.pairIds[v_id];
        double e_v = (double)v_ready;
        
        // ===== KIEM TRA P-DL PRECEDENCE CONSTRAINT =====
        // Neu day la DL (delivery), phai kiem tra P tuong ung da duoc pickup chua
        if (v_type == "DL" && v_pairId > 0) {
            // Tim xe nao da pickup cap nay
            int pickup_truck_id = -1;
            for (int t = 0; t < data.numTrucks; ++t) {
                if (trucks[t].picked_up_pairs.count(v_pairId)) {
                    pickup_truck_id = t;
                    break;
                }
            }
            
            // Neu chua co xe nao pickup -> VI PHAM PRECEDENCE
            if (pickup_truck_id == -1) {
                sol.totalPenalty += 10000; // Penalty nghiem trong
                sol.isFeasible = false;
                continue; // Skip khach hang nay
            }
            
            // Bß║«T BUß╗ÿC g├ín cho xe ─æ├ú pickup
            int truck_id = pickup_truck_id;
            TruckState& truck = trucks[truck_id];
            
            // Thuc hien delivery cho xe nay (khong can chon xe ranh nhat)
            double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
            // DL: Truck da co hang roi, KHONG can doi ready time
            double T_Start = T_Arr;
            truck.available_time = T_Start + data.truckServiceTime;
            truck.current_position = v_id;
            truck.route.push_back(v_id);
            truck.arrival_times.push_back(T_Arr);
            truck.departure_times.push_back(truck.available_time);
            
            // Cß║¡p nhß║¡t load (delivery ΓåÆ giß║úm load)
            truck.current_load += v_demand; // v_demand = -1 cho DL
            
            // Kiem tra capacity (load khong duoc am)
            if (truck.current_load < -0.01) { // Tolerance cho floating point
                sol.totalPenalty += 1000;
                sol.isFeasible = false;
            }
            if (truck.current_load > data.truckCapacity) {
                sol.totalPenalty += 1000;
                sol.isFeasible = false;
            }
            
            // Xoa cap nay khoi picked_up_pairs (da hoan thanh)
            truck.picked_up_pairs.erase(v_pairId);
            
            C_max = max(C_max, truck.available_time);
            continue; // Da xu ly xong DL, chuyen sang khach tiep theo
        }
        
        // ===== XU LY CAC LOAI KHACH HANG KHAC (P, D) =====
        
        // Chon xe tai ranh som nhat
        int truck_id = 0;
        for (int i = 1; i < data.numTrucks; ++i) {
            if (trucks[i].available_time < trucks[truck_id].available_time) {
                truck_id = i;
            }
        }
        
        TruckState& truck = trucks[truck_id];
        
        // PH╞»╞áNG ├üN A: ─Éi ngay lß║¡p tß╗⌐c (logic c┼⌐)
        double cost_immediate = numeric_limits<double>::max();
        
        // PHUONG AN B: Cho de lay batch khach hang
        double cost_batch = numeric_limits<double>::max();
        bool can_batch = false;
        vector<int> batch_customers;
        
        // Thu thap khach hang cho batch (chi P va DL customers)
        if (v_type == "P" || v_type == "DL") {
            batch_customers.push_back(v_id);
            
            // Tim them khach hang P/DL trong sequence
            for (int i = seq_idx + 1; i < seq.size() && batch_customers.size() < max_batch_size; ++i) {
                int next_customer = seq[i];
                if (data.isCustomer(next_customer) && 
                    (data.nodeTypes[next_customer] == "P" || data.nodeTypes[next_customer] == "DL")) {
                    batch_customers.push_back(next_customer);
                }
            }
            
            can_batch = (batch_customers.size() > 1);
        }
        
        // TINH PHUONG AN A: Di ngay lap tuc
        
        // TINH PHUONG AN A: Di ngay lap tuc
        
        // DRONE-ELIGIBLE CUSTOMER (Type D)
        if (v_type == "D" && v_ready > 0) {
            // ===== KIEM TRA HANG CO SAN TREN XE CHUA =====
            bool cargo_already_on_truck = truck.cargo_on_truck.count(v_id) > 0;
            
            if (cargo_already_on_truck) {
                // ==== TRUONG HOP 1: HANG DA CO SAN -> GIAO HANG TRUOC ====
                // Truck da lay hang tu depot/drone roi, khong can doi ready_time nua
                double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
                double T_Start = T_Arr; // Khong can doi ready_time vi hang da co tren xe
                double T_End_Delivery = T_Start + data.truckServiceTime;
                
                truck.available_time = T_End_Delivery;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Arr);
                truck.departure_times.push_back(T_End_Delivery);
                
                // Giao hang -> giam load
                truck.current_load -= v_demand; // v_demand = 1, nen load giam 1
                truck.cargo_on_truck.erase(v_id); // Xoa khoi danh sach hang tren xe
                
                // Kiem tra capacity
                if (truck.current_load < -0.01) {
                    sol.totalPenalty += 1000;
                    sol.isFeasible = false;
                }
                if (truck.current_load > data.truckCapacity) {
                    sol.totalPenalty += 1000;
                    sol.isFeasible = false;
                }
                
                C_max = max(C_max, truck.available_time);
                continue; // Da giao hang xong, khong can resupply
            }
            
            // ==== TRUONG HOP 2: HANG CHUA CO -> CAN DRONE RESUPPLY (WITH CONSOLIDATION) ====
            
            // Tim cac customers co the consolidate trong 1 drone trip
            int drone_capacity = data.getDroneCapacity();
            vector<int> consolidation_candidates = findConsolidationCandidates(
                seq, seq_idx, data, truck.current_position, drone_capacity
            );
            
            // DISABLE CONSOLIDATION: Chi lay 1 customer per trip
            if (!ENABLE_CONSOLIDATION && !consolidation_candidates.empty()) {
                consolidation_candidates.resize(1);  // Chi giu lai customer dau tien
            }
            
            // DEBUG: In ra sß╗æ ß╗⌐ng vi├¬n consolidation (chß╗ë khi > 1)
            // if (consolidation_candidates.size() > 1) {
            //     cout << "[CONSOLIDATION] Node " << v_id 
            //          << " found " << consolidation_candidates.size() << " candidates: ";
            //     for (int c : consolidation_candidates) cout << c << " ";
            //     cout << endl;
            // }
            
            // ========== SO SANH THONG MINH: BATCH VS RIENG LE ==========
            // Thay vi chi so sanh completion_time, ta tinh:
            // - Chi phi batch: thoi gian hoan thanh batch
            // - Chi phi rieng le (uoc tinh): tong thoi gian neu phuc vu tung khach rieng
            // Chon phuong an co tong chi phi thap hon
            
            double Best_Effective_Cost = numeric_limits<double>::max();
            int best_drone_id = -1;
            vector<int> best_consolidation;
            ResupplyEvent best_event;
            int best_num_customers = 0;
            
            // Luu thong tin cua tung batch de so sanh
            struct BatchInfo {
                int num_custs;
                int drone_id;
                double completion_time;
                double effective_cost;  // Chi phi hieu qua (tinh den viec tiet kiem trips)
                bool feasible;
                ResupplyEvent event;
            };
            vector<BatchInfo> all_batches;
            
            // Thu tung so luong customers (tu capacity xuong 1)
            for (int num_custs = min((int)consolidation_candidates.size(), drone_capacity); num_custs >= 1; --num_custs) {
                vector<int> current_batch(consolidation_candidates.begin(), 
                                         consolidation_candidates.begin() + num_custs);
                
                // Thu tat ca cac drones
                for (int d_id = 0; d_id < data.numDrones; ++d_id) {
                    auto result = evaluateDroneTrip(
                        current_batch, d_id, truck, data, Drone_Available_Time
                    );
                    double completion_time = result.first;
                    bool feasible = result.second;
                    
                    if (!feasible) continue;
                    
                    // Tinh chi phi hieu qua:
                    // Neu phuc vu N khach trong 1 trip, ta tiet kiem (N-1) trips
                    // Moi trip tiet kiem ~= depot_load_time + 2*avg_fly_time + overhead
                    // Uoc tinh: moi trip rieng le ton them ~15-20 phut (load + bay ve)
                    double trip_overhead = data.depotDroneLoadTime + 10.0; // thoi gian overhead moi trip
                    double trips_saved = num_custs - 1;
                    double savings = trips_saved * trip_overhead;
                    
                    // Effective cost = completion_time - savings (bonus cho batch lon)
                    double effective_cost = completion_time - savings;
                    
                    // Tao event cho batch nay
                    ResupplyEvent event;
                    event.customer_ids = current_batch;
                    event.drone_id = d_id;
                    event.truck_id = truck_id;
                    
                    double max_ready = 0.0;
                    for (int cust : current_batch) {
                        max_ready = max(max_ready, (double)data.readyTimes[cust]);
                    }
                    
                    double T_Drone_Ready = max(Drone_Available_Time[d_id], max_ready);
                    event.drone_depart_time = T_Drone_Ready + data.depotDroneLoadTime;
                    
                    // ONE RENDEZVOUS: Chi co 1 resupply point (customer dau tien)
                    int resupply_point = current_batch[0];
                    event.resupply_point = resupply_point;
                    
                    // Drone bay tu depot -> resupply point (CHI 1 DIEM)
                    double t_fly_to_resupply = getDroneDistance(data, data.depotIndex, resupply_point) / data.droneSpeed * 60.0;
                    double drone_arrive = event.drone_depart_time + t_fly_to_resupply;
                    event.drone_arrive_time = drone_arrive;
                    
                    // Truck den resupply point
                    double truck_travel = getTruckDistance(data, truck.current_position, resupply_point) / data.truckSpeed * 60.0;
                    double truck_arrive = truck.available_time + truck_travel;
                    event.truck_arrive_time = truck_arrive;
                    
                    // Resupply: Drone giao TAT CA packages cho truck tai 1 diem
                    double resupply_start = max(drone_arrive, truck_arrive);
                    double wait = resupply_start - drone_arrive;
                    double resupply_end = resupply_start + data.resupplyTime;
                    event.resupply_start_time = resupply_start;
                    event.resupply_end_time = resupply_end;
                    
                    // Drone ve depot NGAY LAP TUC
                    double t_return = getDroneDistance(data, resupply_point, data.depotIndex) / data.droneSpeed * 60.0;
                    event.drone_return_time = resupply_end + t_return;
                    event.total_flight_time = t_fly_to_resupply + wait + t_return;
                    
                    // Truck tu di giao hang cho cac customers
                    double truck_delivery_time = resupply_end;
                    int current_truck_pos = resupply_point;
                    for (int cust : current_batch) {
                        double travel = getTruckDistance(data, current_truck_pos, cust) / data.truckSpeed * 60.0;
                        truck_delivery_time += travel + data.truckServiceTime;
                        current_truck_pos = cust;
                    }
                    event.truck_delivery_end = truck_delivery_time;
                    
                    all_batches.push_back({num_custs, d_id, completion_time, effective_cost, true, event});
                }
            }
            
            // Chon batch tot nhat dua tren effective_cost
            for (const auto& batch : all_batches) {
                // DEBUG output (tß║»t khi chß║íy thß╗▒c)
                // if (consolidation_candidates.size() > 1) {
                //     cout << "  [SMART] size=" << batch.num_custs << " drone=" << batch.drone_id 
                //          << " completion=" << fixed << setprecision(1) << batch.completion_time
                //          << " effective=" << batch.effective_cost 
                //          << " best=" << Best_Effective_Cost << endl;
                // }
                
                if (batch.effective_cost < Best_Effective_Cost) {
                    Best_Effective_Cost = batch.effective_cost;
                    best_drone_id = batch.drone_id;
                    best_consolidation = batch.event.customer_ids;
                    best_num_customers = batch.num_custs;
                    best_event = batch.event;
                }
            }
            
            // Lay completion time thuc su cua batch duoc chon
            // C_max = max(xe cuoi cung ve kho, khach cuoi cung duoc giao)
            // Drone return time khong anh huong C_max; chi tinh thoi gian truck giao xong
            double Best_Cost_Resupply = best_event.truck_delivery_end;

            // T├¡nh ph╞░╞íng ├ín DJ2: Truck vß╗ü depot
            double t_prev_to_depot = getTruckDistance(data, truck.current_position, data.depotIndex) / data.truckSpeed * 60.0;
            double T_Arr_Depot = truck.available_time + t_prev_to_depot;
            double T_Ready_Leave_Depot = max(T_Arr_Depot + data.depotReceiveTime, (double)v_ready);
            double t_depot_to_v = getTruckDistance(data, data.depotIndex, v_id) / data.truckSpeed * 60.0;
            double T_Arr_Customer = T_Ready_Leave_Depot + t_depot_to_v;
            double Cost_DepotReturn = max(T_Arr_Customer, e_v) + data.truckServiceTime;

            // Chß╗ìn ph╞░╞íng ├ín tß╗æt nhß║Ñt (DJ1 with consolidation vs DJ2)
            if (Best_Cost_Resupply < Cost_DepotReturn && best_drone_id != -1) {
                // DJ1: Drone resupply vß╗¢i consolidation (ONE RENDEZVOUS)
                sol.resupply_events.push_back(best_event);
                
                // ONE RENDEZVOUS MODEL:
                // 1. Truck den resupply point (customer dau tien)
                // 2. Nhan TAT CA packages tu drone tai do
                // 3. Truck giao hang cho resupply_point (vi no cung la khach hang)
                // 4. Truck tu di giao cho cac customers con lai
                
                int resupply_point = best_event.resupply_point;
                
                // Truck den resupply point, nhan hang tu drone, ROI GIAO HANG cho resupply_point
                // departure = resupply_end + service_time (vi resupply_point cung can duoc giao hang)
                double departure_from_resupply = best_event.resupply_end_time + data.truckServiceTime;
                
                truck.route.push_back(resupply_point);
                truck.arrival_times.push_back(best_event.truck_arrive_time);
                truck.departure_times.push_back(departure_from_resupply);
                truck.current_position = resupply_point;
                truck.available_time = departure_from_resupply;
                
                // Truck di giao cho cac customers CON LAI trong batch (ngoai resupply_point da giao o tren)
                for (int cust_id : best_consolidation) {
                    if (cust_id == resupply_point) continue; // Da giao hang o tren roi
                    
                    // Truck di den customer
                    double travel = getTruckDistance(data, truck.current_position, cust_id) / data.truckSpeed * 60.0;
                    double arrival = truck.available_time + travel;
                    double departure = arrival + data.truckServiceTime;
                    
                    truck.route.push_back(cust_id);
                    truck.arrival_times.push_back(arrival);
                    truck.departure_times.push_back(departure);
                    truck.current_position = cust_id;
                    truck.available_time = departure;
                }
                
                // Kiem tra capacity
                if (truck.current_load > data.truckCapacity) {
                    sol.totalPenalty += 1000;
                    sol.isFeasible = false;
                }
                
                Drone_Available_Time[best_drone_id] = best_event.drone_return_time;
                sol.drone_completion_times[best_drone_id] = best_event.drone_return_time;
                C_max = max(C_max, truck.available_time);
                
                // Skip cac customers da duoc xu ly trong consolidation
                set<int> processed_set(best_consolidation.begin(), best_consolidation.end());
                while (seq_idx + 1 < seq.size()) {
                    int next_cust = seq[seq_idx + 1];
                    // Skip BAT KY customer nao da duoc xu ly (bao gom ca v_id)
                    if (processed_set.count(next_cust)) {
                        seq_idx++; // Skip customer nay
                    } else {
                        break;
                    }
                }
            } else {
                // DJ2: Truck về depot
                // Chỉ thêm depot vào route nếu xe KHÔNG ở depot
                if (truck.current_position != data.depotIndex) {
                    double t_to_depot = getTruckDistance(data, truck.current_position, data.depotIndex) / data.truckSpeed * 60.0;
                    double T_Arr_Depot = truck.available_time + t_to_depot;
                    
                    truck.route.push_back(data.depotIndex);
                    truck.arrival_times.push_back(T_Arr_Depot);
                    truck.departure_times.push_back(T_Arr_Depot + data.depotReceiveTime);
                    
                    truck.available_time = T_Arr_Depot + data.depotReceiveTime;
                    truck.current_position = data.depotIndex;
                    truck.current_load = 0.0; // Reset load khi về depot
                    truck.cargo_on_truck.clear(); // Xóa tất cả hàng trên xe
                }
                
                // Đợi ready time của khách hàng nếu cần
                // IMPORTANT: Xe chỉ có thể rời depot SAU KHI hàng đã sẵn sàng (ready_time)
                double T_Depart_Depot = max(truck.available_time, (double)v_ready);
                
                // Cập nhật departure_time của depot hiện tại
                // Nếu xe đang ở depot ban đầu (route chỉ có 1 node), cập nhật departure_times[0]
                // Nếu xe vừa quay về depot (route có nhiều node), cập nhật departure_times cuối
                if (truck.route.size() >= 1 && truck.route.back() == data.depotIndex) {
                    truck.departure_times.back() = T_Depart_Depot;
                }
                
                truck.available_time = T_Depart_Depot;
                
                // Lấy hàng từ depot (đã sẵn sàng tại depot)
                truck.current_load += v_demand;
                truck.cargo_on_truck.insert(v_id); // Hàng được lấy từ depot
                
                // Đi đến khách hàng
                double t_to_customer = getTruckDistance(data, data.depotIndex, v_id) / data.truckSpeed * 60.0;
                double T_Arr_Customer = truck.available_time + t_to_customer;
                double T_Start = max(T_Arr_Customer, e_v);
                
                truck.available_time = T_Start + data.truckServiceTime;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Arr_Customer);
                truck.departure_times.push_back(truck.available_time);
                
                // Giao h├áng ΓåÆ giß║úm load
                truck.current_load -= v_demand; // ─É├ú giao h├áng cho kh├ích
                truck.cargo_on_truck.erase(v_id); // X├│a khß╗Åi danh s├ích h├áng tr├¬n xe
                
                if (truck.current_load > data.truckCapacity) {
                    sol.totalPenalty += 1000;
                    sol.isFeasible = false;
                }
                if (truck.current_load < -0.01) {
                    sol.totalPenalty += 1000;
                    sol.isFeasible = false;
                }
            }
        } 
        // PICKUP/DELIVERY (Type P/DL) - ├üp dß╗Ñng batching strategy
        else {
            // PH╞»╞áNG ├üN A: ─Éi ngay lß║¡p tß╗⌐c
            double T_Arr_A = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
            double T_Start_A = max(T_Arr_A, e_v);
            cost_immediate = T_Start_A + data.truckServiceTime;
            
            // PH╞»╞áNG ├üN B: Batching - chß╗ë t├¡nh nß║┐u c├│ thß╗â batch
            if (can_batch) {
                // T├¡nh thß╗¥i gian ho├án th├ánh khi ─æi batch
                double total_batch_time = truck.available_time;
                double current_pos = truck.current_position;
                double total_load_change = 0;
                bool batch_feasible = true;
                
                for (int batch_customer : batch_customers) {
                    // Di chuyß╗ân ─æß║┐n kh├ích h├áng tiß║┐p theo
                    double travel_time = getTruckDistance(data, current_pos, batch_customer) / data.truckSpeed * 60.0;
                    double arrival_time = total_batch_time + travel_time;
                    double service_start = max(arrival_time, (double)data.readyTimes[batch_customer]);
                    
                    total_batch_time = service_start + data.truckServiceTime;
                    current_pos = batch_customer;
                    
                    // Kiß╗âm tra capacity cho batch
                    total_load_change += data.demands[batch_customer];
                    if (truck.current_load + total_load_change > data.truckCapacity || 
                        truck.current_load + total_load_change < 0) {
                        batch_feasible = false;
                        break;
                    }
                }
                
                if (batch_feasible) {
                    cost_batch = total_batch_time;
                }
            }
            
            // CHß╗îN PH╞»╞áNG ├üN Tß╗ÉT NHß║ñT Tß╗¬ A, B
            
            // T├¼m ph╞░╞íng ├ín c├│ cost nhß╗Å nhß║Ñt
            double best_cost = cost_immediate;
            string best_strategy = "immediate";
            
            if (can_batch && cost_batch < best_cost) {
                best_cost = cost_batch;
                best_strategy = "batch";
            }
            
            // Thực hiện phương án tốt nhất
            if (best_strategy == "batch") {
                // Thực hiện BATCH (Phương án B)
                set<int> processed_customers;
                
                for (int batch_customer : batch_customers) {
                    // Nếu xe ở depot và cần đợi ready_time, cập nhật departure_time của depot
                    if (truck.current_position == data.depotIndex && 
                        truck.route.size() >= 1 && 
                        truck.route.back() == data.depotIndex) {
                        double ready_time = (double)data.readyTimes[batch_customer];
                        if (ready_time > truck.available_time) {
                            truck.available_time = ready_time;
                            truck.departure_times.back() = ready_time;
                        }
                    }
                    
                    double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, batch_customer) / data.truckSpeed * 60.0;
                    double T_Start = max(T_Arr, (double)data.readyTimes[batch_customer]);
                    truck.available_time = T_Start + data.truckServiceTime;
                    truck.current_position = batch_customer;
                    truck.route.push_back(batch_customer);
                    truck.arrival_times.push_back(T_Arr);
                    truck.departure_times.push_back(truck.available_time);

                    // Cập nhật load
                    truck.current_load += data.demands[batch_customer];
                    
                    // Nếu là pickup, đánh dấu đã pickup cặp này và thêm hàng vào xe
                    if (data.nodeTypes[batch_customer] == "P" && data.pairIds[batch_customer] > 0) {
                        truck.picked_up_pairs.insert(data.pairIds[batch_customer]);
                        truck.cargo_on_truck.insert(batch_customer); // H├áng ─æ╞░ß╗úc lß║Ñy l├¬n xe
                    }
                    // DL kh├┤ng cß║ºn th├¬m v├áo cargo_on_truck
                    
                    if (truck.current_load > data.truckCapacity) sol.totalPenalty += 1000;
                    if (truck.current_load < 0) sol.totalPenalty += 1000;
                    
                    processed_customers.insert(batch_customer);
                }
                
                // Skip c├íc kh├ích ─æ├ú ─æ╞░ß╗úc xß╗¡ l├╜ trong batch
                while (seq_idx + 1 < seq.size()) {
                    int next_customer = seq[seq_idx + 1];
                    if (processed_customers.count(next_customer) && next_customer != v_id) {
                        seq_idx++; // Skip customer n├áy
                    } else {
                        break;
                    }
                }
                
            } else {
                // Thực hiện IMMEDIATE (phương án A)
                
                // Nếu xe ở depot và cần đợi ready_time, cập nhật departure_time của depot
                if (truck.current_position == data.depotIndex && 
                    truck.route.size() >= 1 && 
                    truck.route.back() == data.depotIndex &&
                    e_v > truck.available_time) {
                    truck.available_time = e_v;
                    truck.departure_times.back() = e_v;
                }
                
                double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
                double T_Start = max(T_Arr, e_v);
                truck.available_time = T_Start + data.truckServiceTime;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Arr);
                truck.departure_times.push_back(truck.available_time);

                // Cập nhật load
                if (v_type == "P") {
                    truck.current_load += v_demand;
                    truck.cargo_on_truck.insert(v_id); // Hàng được lấy lên xe (chưa giao)
                    // Đánh dấu đã pickup cặp này
                    if (data.pairIds[v_id] > 0) {
                        truck.picked_up_pairs.insert(data.pairIds[v_id]);
                    }
                } else if (v_type == "DL") {
                    truck.current_load += v_demand; // demand âm -> giảm load
                    // Không cần xóa cargo_on_truck vì DL không phải là hàng trên xe
                }
                
                // Kiểm tra capacity
                if (truck.current_load > data.truckCapacity) sol.totalPenalty += 1000;
                if (truck.current_load < 0) sol.totalPenalty += 1000;
            }
        }
        
        C_max = max(C_max, truck.available_time);
    }
    // ========== SMART MERGE STRATEGIES ==========
    // Apply merge strategies to reduce drone trips if beneficial
    
    if (sol.resupply_events.size() >= 2) {
        bool improved = true;
        int merge_iterations = 0;
        const int MAX_MERGE_ITERATIONS = 3; // Prevent infinite loop
        
        while (improved && merge_iterations < MAX_MERGE_ITERATIONS) {
            improved = false;
            merge_iterations++;
            
            // STRATEGY 1: Sequential Merge (highest priority)
            // Merge trips that are consecutive in truck route
            for (int t = 0; t < data.numTrucks; ++t) {
                vector<pair<int, size_t>> truck_events; // <position_in_route, event_index>
                
                for (size_t i = 0; i < sol.resupply_events.size(); ++i) {
                    if (sol.resupply_events[i].truck_id != t) continue;
                    
                    // Find position of first customer in truck route
                    for (size_t pos = 0; pos < trucks[t].route.size(); ++pos) {
                        if (trucks[t].route[pos] == sol.resupply_events[i].customer_ids[0]) {
                            truck_events.push_back({pos, i});
                            break;
                        }
                    }
                }
                
                sort(truck_events.begin(), truck_events.end());
                
                // Try merge consecutive trips (within 2 hops)
                for (size_t i = 0; i + 1 < truck_events.size(); ++i) {
                    int pos1 = truck_events[i].first;
                    int pos2 = truck_events[i+1].first;
                    
                    if (pos2 - pos1 > 2) continue; // Not close enough
                    
                    size_t idx1 = truck_events[i].second;
                    size_t idx2 = truck_events[i+1].second;
                    
                    if (!ENABLE_MERGE_STRATEGIES) continue;
                    
                    auto merge_result = tryMergeTrips(
                        sol.resupply_events[idx1],
                        sol.resupply_events[idx2],
                        data, trucks, Drone_Available_Time
                    );
                    
                    if (merge_result.second > 0.01) { // Improvement threshold
                        sol.resupply_events[idx1] = merge_result.first;
                        sol.resupply_events.erase(sol.resupply_events.begin() + idx2);
                        improved = true;
                        break; // Restart after structure change
                    }
                }
                if (improved) break;
            }
            
            if (improved) continue; // Go to next iteration
            
            // STRATEGY 2: Temporal Clustering (medium priority)
            // Merge trips with similar ready times
            for (size_t i = 0; i < sol.resupply_events.size() && !improved; ++i) {
                for (size_t j = i + 1; j < sol.resupply_events.size(); ++j) {
                    if (sol.resupply_events[i].truck_id != sol.resupply_events[j].truck_id)
                        continue;
                    
                    // Calculate average ready times
                    double avg_ready1 = 0, avg_ready2 = 0;
                    for (int c : sol.resupply_events[i].customer_ids)
                        avg_ready1 += data.readyTimes[c];
                    for (int c : sol.resupply_events[j].customer_ids)
                        avg_ready2 += data.readyTimes[c];
                    
                    avg_ready1 /= sol.resupply_events[i].customer_ids.size();
                    avg_ready2 /= sol.resupply_events[j].customer_ids.size();
                    
                    if (abs(avg_ready1 - avg_ready2) > 20) continue; // Too far apart
                    
                    if (!ENABLE_MERGE_STRATEGIES) continue;
                    
                    auto merge_result = tryMergeTrips(
                        sol.resupply_events[i],
                        sol.resupply_events[j],
                        data, trucks, Drone_Available_Time
                    );
                    
                    if (merge_result.second > 0.01) {
                        sol.resupply_events[i] = merge_result.first;
                        sol.resupply_events.erase(sol.resupply_events.begin() + j);
                        improved = true;
                        break;
                    }
                }
            }
            
            if (improved) continue;
            
            // STRATEGY 3: Spatial Clustering (lowest priority)
            // Merge trips with nearby customers
            for (size_t i = 0; i < sol.resupply_events.size() && !improved; ++i) {
                for (size_t j = i + 1; j < sol.resupply_events.size(); ++j) {
                    if (sol.resupply_events[i].truck_id != sol.resupply_events[j].truck_id)
                        continue;
                    
                    // Calculate average distance between customers
                    double avg_dist = 0;
                    int count = 0;
                    for (int c1 : sol.resupply_events[i].customer_ids) {
                        for (int c2 : sol.resupply_events[j].customer_ids) {
                            avg_dist += getDroneDistance(data, c1, c2);
                            count++;
                        }
                    }
                    avg_dist /= count;
                    
                    if (avg_dist > 5.0) continue; // Too far apart
                    
                    if (!ENABLE_MERGE_STRATEGIES) continue;
                    
                    auto merge_result = tryMergeTrips(
                        sol.resupply_events[i],
                        sol.resupply_events[j],
                        data, trucks, Drone_Available_Time
                    );
                    
                    if (merge_result.second > 0.01) {
                        sol.resupply_events[i] = merge_result.first;
                        sol.resupply_events.erase(sol.resupply_events.begin() + j);
                        improved = true;
                        break;
                    }
                }
            }
        }
    }
    
    // ========== END MERGE STRATEGIES ==========
    
    // ========== PROPAGATE DELAY: Truck phải đợi drone tại resupply point ==========
    // Đây là bước QUAN TRỌNG để tính đúng C_max
    // Khi drone đến sau truck tại resupply point, truck phải đợi
    // Delay này cần được propagate đến tất cả nodes tiếp theo trong route
    
    for (auto& event : sol.resupply_events) {
        if (event.customer_ids.empty()) continue;
        
        int truck_id = event.truck_id;
        if (truck_id < 0 || truck_id >= (int)trucks.size()) continue;
        
        int resupply_point = event.resupply_point;
        if (resupply_point <= 0) resupply_point = event.customer_ids[0];
        
        auto& truck = trucks[truck_id];
        
        // Tìm resupply point trong route của truck
        for (size_t k = 0; k < truck.route.size(); ++k) {
            if (truck.route[k] == resupply_point) {
                // Thời điểm truck có thể rời đi = sau khi resupply xong
                double required_departure = event.resupply_end_time;
                
                // Nếu truck phải đợi drone
                if (required_departure > truck.departure_times[k]) {
                    double delay = required_departure - truck.departure_times[k];
                    truck.departure_times[k] = required_departure;
                    
                    // Propagate delay đến tất cả nodes tiếp theo
                    for (size_t m = k + 1; m < truck.route.size(); ++m) {
                        truck.arrival_times[m] += delay;
                        truck.departure_times[m] += delay;
                    }
                    
                    // Cập nhật available_time của truck
                    truck.available_time = truck.departure_times.back();
                }
                break;
            }
        }
    }
    
    // ========== END PROPAGATE DELAY ==========
    
    // Cập nhật C_max với thời gian hoàn thành thực sự của tất cả resupply events
    // C_max = max(xe cuối cùng về kho, khách cuối cùng được giao)
    // Drone return time không tính vào C_max; chỉ tính thời gian truck giao hàng xong
    for (const auto& event : sol.resupply_events) {
        C_max = max(C_max, event.truck_delivery_end);
    }
    
    // Cập nhật C_max với truck completion times (sau khi propagate delay)
    for (int i = 0; i < data.numTrucks; ++i) {
        C_max = max(C_max, trucks[i].available_time);
    }
    
    // Tất cả xe về depot
    for (int i = 0; i < data.numTrucks; ++i) {
        if (trucks[i].current_position != data.depotIndex) {
            double T_Return_Depot = trucks[i].available_time + 
                getTruckDistance(data, trucks[i].current_position, data.depotIndex) / data.truckSpeed * 60.0;
            
            trucks[i].route.push_back(data.depotIndex);
            trucks[i].arrival_times.push_back(T_Return_Depot);
            trucks[i].departure_times.push_back(T_Return_Depot);
            trucks[i].available_time = T_Return_Depot;
            C_max = max(C_max, T_Return_Depot);
        }
        
        // Lưu thông tin chi tiết của xe
        TruckRouteInfo info;
        info.truck_id = i;
        info.route = trucks[i].route;
        info.arrival_times = trucks[i].arrival_times;
        info.departure_times = trucks[i].departure_times;
        info.completion_time = trucks[i].available_time;
        sol.truck_details.push_back(info);
        
        // Lưu route vào solution (để tương thích)
        if (trucks[i].route.size() > 2) { // Có ít nhất depot-customer-depot
            sol.routes.push_back(trucks[i].route);
        }
    }

    sol.totalCost = C_max;
    if (sol.totalPenalty > 1.0) sol.isFeasible = false;

    // ===== ASSIGNMENT LOCAL SEARCH =====
    if (ENABLE_ASSIGNMENT_LS && !seq.empty()) {
        AssignmentEncoding enc = initFromSolution(seq, sol, data);
        PDPSolution ls_sol = runAssignmentLS(seq, enc, data, ASSIGNMENT_LS_MAX_ITER);
        double ls_cost = ls_sol.totalCost + ls_sol.totalPenalty * 1000.0;
        double cur_cost = sol.totalCost + sol.totalPenalty * 1000.0;
        if (ls_cost < cur_cost - 0.01) {
            sol = ls_sol;
        }
    }

    return sol;
}
