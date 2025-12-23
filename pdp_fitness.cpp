#include "pdp_fitness.h"
#include "pdp_types.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <set>

using namespace std;

// ============ CONFIGURATION ============
static const bool ENABLE_MERGE_STRATEGIES = true;   // Set to false to disable merge
static const bool ENABLE_CONSOLIDATION = true;      // Allow multiple packages per trip

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
    
    // Completion time = max(drone return, truck finish delivery)
    double completion_time = max(drone_return_time, truck_time);
    
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
    
    // Chi luu 1 resupply event
    merged.arrive_times.push_back(drone_arrive);
    merged.truck_arrive_times.push_back(truck_arrive);
    merged.resupply_starts.push_back(resupply_start);
    merged.resupply_ends.push_back(resupply_end);
    
    return {merged, improvement};
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
                    
                    // LOGIC DUNG: Chi co 1 resupply point (customer dau tien)
                    int resupply_point = current_batch[0];
                    
                    // Drone bay tu depot -> resupply point
                    double t_fly_to_resupply = getDroneDistance(data, data.depotIndex, resupply_point) / data.droneSpeed * 60.0;
                    double drone_arrive = event.drone_depart_time + t_fly_to_resupply;
                    
                    // Truck den resupply point
                    double truck_travel = getTruckDistance(data, truck.current_position, resupply_point) / data.truckSpeed * 60.0;
                    double truck_arrive = truck.available_time + truck_travel;
                    
                    // Resupply
                    double resupply_start = max(drone_arrive, truck_arrive);
                    double wait = resupply_start - drone_arrive;
                    double resupply_end = resupply_start + data.resupplyTime;
                    
                    // Drone ve depot
                    double t_return = getDroneDistance(data, resupply_point, data.depotIndex) / data.droneSpeed * 60.0;
                    event.drone_return_time = resupply_end + t_return;
                    event.total_flight_time = t_fly_to_resupply + wait + t_return;
                    
                    // Chi luu 1 resupply event
                    event.arrive_times.push_back(drone_arrive);
                    event.truck_arrive_times.push_back(truck_arrive);
                    event.resupply_starts.push_back(resupply_start);
                    event.resupply_ends.push_back(resupply_end);
                    
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
            double Best_Cost_Resupply = best_event.drone_return_time;

            // T├¡nh ph╞░╞íng ├ín DJ2: Truck vß╗ü depot
            double t_prev_to_depot = getTruckDistance(data, truck.current_position, data.depotIndex) / data.truckSpeed * 60.0;
            double T_Arr_Depot = truck.available_time + t_prev_to_depot;
            double T_Ready_Leave_Depot = max(T_Arr_Depot + data.depotReceiveTime, (double)v_ready);
            double t_depot_to_v = getTruckDistance(data, data.depotIndex, v_id) / data.truckSpeed * 60.0;
            double T_Arr_Customer = T_Ready_Leave_Depot + t_depot_to_v;
            double Cost_DepotReturn = max(T_Arr_Customer, e_v) + data.truckServiceTime;

            // Chß╗ìn ph╞░╞íng ├ín tß╗æt nhß║Ñt (DJ1 with consolidation vs DJ2)
            if (Best_Cost_Resupply < Cost_DepotReturn && best_drone_id != -1) {
                // DJ1: Drone resupply vß╗¢i consolidation
                sol.resupply_events.push_back(best_event);
                
                // Cß║¡p nhß║¡t trß║íng th├íi cho Tß║ñT Cß║ó customers trong batch
                for (size_t i = 0; i < best_consolidation.size(); ++i) {
                    int cust_id = best_consolidation[i];
                    double T_End = best_event.resupply_ends[i];
                    
                    truck.available_time = T_End;
                    truck.current_position = cust_id;
                    truck.route.push_back(cust_id);
                    truck.arrival_times.push_back(best_event.truck_arrive_times[i]);
                    truck.departure_times.push_back(T_End);
                    
                    // Drone resupply v├á truck giao ngay cho customer
                    // Load kh├┤ng thay ─æß╗òi v├¼: +1 (nhß║¡n tß╗½ drone) -1 (giao cho kh├ích) = 0
                    // Kh├┤ng cß║ºn insert v├áo cargo_on_truck v├¼ ─æ├ú giao ngay
                    
                    if (truck.current_load > data.truckCapacity) {
                        sol.totalPenalty += 1000;
                        sol.isFeasible = false;
                    }
                    if (truck.current_load < -0.01) {
                        sol.totalPenalty += 1000;
                        sol.isFeasible = false;
                    }
                }
                
                Drone_Available_Time[best_drone_id] = best_event.drone_return_time;
                sol.drone_completion_times[best_drone_id] = best_event.drone_return_time;
                C_max = max(C_max, best_event.drone_return_time);
                
                // Skip c├íc customers ─æ├ú ─æ╞░ß╗úc xß╗¡ l├╜ trong consolidation
                set<int> processed_set(best_consolidation.begin(), best_consolidation.end());
                while (seq_idx + 1 < seq.size()) {
                    int next_cust = seq[seq_idx + 1];
                    if (processed_set.count(next_cust) && next_cust != v_id) {
                        seq_idx++; // Skip customer n├áy
                    } else {
                        break;
                    }
                }
            } else {
                // DJ2: Truck vß╗ü depot
                // Chß╗ë th├¬m depot v├áo route nß║┐u xe KH├öNG ß╗ƒ depot
                if (truck.current_position != data.depotIndex) {
                    double t_to_depot = getTruckDistance(data, truck.current_position, data.depotIndex) / data.truckSpeed * 60.0;
                    double T_Arr_Depot = truck.available_time + t_to_depot;
                    
                    truck.route.push_back(data.depotIndex);
                    truck.arrival_times.push_back(T_Arr_Depot);
                    truck.departure_times.push_back(T_Arr_Depot + data.depotReceiveTime);
                    
                    truck.available_time = T_Arr_Depot + data.depotReceiveTime;
                    truck.current_position = data.depotIndex;
                    truck.current_load = 0.0; // Reset load khi vß╗ü depot
                    truck.cargo_on_truck.clear(); // X├│a tß║Ñt cß║ú h├áng tr├¬n xe
                }
                
                // ─Éß╗úi ready time cß╗ºa kh├ích h├áng nß║┐u cß║ºn
                truck.available_time = max(truck.available_time, (double)v_ready);
                
                // Lß║Ñy h├áng tß╗½ depot (─æ├ú sß║╡n s├áng tß║íi depot)
                truck.current_load += v_demand;
                truck.cargo_on_truck.insert(v_id); // H├áng ─æ╞░ß╗úc lß║Ñy tß╗½ depot
                
                // ─Éi ─æß║┐n kh├ích h├áng
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
            
            // Thß╗▒c hiß╗çn ph╞░╞íng ├ín tß╗æt nhß║Ñt
            if (best_strategy == "batch") {
                // Thß╗▒c hiß╗çn BATCH (Ph╞░╞íng ├ín B)
                set<int> processed_customers;
                
                for (int batch_customer : batch_customers) {
                    double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, batch_customer) / data.truckSpeed * 60.0;
                    double T_Start = max(T_Arr, (double)data.readyTimes[batch_customer]);
                    truck.available_time = T_Start + data.truckServiceTime;
                    truck.current_position = batch_customer;
                    truck.route.push_back(batch_customer);
                    truck.arrival_times.push_back(T_Arr);
                    truck.departure_times.push_back(truck.available_time);

                    // Cß║¡p nhß║¡t load
                    truck.current_load += data.demands[batch_customer];
                    
                    // Nß║┐u l├á pickup, ─æ├ính dß║Ñu ─æ├ú pickup cß║╖p n├áy v├á th├¬m h├áng v├áo xe
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
                // Thß╗▒c hiß╗çn IMMEDIATE (ph╞░╞íng ├ín A)
                double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
                double T_Start = max(T_Arr, e_v);
                truck.available_time = T_Start + data.truckServiceTime;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Arr);
                truck.departure_times.push_back(truck.available_time);

                // Cß║¡p nhß║¡t load
                if (v_type == "P") {
                    truck.current_load += v_demand;
                    truck.cargo_on_truck.insert(v_id); // H├áng ─æ╞░ß╗úc lß║Ñy l├¬n xe (ch╞░a giao)
                    // ─É├ính dß║Ñu ─æ├ú pickup cß║╖p n├áy
                    if (data.pairIds[v_id] > 0) {
                        truck.picked_up_pairs.insert(data.pairIds[v_id]);
                    }
                } else if (v_type == "DL") {
                    truck.current_load += v_demand; // demand ├óm -> giß║úm load
                    // Kh├┤ng cß║ºn x├│a cargo_on_truck v├¼ DL kh├┤ng phß║úi l├á h├áng tr├¬n xe
                }
                
                // Kiß╗âm tra capacity
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
        // Tß║Ñt cß║ú xe vß╗ü depot
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
        
        // L╞░u th├┤ng tin chi tiß║┐t cß╗ºa xe
        TruckRouteInfo info;
        info.truck_id = i;
        info.route = trucks[i].route;
        info.arrival_times = trucks[i].arrival_times;
        info.departure_times = trucks[i].departure_times;
        info.completion_time = trucks[i].available_time;
        sol.truck_details.push_back(info);
        
        // L╞░u route v├áo solution (─æß╗â t╞░╞íng th├¡ch)
        if (trucks[i].route.size() > 2) { // C├│ ├¡t nhß║Ñt depot-customer-depot
            sol.routes.push_back(trucks[i].route);
        }
    }

    sol.totalCost = C_max;
    if (sol.totalPenalty > 1.0) sol.isFeasible = false;

    return sol;
}
