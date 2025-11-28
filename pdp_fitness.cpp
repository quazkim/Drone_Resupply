#include "pdp_fitness.h"
#include "pdp_types.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <set>

using namespace std;

// ============ UTILITY FUNCTIONS ============

// Hàm tiện ích cho truck (Manhattan)
static double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.truckDistMatrix[nodeA_id][nodeB_id];
}

// Hàm tiện ích cho drone (Euclidean)
static double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.droneDistMatrix[nodeA_id][nodeB_id];
}

// =========================================================
// === HÀM ĐÁNH GIÁ (FITNESS FUNCTION) ===
// =========================================================

/**
 * @brief Hàm đánh giá fitness - GÁN TỪNG KHÁCH CHO XE RẢNH NHẤT
 * 
 * Logic mới:
 * - Duyệt sequence theo thứ tự
 * - Với mỗi khách hàng, chọn xe tải có thời gian rảnh sớm nhất
 * - Gán khách hàng đó cho xe đó
 * - Cập nhật thời gian rảnh của xe sau khi phục vụ
 * 
 * @param seq Chromosome (chỉ thứ tự khách hàng, không có separator)
 * @param data Dữ liệu bài toán
 * @return PDPSolution chứa C_max (totalCost) và totalPenalty
 */
PDPSolution decodeAndEvaluate(const vector<int>& seq, const PDPData& data) {
    PDPSolution sol;
    sol.totalCost = 0.0;     
    sol.totalPenalty = 0.0;
    sol.isFeasible = true;

    if (seq.empty()) {
        sol.totalPenalty = 1e9;
        sol.isFeasible = false;
        return sol;
    }

    double C_max = 0.0;
    
    // Trạng thái của mỗi xe tải
    struct TruckState {
        double available_time;      // Thời điểm xe rảnh
        int current_position;       // Vị trí hiện tại của xe
        double current_load;        // Load hiện tại
        vector<int> route;          // Route của xe này
        vector<double> arrival_times;   // Thời gian đến mỗi node
        vector<double> departure_times; // Thời gian rời mỗi node
    };
    
    vector<TruckState> trucks(data.numTrucks);
    for (int i = 0; i < data.numTrucks; ++i) {
        trucks[i].available_time = 0.0;
        trucks[i].current_position = data.depotIndex;
        trucks[i].current_load = 0.0;
        trucks[i].route.push_back(data.depotIndex); // Bắt đầu từ depot
        trucks[i].arrival_times.push_back(0.0);
        trucks[i].departure_times.push_back(0.0);
    }
    
    // Trạng thái drone
    vector<double> Drone_Available_Time(data.numDrones, 0.0);
    sol.drone_completion_times.resize(data.numDrones, 0.0);

    // Tính số khách tối đa để chờ cho batching
    int max_batch_size = max(3, (int)(data.numCustomers * 0.05));
    
    // Duyệt từng khách hàng trong sequence
    for (int seq_idx = 0; seq_idx < seq.size(); ++seq_idx) {
        int v_id = seq[seq_idx];
        if (!data.isCustomer(v_id)) continue;
        
        // Chọn xe tải rảnh sớm nhất
        int truck_id = 0;
        for (int i = 1; i < data.numTrucks; ++i) {
            if (trucks[i].available_time < trucks[truck_id].available_time) {
                truck_id = i;
            }
        }
        
        TruckState& truck = trucks[truck_id];
        
        string v_type = data.nodeTypes[v_id];
        int v_ready = data.readyTimes[v_id];
        int v_demand = data.demands[v_id];
        double e_v = (double)v_ready;
        
        // PHƯƠNG ÁN A: Đi ngay lập tức (logic cũ)
        double cost_immediate = numeric_limits<double>::max();
        
        // PHƯƠNG ÁN B: Chờ để lấy batch khách hàng
        double cost_batch = numeric_limits<double>::max();
        bool can_batch = false;
        vector<int> batch_customers;
        
        // PHƯƠNG ÁN C: Chờ drone rảnh + 2 khách hàng thêm (khi ở depot)
        double cost_wait_drone = numeric_limits<double>::max();
        bool can_wait_drone = false;
        vector<int> wait_customers;
        
        // Thu thập khách hàng cho batch (chỉ P và DL customers)
        if (v_type == "P" || v_type == "DL") {
            batch_customers.push_back(v_id);
            
            // Tìm thêm khách hàng P/DL trong sequence
            for (int i = seq_idx + 1; i < seq.size() && batch_customers.size() < max_batch_size; ++i) {
                int next_customer = seq[i];
                if (data.isCustomer(next_customer) && 
                    (data.nodeTypes[next_customer] == "P" || data.nodeTypes[next_customer] == "DL")) {
                    batch_customers.push_back(next_customer);
                }
            }
            
            can_batch = (batch_customers.size() > 1);
        }
        
        // Thu thập khách hàng cho phương án chờ drone (khi truck ở depot và có drone bận)
        if ((v_type == "P" || v_type == "DL") && truck.current_position == data.depotIndex) {
            // Kiểm tra xem có drone nào đang bận không
            bool any_drone_busy = false;
            for (int d_id = 0; d_id < data.numDrones; ++d_id) {
                if (Drone_Available_Time[d_id] > truck.available_time) {
                    any_drone_busy = true;
                    break;
                }
            }
            
            if (any_drone_busy) {
                wait_customers.push_back(v_id);
                
                // Tìm thêm 2 khách hàng P/DL trong sequence
                for (int i = seq_idx + 1; i < seq.size() && wait_customers.size() < 3; ++i) {
                    int next_customer = seq[i];
                    if (data.isCustomer(next_customer) && 
                        (data.nodeTypes[next_customer] == "P" || data.nodeTypes[next_customer] == "DL")) {
                        wait_customers.push_back(next_customer);
                    }
                }
                
                can_wait_drone = (wait_customers.size() >= 2); // Cần ít nhất 2 khách
            }
        }
        
        // TÍNH PHƯƠNG ÁN A: Đi ngay lập tức
        
        // TÍNH PHƯƠNG ÁN A: Đi ngay lập tức
        
        // DRONE-ELIGIBLE CUSTOMER (Type D)
        if (v_type == "D" && v_ready > 0) {
            // Logic drone giữ nguyên như cũ
            double Cost_Resupply = numeric_limits<double>::max();
            double T_Drone_Return = 0;
            int best_drone_id = -1;

            // THỬ TẤT CẢ CÁC DRONE - Chọn drone tốt nhất (minimize completion time)
            for(int d_id = 0; d_id < data.numDrones; ++d_id) {
                double t_fly = getDroneDistance(data, data.depotIndex, v_id) / data.droneSpeed * 60.0;
                double T_Drone_Ready = max(Drone_Available_Time[d_id], (double)v_ready);
                double T_Drone_Arr = T_Drone_Ready + data.depotDroneLoadTime + t_fly;
                
                double T_Truck_Arr = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
                double T_Start_Resupply = max({T_Truck_Arr, T_Drone_Arr, e_v});
                double T_End_Resupply = T_Start_Resupply + data.resupplyTime; // Chỉ 5 phút resupply

                double T_Wait = T_Start_Resupply - T_Drone_Arr;
                double t_return = getDroneDistance(data, v_id, data.depotIndex) / data.droneSpeed * 60.0;
                
                // Kiểm tra drone endurance
                if (t_fly + T_Wait + data.resupplyTime + t_return <= data.droneEndurance) {
                    // Chọn drone có completion time nhỏ nhất
                    if (T_End_Resupply < Cost_Resupply) {
                        Cost_Resupply = T_End_Resupply;
                        best_drone_id = d_id;
                        T_Drone_Return = T_End_Resupply + t_return - data.truckServiceTime;
                    }
                }
            }

            // Truck về kho (DJ2)
            double t_prev_to_depot = getTruckDistance(data, truck.current_position, data.depotIndex) / data.truckSpeed * 60.0;
            double T_Arr_Depot = truck.available_time + t_prev_to_depot;
            double T_Ready_Leave_Depot = max(T_Arr_Depot + data.depotReceiveTime, (double)v_ready);
            double t_depot_to_v = getTruckDistance(data, data.depotIndex, v_id) / data.truckSpeed * 60.0;
            double T_Arr_Customer = T_Ready_Leave_Depot + t_depot_to_v;
            double Cost_DepotReturn = max(T_Arr_Customer, e_v) + data.truckServiceTime;

            // Chọn phương án tốt nhất (DJ1 vs DJ2) - Giữ nguyên logic
            if (Cost_Resupply < Cost_DepotReturn) {
                // DJ1: Drone resupply
                double T_Truck_Arr = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
                double T_Drone_Ready = max(Drone_Available_Time[best_drone_id], (double)v_ready);
                double t_fly = getDroneDistance(data, data.depotIndex, v_id) / data.droneSpeed * 60.0;
                double T_Drone_Depart = T_Drone_Ready + data.depotDroneLoadTime;
                double T_Drone_Arr = T_Drone_Depart + t_fly;
                double T_Start_Resupply = max({T_Truck_Arr, T_Drone_Arr, e_v});
                double t_return = getDroneDistance(data, v_id, data.depotIndex) / data.droneSpeed * 60.0;
                
                // Lưu resupply event
                ResupplyEvent event;
                event.customer_id = v_id;
                event.drone_id = best_drone_id;
                event.truck_id = truck_id;
                event.drone_depart_time = T_Drone_Depart;  
                event.drone_arrive_time = T_Drone_Arr;
                event.truck_arrive_time = T_Truck_Arr;
                event.resupply_start = T_Start_Resupply;
                event.resupply_end = Cost_Resupply;
                event.drone_return_time = Cost_Resupply + t_return; // Drone return sau khi resupply xong
                sol.resupply_events.push_back(event);
                
                truck.available_time = Cost_Resupply;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Truck_Arr);
                truck.departure_times.push_back(Cost_Resupply);
                
                Drone_Available_Time[best_drone_id] = event.drone_return_time;
                sol.drone_completion_times[best_drone_id] = event.drone_return_time;
                C_max = max(C_max, event.drone_return_time);
                
                // Cập nhật load
                truck.current_load += v_demand;
                if (truck.current_load > data.truckCapacity) {
                    sol.totalPenalty += 1000;
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
                }
                
                // Đợi ready time của khách hàng nếu cần
                truck.available_time = max(truck.available_time, (double)v_ready);
                
                // Đi đến khách hàng
                double t_to_customer = getTruckDistance(data, data.depotIndex, v_id) / data.truckSpeed * 60.0;
                double T_Arr_Customer = truck.available_time + t_to_customer;
                double T_Start = max(T_Arr_Customer, e_v);
                
                truck.available_time = T_Start + data.truckServiceTime;
                truck.current_position = v_id;
                truck.route.push_back(v_id);
                truck.arrival_times.push_back(T_Arr_Customer);
                truck.departure_times.push_back(truck.available_time);
                
                truck.current_load += v_demand;
                if (truck.current_load > data.truckCapacity) {
                    sol.totalPenalty += 1000;
                }
            }
        } 
        // PICKUP/DELIVERY (Type P/DL) - Áp dụng batching strategy
        else {
            // PHƯƠNG ÁN A: Đi ngay lập tức
            double T_Arr_A = truck.available_time + getTruckDistance(data, truck.current_position, v_id) / data.truckSpeed * 60.0;
            double T_Start_A = max(T_Arr_A, e_v);
            cost_immediate = T_Start_A + data.truckServiceTime;
            
            // PHƯƠNG ÁN B: Batching - chỉ tính nếu có thể batch
            if (can_batch) {
                // Tính thời gian hoàn thành khi đi batch
                double total_batch_time = truck.available_time;
                double current_pos = truck.current_position;
                double total_load_change = 0;
                bool batch_feasible = true;
                
                for (int batch_customer : batch_customers) {
                    // Di chuyển đến khách hàng tiếp theo
                    double travel_time = getTruckDistance(data, current_pos, batch_customer) / data.truckSpeed * 60.0;
                    double arrival_time = total_batch_time + travel_time;
                    double service_start = max(arrival_time, (double)data.readyTimes[batch_customer]);
                    
                    total_batch_time = service_start + data.truckServiceTime;
                    current_pos = batch_customer;
                    
                    // Kiểm tra capacity cho batch
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
            
            // PHƯƠNG ÁN C: Chờ drone rảnh + batch khách hàng
            if (can_wait_drone) {
                // Tìm thời điểm drone rảnh sớm nhất
                double earliest_drone_free = numeric_limits<double>::max();
                for (int d_id = 0; d_id < data.numDrones; ++d_id) {
                    earliest_drone_free = min(earliest_drone_free, Drone_Available_Time[d_id]);
                }
                
                // Truck chờ đến khi drone rảnh
                double wait_start_time = max(truck.available_time, earliest_drone_free);
                double total_wait_time = wait_start_time;
                double current_pos = truck.current_position;
                double total_load_change = 0;
                bool wait_feasible = true;
                
                for (int wait_customer : wait_customers) {
                    // Di chuyển đến khách hàng tiếp theo
                    double travel_time = getTruckDistance(data, current_pos, wait_customer) / data.truckSpeed * 60.0;
                    double arrival_time = total_wait_time + travel_time;
                    double service_start = max(arrival_time, (double)data.readyTimes[wait_customer]);
                    
                    total_wait_time = service_start + data.truckServiceTime;
                    current_pos = wait_customer;
                    
                    // Kiểm tra capacity cho wait strategy
                    total_load_change += data.demands[wait_customer];
                    if (truck.current_load + total_load_change > data.truckCapacity || 
                        truck.current_load + total_load_change < 0) {
                        wait_feasible = false;
                        break;
                    }
                }
                
                if (wait_feasible) {
                    cost_wait_drone = total_wait_time;
                }
            }
            
            // CHỌN PHƯƠNG ÁN TỐT NHẤT TỪ A, B, C
            
            // Tìm phương án có cost nhỏ nhất
            double best_cost = cost_immediate;
            string best_strategy = "immediate";
            
            if (can_batch && cost_batch < best_cost) {
                best_cost = cost_batch;
                best_strategy = "batch";
            }
            
            if (can_wait_drone && cost_wait_drone < best_cost) {
                best_cost = cost_wait_drone;
                best_strategy = "wait_drone";
            }
            
            // Thực hiện phương án tốt nhất
            if (best_strategy == "batch") {
                // Thực hiện BATCH (Phương án B)
                set<int> processed_customers;
                
                for (int batch_customer : batch_customers) {
                    double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, batch_customer) / data.truckSpeed * 60.0;
                    double T_Start = max(T_Arr, (double)data.readyTimes[batch_customer]);
                    truck.available_time = T_Start + data.truckServiceTime;
                    truck.current_position = batch_customer;
                    truck.route.push_back(batch_customer);
                    truck.arrival_times.push_back(T_Arr);
                    truck.departure_times.push_back(truck.available_time);

                    // Cập nhật load
                    truck.current_load += data.demands[batch_customer];
                    if (truck.current_load > data.truckCapacity) sol.totalPenalty += 1000;
                    if (truck.current_load < 0) sol.totalPenalty += 1000;
                    
                    processed_customers.insert(batch_customer);
                }
                
                // Skip các khách đã được xử lý trong batch
                while (seq_idx + 1 < seq.size()) {
                    int next_customer = seq[seq_idx + 1];
                    if (processed_customers.count(next_customer) && next_customer != v_id) {
                        seq_idx++; // Skip customer này
                    } else {
                        break;
                    }
                }
                
            } else if (best_strategy == "wait_drone") {
                // Thực hiện WAIT DRONE (Phương án C)
                set<int> processed_customers;
                
                // Tìm thời điểm drone rảnh sớm nhất
                double earliest_drone_free = numeric_limits<double>::max();
                for (int d_id = 0; d_id < data.numDrones; ++d_id) {
                    earliest_drone_free = min(earliest_drone_free, Drone_Available_Time[d_id]);
                }
                
                // Truck chờ đến khi drone rảnh
                truck.available_time = max(truck.available_time, earliest_drone_free);
                
                for (int wait_customer : wait_customers) {
                    double T_Arr = truck.available_time + getTruckDistance(data, truck.current_position, wait_customer) / data.truckSpeed * 60.0;
                    double T_Start = max(T_Arr, (double)data.readyTimes[wait_customer]);
                    truck.available_time = T_Start + data.truckServiceTime;
                    truck.current_position = wait_customer;
                    truck.route.push_back(wait_customer);
                    truck.arrival_times.push_back(T_Arr);
                    truck.departure_times.push_back(truck.available_time);

                    // Cập nhật load
                    truck.current_load += data.demands[wait_customer];
                    if (truck.current_load > data.truckCapacity) sol.totalPenalty += 1000;
                    if (truck.current_load < 0) sol.totalPenalty += 1000;
                    
                    processed_customers.insert(wait_customer);
                }
                
                // Skip các khách đã được xử lý trong wait strategy
                while (seq_idx + 1 < seq.size()) {
                    int next_customer = seq[seq_idx + 1];
                    if (processed_customers.count(next_customer) && next_customer != v_id) {
                        seq_idx++; // Skip customer này
                    } else {
                        break;
                    }
                }
                
            } else {
                // Thực hiện IMMEDIATE (phương án A)
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
                } else if (v_type == "DL") {
                    truck.current_load += v_demand; // demand âm -> giảm load
                }
                
                // Kiểm tra capacity
                if (truck.current_load > data.truckCapacity) sol.totalPenalty += 1000;
                if (truck.current_load < 0) sol.totalPenalty += 1000;
            }
        }
        
        C_max = max(C_max, truck.available_time);
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

    return sol;
}
