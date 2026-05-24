#include "pdp_fitness.h"
#include "pdp_types.h"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <set>
#include <map>
#include <unordered_map>


using namespace std;

// ============ CONFIGURATION ============
static const bool ENABLE_MERGE_STRATEGIES = true;   // Set to false to disable merge
static const bool ENABLE_CONSOLIDATION = true;      // Allow multiple packages per trip
static const bool ENABLE_ASSIGNMENT_LS = false;      // Disabled: first-improvement LS inside fitness
static const int ASSIGNMENT_LS_MAX_ITER = 50;        // Max iterations for assignment LS

// ============ FORWARD DECLARATIONS ============

PDPSolution decode_sequence(
    const vector<SeqStop>& seq,
    const PDPData& data,
    bool throw_on_infeasible
);

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

// ============================================================
// === IMPLEMENTATION: decode_sequence() & print_decoded_routes()
// === Sử dụng encoding mới: vector<SeqStop>
// ============================================================

// ---- Helper nội bộ: trạng thái xe tải trong decode_sequence ----
struct NewTruckState {
    int    truck_id       = 0;
    double available_time = 0.0;   // Thời điểm xe rảnh (sẵn sàng di chuyển)
    int    current_pos    = 0;     // Vị trí hiện tại (node index)
    double current_load   = 0.0;   // Tải trọng hiện tại

    // Route log
    vector<int>    route;
    vector<double> arrival_times;
    vector<double> departure_times;

    // Hàng đang trên xe: customer_id đang chiếm chỗ
    set<int> cargo_on_truck;   // IDs gói hàng đang trên xe
    set<int> picked_up_pairs;  // pairId của các P đã pickup (chờ DL)
};

// ---- Helper: tính drone event tại một điểm gặp (ONE RENDEZVOUS model) ----
// Trả về (ResupplyEvent, feasible)
static pair<ResupplyEvent, bool> buildDroneEvent(
    int drone_id,
    int truck_id,
    const vector<int>& package_ids,
    int resupply_point,
    double truck_avail_time,    // thời điểm xe sẵn sàng (vừa đến điểm này)
    int    truck_current_pos,   // vị trí xe trước khi đến resupply_point
    double drone_avail_time,
    const PDPData& data
) {
    ResupplyEvent ev;
    ev.customer_ids   = package_ids;
    ev.resupply_point = resupply_point;
    ev.drone_id       = drone_id;
    ev.truck_id       = truck_id;

    // Drone đợi tất cả packages ready tại depot
    double max_ready = 0.0;
    for (int pid : package_ids)
        if (pid > 0 && pid < data.numNodes)
            max_ready = max(max_ready, (double)data.readyTimes[pid]);

    double T_drone_ready  = max(drone_avail_time, max_ready);
    ev.drone_depart_time  = T_drone_ready + data.depotDroneLoadTime;

    // Drone bay depot → resupply_point
    double t_fly_out      = getDroneDistance(data, data.depotIndex, resupply_point)
                            / data.droneSpeed * 60.0;
    ev.drone_arrive_time  = ev.drone_depart_time + t_fly_out;

    // Xe tải đến resupply_point
    double t_truck_travel = getTruckDistance(data, truck_current_pos, resupply_point)
                            / data.truckSpeed * 60.0;
    ev.truck_arrive_time  = truck_avail_time + t_truck_travel;

    // Bắt đầu bàn giao = max(drone, truck); drone chờ tính vào flight time
    ev.resupply_start_time = max(ev.drone_arrive_time, ev.truck_arrive_time);
    double wait_time       = ev.resupply_start_time - ev.drone_arrive_time;
    ev.resupply_end_time   = ev.resupply_start_time + data.resupplyTime;

    // Drone về depot
    double t_fly_back     = getDroneDistance(data, resupply_point, data.depotIndex)
                            / data.droneSpeed * 60.0;
    ev.drone_return_time  = ev.resupply_end_time + t_fly_back;
    ev.total_flight_time  = t_fly_out + wait_time + t_fly_back;

    bool feasible = (ev.total_flight_time <= data.droneEndurance);

    // truck_delivery_end: sau khi nhận hàng, xe đi giao từng package
    double t_del = ev.resupply_end_time;
    int cur      = resupply_point;
    for (int pid : package_ids) {
        if (pid <= 0 || pid >= data.numNodes) continue;
        double tt = getTruckDistance(data, cur, pid) / data.truckSpeed * 60.0;
        t_del += tt + data.truckServiceTime;
        cur    = pid;
    }
    ev.truck_delivery_end = t_del;

    return {ev, feasible};
}

// ---- Helper: thêm node vào route log ----
static void logVisit(
    NewTruckState& truck, int node,
    double arrival, double departure
) {
    truck.route.push_back(node);
    truck.arrival_times.push_back(arrival);
    truck.departure_times.push_back(departure);
    truck.current_pos    = node;
    truck.available_time = departure;
}

// ---- Helper: xe về depot (node_id == -1) ----
// Dỡ toàn bộ C2 (pickup items), reset load; nhận C1 khi ghé khách sau đó.
static void handleDepotReturn(
    NewTruckState& truck,
    const PDPData& data,
    PDPSolution& sol,
    bool throw_on_infeasible
) {
    double t_travel = getTruckDistance(data, truck.current_pos, data.depotIndex)
                      / data.truckSpeed * 60.0;
    double T_arr    = truck.available_time + t_travel;
    double T_depart = T_arr + data.depotReceiveTime;

    // Kiểm tra tải trọng TRƯỚC khi reset (dùng load hiện tại để phát hiện overload)
    if (truck.current_load > data.truckCapacity + 1e-6) {
        string msg = "Depot return: load=" + to_string(truck.current_load)
                   + " > M_T=" + to_string(data.truckCapacity);
        if (throw_on_infeasible) throw InfeasibleException(msg);
        sol.totalPenalty += 1000.0;
        sol.isFeasible = false;
    }

    // Dỡ C2 (pickup) + nhận C1 delivery mới → reset về 0
    truck.current_load   = 0.0;
    truck.cargo_on_truck.clear();
    truck.picked_up_pairs.clear();

    logVisit(truck, data.depotIndex, T_arr, T_depart);
}

// ================================================================
// === HÀM CHÍNH: decode_sequence
// ================================================================
PDPSolution decode_sequence(
    const vector<SeqStop>& seq,
    const PDPData& data,
    bool throw_on_infeasible
) {
    PDPSolution sol;
    sol.totalCost    = 0.0;
    sol.totalPenalty = 0.0;
    sol.isFeasible   = true;
    sol.drone_completion_times.resize(data.numDrones, 0.0);

    if (seq.empty()) {
        sol.totalPenalty = 1e9;
        sol.isFeasible   = false;
        return sol;
    }

    // ---- Bước 1: Tìm separator (node_id == 0), tách route1 và route2 ----
    int sep_idx = -1;
    for (int i = 0; i < (int)seq.size(); ++i) {
        if (seq[i].node_id == 0) { sep_idx = i; break; }
    }

    vector<SeqStop> route1, route2;
    if (sep_idx == -1) {
        route1 = seq;
    } else {
        route1 = vector<SeqStop>(seq.begin(), seq.begin() + sep_idx);
        route2 = vector<SeqStop>(seq.begin() + sep_idx + 1, seq.end());
    }

    // ---- Bước 2: Khởi tạo trạng thái 2 xe tải ----
    vector<NewTruckState> trucks(data.numTrucks);
    for (int i = 0; i < data.numTrucks; ++i) {
        trucks[i].truck_id        = i;
        trucks[i].available_time  = 0.0;
        trucks[i].current_pos     = data.depotIndex;
        trucks[i].current_load    = 0.0;
        trucks[i].route.push_back(data.depotIndex);
        trucks[i].arrival_times.push_back(0.0);
        trucks[i].departure_times.push_back(0.0);
    }
    vector<double> drone_avail(data.numDrones, 0.0);

    // ---- Bước 3: Hàm xử lý sub-route cho một xe ----
    auto processRoute = [&](const vector<SeqStop>& route, int truck_id) {
        if (truck_id >= data.numTrucks) return;
        NewTruckState& truck = trucks[truck_id];

        for (const SeqStop& stop : route) {

            // ===== node_id == -1: Lệnh quay về Depot =====
            if (stop.node_id == -1) {
                handleDepotReturn(truck, data, sol, throw_on_infeasible);
                continue;
            }

            // ===== node_id <= 0: Bỏ qua (separator còn sót, ...) =====
            if (stop.node_id <= 0) continue;

            int    v_id     = stop.node_id;
            if (v_id >= data.numNodes) {
                sol.totalPenalty += 500.0;
                continue;
            }

            string v_type   = data.nodeTypes[v_id];
            int    v_demand = data.demands[v_id];
            int    v_pairId = data.pairIds[v_id];

            // Tính thời điểm xe đến
            double t_travel = getTruckDistance(data, truck.current_pos, v_id)
                              / data.truckSpeed * 60.0;
            double T_arrive = truck.available_time + t_travel;
            double T_now    = T_arrive; // con trỏ thời gian "hiện tại" tại điểm này

            // ========= BƯỚC 1: GIAO HÀNG CÓ SẴN (Delivery-first) =========
            bool delivered_here = false;
            if (truck.cargo_on_truck.count(v_id)) {
                // Giao hàng: demand của D/DL > 0 (tải giảm)
                truck.current_load -= abs(v_demand);
                truck.cargo_on_truck.erase(v_id);
                T_now += data.truckServiceTime;
                delivered_here = true;

                if (truck.current_load < -1e-6) {
                    string msg = "Node " + to_string(v_id)
                               + ": load<0 sau delivery = " + to_string(truck.current_load);
                    if (throw_on_infeasible) throw InfeasibleException(msg);
                    sol.totalPenalty += 1000.0;
                    sol.isFeasible    = false;
                    truck.current_load = 0.0;
                }
            }

            // ========= BƯỚC 2: NHẬN HÀNG TỪ DRONE (Drone Resupply) =========
            if (!stop.resupply_vector.empty()) {

                // Kiểm tra số lượng gói <= drone capacity
                int drone_cap = data.getDroneCapacity();
                if ((int)stop.resupply_vector.size() > drone_cap) {
                    string msg = "Node " + to_string(v_id)
                               + ": " + to_string(stop.resupply_vector.size())
                               + " gói > drone capacity " + to_string(drone_cap);
                    if (throw_on_infeasible) throw InfeasibleException(msg);
                    sol.totalPenalty += 500.0;
                    sol.isFeasible    = false;
                }

                // Chọn drone rảnh sớm nhất
                int drone_id = 0;
                for (int d = 1; d < data.numDrones; ++d)
                    if (drone_avail[d] < drone_avail[drone_id]) drone_id = d;

                // Vị trí xe TRƯỚC khi đến v_id (để tính truck_arrive_time trong event)
                int prev_pos = truck.current_pos; // current_pos chưa cập nhật

                auto drone_ev_result = buildDroneEvent(
                    drone_id, truck_id,
                    stop.resupply_vector,
                    v_id,                // resupply point = điểm này
                    truck.available_time,// thời điểm xe bắt đầu di chuyển đến v_id
                    prev_pos,
                    drone_avail[drone_id],
                    data
                );
                ResupplyEvent ev  = drone_ev_result.first;
                bool  feasible    = drone_ev_result.second;

                if (!feasible) {
                    string msg = "Drone " + to_string(drone_id)
                               + " flight=" + to_string(ev.total_flight_time)
                               + " > endurance=" + to_string(data.droneEndurance);
                    if (throw_on_infeasible) throw InfeasibleException(msg);
                    sol.totalPenalty += 500.0;
                    sol.isFeasible    = false;
                }

                // Xe phải đợi đến khi bàn giao xong
                T_now = max(T_now, ev.resupply_end_time);

                // Thêm hàng vào xe
                for (int pid : stop.resupply_vector) {
                    if (pid > 0 && pid < data.numNodes) {
                        truck.current_load += data.demands[pid];
                        truck.cargo_on_truck.insert(pid);
                    }
                }

                // Cập nhật drone state
                drone_avail[drone_id] = ev.drone_return_time;
                sol.drone_completion_times[drone_id] = max(
                    sol.drone_completion_times[drone_id], ev.drone_return_time
                );

                // Lưu event + thứ tự bay
                int ev_idx = (int)sol.resupply_events.size();
                sol.resupply_events.push_back(ev);
                sol.drone_order.push_back(ev_idx);
            }

            // ========= BƯỚC 3: PICKUP (Lấy hàng lên xe) =========
            if (!delivered_here) {
                if (v_type == "P") {
                    truck.current_load += v_demand;
                    truck.cargo_on_truck.insert(v_id);
                    if (v_pairId > 0) truck.picked_up_pairs.insert(v_pairId);
                    T_now += data.truckServiceTime;

                } else if (v_type == "DL" && v_pairId > 0) {
                    // Giao hàng DL: phải pickup P trước (precedence)
                    if (truck.picked_up_pairs.count(v_pairId)) {
                        truck.current_load += v_demand; // v_demand âm (= -demand)
                        truck.picked_up_pairs.erase(v_pairId);
                        T_now += data.truckServiceTime;
                    } else {
                        // Vi phạm precedence
                        sol.totalPenalty += 10000.0;
                        sol.isFeasible    = false;
                    }

                } else if (v_type == "D") {
                    // Loại D nhưng không có resupply → coi như thăm thông thường
                    T_now += data.truckServiceTime;
                }
            }

            // ========= BƯỚC 4: KIỂM TRA TẢI TRỌNG trước khi rời điểm =========
            if (truck.current_load > data.truckCapacity + 1e-6) {
                string msg = "Node " + to_string(v_id)
                           + ": load=" + to_string(truck.current_load)
                           + " > M_T=" + to_string(data.truckCapacity);
                if (throw_on_infeasible) throw InfeasibleException(msg);
                sol.totalPenalty += 1000.0;
                sol.isFeasible    = false;
            }
            if (truck.current_load < -1e-6) {
                string msg = "Node " + to_string(v_id)
                           + ": load=" + to_string(truck.current_load) + " < 0";
                if (throw_on_infeasible) throw InfeasibleException(msg);
                sol.totalPenalty += 1000.0;
                sol.isFeasible    = false;
                truck.current_load = 0.0;
            }

            // Ghi lại điểm dừng vào route
            logVisit(truck, v_id, T_arrive, T_now);

        } // end for each stop
    }; // end lambda processRoute

    // ---- Bước 4: Chạy processRoute cho từng xe ----
    processRoute(route1, 0);
    if (data.numTrucks >= 2)
        processRoute(route2, 1);

    // ---- Bước 5: Đưa tất cả xe về depot, tính C_max ----
    double C_max = 0.0;
    for (int i = 0; i < data.numTrucks; ++i) {
        NewTruckState& truck = trucks[i];
        if (truck.current_pos != data.depotIndex) {
            double t_back   = getTruckDistance(data, truck.current_pos, data.depotIndex)
                              / data.truckSpeed * 60.0;
            double T_return = truck.available_time + t_back;
            truck.route.push_back(data.depotIndex);
            truck.arrival_times.push_back(T_return);
            truck.departure_times.push_back(T_return);
            truck.available_time = T_return;
        }

        TruckRouteInfo info;
        info.truck_id        = i;
        info.route           = truck.route;
        info.arrival_times   = truck.arrival_times;
        info.departure_times = truck.departure_times;
        info.completion_time = truck.available_time;
        sol.truck_details.push_back(info);

        if ((int)truck.route.size() > 2)
            sol.routes.push_back(truck.route);

        C_max = max(C_max, truck.available_time);
    }

    // C_max từ drone completion times và truck_delivery_end
    for (int d = 0; d < data.numDrones; ++d)
        C_max = max(C_max, sol.drone_completion_times[d]);
    for (const auto& ev : sol.resupply_events)
        C_max = max(C_max, ev.truck_delivery_end);

    sol.totalCost = C_max;
    if (sol.totalPenalty > 1.0) sol.isFeasible = false;
    return sol;
}

// ================================================================
// === HÀM IN KẾT QUẢ: print_decoded_routes
// ================================================================
void print_decoded_routes(const PDPSolution& sol, const PDPData& data) {
    cout << "\n"
         << "=========================================================\n"
         << "    KET QUA GIAI MA LO TRINH (decode_sequence)\n"
         << "=========================================================\n";

    cout << fixed << setprecision(2);

    // ---- In lộ trình từng xe tải ----
    for (const auto& td : sol.truck_details) {
        cout << "\n>> XE TAI " << (td.truck_id + 1)
             << "  (Hoan thanh luc: " << td.completion_time << " phut)\n";

        cout << "   Route: ";
        for (int i = 0; i < (int)td.route.size(); ++i) {
            int node = td.route[i];
            if (node == data.depotIndex)
                cout << "[DEPOT]";
            else
                cout << node << "(" << data.nodeTypes[node] << ")";
            if (i + 1 < (int)td.route.size()) cout << " -> ";
        }
        cout << "\n";

        cout << "   Chi tiet:\n";
        for (int i = 0; i < (int)td.route.size(); ++i) {
            int node = td.route[i];
            string label = (node == data.depotIndex)
                           ? "  DEPOT"
                           : ("  Node " + to_string(node)
                              + " [" + data.nodeTypes[node] + "]");
            cout << "    " << setw(20) << left  << label
                 << "  Den: "  << setw(8) << td.arrival_times[i]
                 << "  Roi: "  << setw(8) << td.departure_times[i]
                 << "\n";
        }
    }

    // ---- In thứ tự bay của drone ----
    if (sol.resupply_events.empty()) {
        cout << "\n>> DRONE: Khong co chuyen bay nao.\n";
    } else {
        // Sắp xếp theo drone_depart_time để đúng thứ tự thực tế
        vector<int> order((int)sol.resupply_events.size());
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int a, int b) {
            return sol.resupply_events[a].drone_depart_time
                 < sol.resupply_events[b].drone_depart_time;
        });

        cout << "\n>> THU TU BAY CUA DRONE (" << sol.resupply_events.size() << " chuyen):\n";
        int flight_no = 1;
        for (int idx : order) {
            const ResupplyEvent& ev = sol.resupply_events[idx];
            cout << "\n   [Chuyen " << flight_no++ << "]"
                 << "  Drone " << (ev.drone_id + 1)
                 << " -> Xe tai " << (ev.truck_id + 1) << "\n";
            cout << "   Diem gap (resupply): Node " << ev.resupply_point << "\n";
            cout << "   Goi hang: [ ";
            for (int pid : ev.customer_ids) cout << pid << " ";
            cout << "]\n";
            cout << "   Xuat phat depot:  " << setw(8) << ev.drone_depart_time   << " phut\n";
            cout << "   Drone den diem:   " << setw(8) << ev.drone_arrive_time   << " phut\n";
            cout << "   Xe tai den diem:  " << setw(8) << ev.truck_arrive_time   << " phut\n";
            cout << "   Bat dau ban giao: " << setw(8) << ev.resupply_start_time << " phut\n";
            cout << "   Xong ban giao:    " << setw(8) << ev.resupply_end_time   << " phut\n";
            cout << "   Drone ve depot:   " << setw(8) << ev.drone_return_time   << " phut\n";
            bool over = ev.total_flight_time > data.droneEndurance;
            cout << "   Tong TG bay:      " << setw(8) << ev.total_flight_time   << " phut"
                 << (over ? "  [!] VI PHAM ENDURANCE" : "  [OK]") << "\n";
        }
    }

    // ---- Tổng kết ----
    cout << "\n---------------------------------------------------------\n";
    cout << "  Makespan (C_max): " << sol.totalCost    << " phut\n";
    cout << "  Penalty:          " << sol.totalPenalty << "\n";
    cout << "  Feasible:         " << (sol.isFeasible ? "YES" : "NO") << "\n";
    cout << "---------------------------------------------------------\n\n";
}

// ================================================================
// === BRIDGE FUNCTIONS: vector<int> → decode_sequence
// ================================================================

/// Helper nội bộ: chuyển vector<int> node_ids thành vector<SeqStop>
static vector<SeqStop> idsToSeqStops(const vector<int>& ids) {
    vector<SeqStop> seq;
    seq.reserve(ids.size());
    for (int id : ids) {
        seq.push_back(SeqStop(id));
    }
    return seq;
}

PDPSolution evaluateWithCache(const vector<int>& sequence,
                              const PDPData& data,
                              SolutionCache& cache) {
    // Kiểm tra cache
    if (cache.contains(sequence)) {
        cache.recordHit();
        return cache.get(sequence);
    }
    cache.recordMiss();

    // Chuyển đổi và decode
    vector<SeqStop> seq = idsToSeqStops(sequence);
    PDPSolution sol = decode_sequence(seq, data, false);

    // Lưu cache
    cache.put(sequence, sol);
    return sol;
}

PDPSolution decodeAndEvaluate(const vector<int>& sequence,
                              const PDPData& data) {
    vector<SeqStop> seq = idsToSeqStops(sequence);
    return decode_sequence(seq, data, false);
}
