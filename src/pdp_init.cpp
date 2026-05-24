/**
 * @file pdp_init.cpp
 * @brief Khởi tạo quần thể ban đầu cho GA giải VRP-Drone Resupply & Pick-up/Delivery.
 *
 * Refactored: Lời giải được biểu diễn bằng vector<Gene> (Chromosome) thay vì vector<int>.
 *
 * Pipeline cho mỗi cá thể:
 *   1. Heuristic tạo base_seq (vector<int> chứa ID khách hàng)
 *   2. insertBalancedSeparators() chèn số 0 để chia đều tuyến cho numTrucks
 *   3. buildFinalSequence() chuyển base_seq → Chromosome:
 *      a. Wrap từng ID thành Gene{id, {}}
 *      b. Chèn 1–3 depot-return Gene{-1,{}} vào giữa chuỗi
 */

#include "pdp_init.h"
#include "pdp_types.h"
#include "pdp_reader.h"
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <map>
#include <cmath>
#include <limits>
#include <set>
#include <numeric>
#include "pdp_utils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// Cached instance pointer used by buildFinalSequence (set by init* calls).
static const PDPData* g_init_data = nullptr;

// ============================================================
// === UTILITY FUNCTIONS (nội bộ file này) ====================
// ============================================================

double getTruckDistance(const PDPData& data, int a, int b) {
    if (a < 0 || a >= data.numNodes || b < 0 || b >= data.numNodes)
        return numeric_limits<double>::infinity();
    return data.truckDistMatrix[a][b];
}

double getDroneDistance(const PDPData& data, int a, int b) {
    if (a < 0 || a >= data.numNodes || b < 0 || b >= data.numNodes)
        return numeric_limits<double>::infinity();
    return data.droneDistMatrix[a][b];
}

/// Tính góc cực của một điểm so với depot (dùng cho Sweep heuristic)
static double polarAngle(
    const pair<double,double>& depot,
    const pair<double,double>& customer,
    bool normalize = false)
{
    double dx = customer.first  - depot.first;
    double dy = customer.second - depot.second;
    double angle = atan2(dy, dx);
    if (normalize && angle < 0) angle += 2.0 * M_PI;
    return angle;
}

/// Trả về danh sách tất cả node khách hàng (P, DL, D hợp lệ)
static vector<int> getAllCustomerNodes(const PDPData& data) {
    vector<int> customers;
    customers.reserve(data.numNodes);
    for (int i = 0; i < data.numNodes; ++i)
        if (data.isCustomer(i))
            customers.push_back(i);
    return customers;
}

/// Xây dựng ma trận khoảng cách từ truckDistMatrix
static vector<vector<double>> buildDistanceMatrix(const PDPData& data) {
    int n = data.numNodes;
    vector<vector<double>> dist(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (i != j) dist[i][j] = getTruckDistance(data, i, j);
    return dist;
}

// ============================================================
// === BALANCED CHUNKING & BUILD FINAL SEQUENCE ===============
// ============================================================

vector<int> insertBalancedSeparators(const vector<int>& pure_seq, int numTrucks) {
    int N = (int)pure_seq.size();
    if (N == 0 || numTrucks <= 1) return pure_seq;

    int base = N / numTrucks;
    int remainder = N % numTrucks;

    vector<int> result;
    result.reserve(N + numTrucks - 1);

    int idx = 0;
    for (int t = 0; t < numTrucks; ++t) {
        int take = base + (t < remainder ? 1 : 0);
        for (int i = 0; i < take && idx < N; ++i) {
            result.push_back(pure_seq[idx++]);
        }
        if (t < numTrucks - 1) { // Trừ xe cuối cùng
            result.push_back(0); // Separator
        }
    }
    return result;
}

Chromosome buildFinalSequence(const vector<int>& base_seq, mt19937& gen) {
    if (base_seq.empty()) return {};
    if (g_init_data == nullptr) {
        Chromosome fallback;
        fallback.reserve(base_seq.size());
        for (int id : base_seq) fallback.push_back(Gene(id));
        return fallback;
    }

    const PDPData& data = *g_init_data;

    vector<int> customers;
    customers.reserve(base_seq.size());
    for (int id : base_seq) {
        if (data.isCustomer(id)) customers.push_back(id);
    }
    if (customers.empty()) return {};

    const int n = (int)customers.size();

    uniform_real_distribution<double> ratio_dist(0.4, 0.6);
    int split_idx = (int)floor(n * ratio_dist(gen));
    if (n == 1) split_idx = 1;
    split_idx = max(1, min(split_idx, n - 1));

    // Precedence protection: keep P and DL on same truck.
    for (int i = 0; i < split_idx && i < n; ++i) {
        int node_id = customers[i];
        if (data.nodeTypes[node_id] != "P") continue;
        int pair_id = (node_id < (int)data.pairIds.size()) ? data.pairIds[node_id] : 0;
        if (pair_id <= 0) continue;

        int dl_pos = -1;
        for (int j = i + 1; j < n; ++j) {
            int cand = customers[j];
            if (data.nodeTypes[cand] == "DL" && data.pairIds[cand] == pair_id) {
                dl_pos = j;
                break;
            }
        }
        if (dl_pos >= split_idx) {
            split_idx = min(n, dl_pos + 1);
        }
    }

    auto processRoute = [&](int start, int end) {
        vector<Gene> route;
        route.reserve(end - start + 4);

        double truck_time = 0.0;
        int current_pos = data.depotIndex;
        double current_load = 0.0;
        vector<double> drone_avail_time(data.numDrones, 0.0);
        vector<int> pending_resupply;

        auto flush_pending = [&]() {
            if (!pending_resupply.empty() && !route.empty()) {
                int target_node = pending_resupply.back();
                for (int k = (int)route.size() - 1; k >= 0; --k) {
                    if (route[k].node_id == target_node) {
                        route[k].resupply_vector = pending_resupply;
                        break;
                    }
                }
                pending_resupply.clear();
            }
        };

        for (int idx = start; idx < end; ++idx) {
            int node = customers[idx];
            if (!data.isCustomer(node)) continue;

            const string& type = data.nodeTypes[node];
            int demand = data.demands[node];
            int ready_time = data.readyTimes[node];
            double t_travel = getTruckDistance(data, current_pos, node) / data.truckSpeed * 60.0;
            double next_truck_time = truck_time + t_travel;
            if (next_truck_time < (double)ready_time) next_truck_time = (double)ready_time;

            Gene g(node);
            bool used_drone = false;
            if (type == "D" && ready_time > 0 && data.numDrones > 0) {
                // Option 1: Truck returns to depot to pick up the package itself
                double t_to_depot = truck_time + getTruckDistance(data, current_pos, data.depotIndex) / data.truckSpeed * 60.0;
                double t_prep = t_to_depot + data.depotReceiveTime;
                double t_ready = max(t_prep, (double)ready_time);
                double time_if_return_depot = t_ready + getTruckDistance(data, data.depotIndex, node) / data.truckSpeed * 60.0;

                // Option 2: Drone resupplies the truck at the customer node
                int best_d = 0;
                for (int d = 1; d < data.numDrones; ++d) {
                    if (drone_avail_time[d] < drone_avail_time[best_d]) best_d = d;
                }
                double d_start = max(drone_avail_time[best_d], (double)ready_time);
                double d_arrive = d_start + data.depotDroneLoadTime + getDroneDistance(data, data.depotIndex, node) / data.droneSpeed * 60.0;
                double t_truck_straight = truck_time + getTruckDistance(data, current_pos, node) / data.truckSpeed * 60.0;
                
                double time_if_use_drone = max(d_arrive, t_truck_straight) + data.resupplyTime;
                
                // ---- KIỂM TRA ĐIỀU KIỆN DRONE ----
                double t_fly_to_node = getDroneDistance(data, data.depotIndex, node) / data.droneSpeed * 60.0;
                double t_return = getDroneDistance(data, node, data.depotIndex) / data.droneSpeed * 60.0;
                double wait_time = max(0.0, t_truck_straight - d_arrive);
                double total_flight_time = t_fly_to_node + wait_time + t_return;
                
                bool endurance_ok = (total_flight_time <= data.droneEndurance);
                bool can_take = (current_load + demand <= data.truckCapacity);

                // Quyết định Trade-off
                if (time_if_use_drone <= time_if_return_depot && can_take && endurance_ok) {
                    used_drone = true;
                    pending_resupply.push_back(node);
                    
                    drone_avail_time[best_d] = time_if_use_drone + getDroneDistance(data, node, data.depotIndex) / data.droneSpeed * 60.0;
                    
                    if ((int)pending_resupply.size() >= data.getDroneCapacity()) {
                        // Batch full, we can safely assign it to the CURRENT node (g)
                        g.resupply_vector = pending_resupply;
                        pending_resupply.clear();
                    }
                    
                    truck_time = time_if_use_drone;
                    current_load += demand; 
                } else {
                    // Force depot return
                    flush_pending(); // Flush previous batch to the last valid drone node
                    route.push_back(Gene(-1));
                    truck_time = time_if_return_depot;
                    current_load = demand;
                }
            } else {
                // Not a D node, or no drones
                flush_pending(); // Flush any pending drone batch to the last valid drone node
                
                truck_time = next_truck_time;
                
                if (type == "P" || type == "DL") {
                    current_load += demand; // P cộng vào (+), DL trừ đi (-) vì demand của DL là âm
                } else if (type == "D") {
                    current_load += demand; // Xe tải tự mang theo gói hàng D này từ Depot trước đó
                }
            }

            // ---- KIỂM TRA QUÁ TẢI (SAU KHI NHẬN HÀNG Ở NODE NÀY) ----
            if (current_load > data.truckCapacity) {
                route.push_back(Gene(-1)); // Lệnh quay về kho
                double t_to_depot = getTruckDistance(data, node, data.depotIndex) / data.truckSpeed * 60.0;
                double t_from_depot = getTruckDistance(data, data.depotIndex, node) / data.truckSpeed * 60.0;
                // Cộng thêm chi phí đi vòng về kho
                truck_time += t_to_depot + data.depotReceiveTime + t_from_depot;
                
                if (type == "DL") {
                    current_load = 0.0;
                } else if (type == "P" || (type == "D" && !used_drone)) {
                    current_load = demand; // Dỡ hết, chỉ chứa món hàng vừa bốc
                } else {
                    current_load = 0.0;
                }
            }

            route.push_back(g);
            current_pos = node;

            // ---- TRỪ TẢI SAU KHI GIAO (CHỈ ÁP DỤNG NẾU XE TỰ GIAO D/DL) ----
            // Lưu ý: Đối với mô hình này, khi rời khỏi node DL/D, xe tải đã giao xong hàng
            if (type == "D" && !used_drone) {
                current_load -= demand;
            } else if (type == "DL" && demand > 0) {
                current_load -= demand; // Nếu demand truyền vào là dương
            }
        }

        flush_pending();

        return route;
    };

    vector<Gene> route1 = processRoute(0, split_idx);
    vector<Gene> route2 = processRoute(split_idx, n);

    Chromosome final_seq;
    final_seq.reserve(route1.size() + route2.size() + 1);
    final_seq.insert(final_seq.end(), route1.begin(), route1.end());
    final_seq.push_back(Gene(0));
    final_seq.insert(final_seq.end(), route2.begin(), route2.end());
    return final_seq;
}

// ============================================================
// === REPAIR FUNCTIONS =======================================
// ============================================================

/**
 * Repair nội bộ: loại bỏ duplicate khách hàng, bổ sung khách còn thiếu.
 * Chỉ động đến Gene có node_id > 0. Gene đặc biệt (0, -1) không bị thay đổi.
 */
static void repairCustomerGenes(Chromosome& seq, const PDPData& data, mt19937& gen) {
    // Đếm tần suất xuất hiện của từng node_id > 0
    vector<int> count(data.numNodes + 1, 0);
    for (const Gene& g : seq)
        if (g.node_id > 0 && data.isCustomer(g.node_id))
            count[g.node_id]++;

    // Tìm khách còn thiếu (count == 0)
    vector<int> missing;
    for (int i = 0; i < data.numNodes; ++i)
        if (data.isCustomer(i) && count[i] == 0)
            missing.push_back(i);
    shuffle(missing.begin(), missing.end(), gen);

    int missingIdx = 0;

    // Pass 1: Thay thế duplicate bằng khách còn thiếu
    for (Gene& g : seq) {
        if (g.node_id <= 0) continue;  // bỏ qua separator và depot-return
        if (!data.isCustomer(g.node_id)) continue;

        if (count[g.node_id] > 1 && missingIdx < (int)missing.size()) {
            int old_id  = g.node_id;
            int new_id  = missing[missingIdx++];
            g.node_id   = new_id;
            // Giữ nguyên resupply_vector của gene này
            count[old_id]--;
            count[new_id]++;
        }
    }

    // Pass 2: Thêm khách vẫn còn thiếu vào cuối (trước khi gặp phần Truck 2)
    // Tìm vị trí separator để thêm vào đúng phần
    for (int i = missingIdx; i < (int)missing.size(); ++i)
        seq.push_back(Gene(missing[i]));
}

void quickRepairPDP(Chromosome& seq, const PDPData& data, mt19937& gen) {
    // Bước 1: Sửa khách hàng (duplicate/missing) — không động đến node_id 0/-1
    repairCustomerGenes(seq, data, gen);

    // Bước 2: Đảm bảo P đứng trước DL tương ứng (swap nếu cần)
    // Chỉ xét Gene có node_id > 0
    map<int, int> positionP;   // pairId -> index trong seq của node P
    map<int, int> positionDL;  // pairId -> index trong seq của node DL

    for (int i = 0; i < (int)seq.size(); ++i) {
        const Gene& g = seq[i];
        if (g.node_id <= 0) continue;
        if (g.node_id >= (int)data.pairIds.size()) continue;

        int pairId = data.pairIds[g.node_id];
        if (pairId <= 0) continue;

        if (data.nodeTypes[g.node_id] == "P")
            positionP[pairId] = i;
        else if (data.nodeTypes[g.node_id] == "DL")
            positionDL[pairId] = i;
    }

    // Swap P và DL nếu P đứng sau DL
    for (auto it2 = positionP.begin(); it2 != positionP.end(); ++it2) {
        int pairId = it2->first;
        int posP   = it2->second;
        auto itDL = positionDL.find(pairId);
        if (itDL == positionDL.end()) continue;
        int posDL = itDL->second;
        if (posP > posDL) {
            // Chỉ swap node_id; giữ nguyên resupply_vector của cả hai
            swap(seq[posP].node_id, seq[posDL].node_id);
        }
    }
}

// ============================================================
// === 4 HEURISTIC INITIALIZATION =============================
// ============================================================

// --- 1. Random ---
vector<Chromosome> initRandomPDP(
    int populationSize, const PDPData& data, mt19937& gen)
{
    g_init_data = &data;
    vector<int> customers = getAllCustomerNodes(data);
    vector<Chromosome> population;
    population.reserve(populationSize);

    for (int p = 0; p < populationSize; ++p) {
        vector<int> base = customers;
        shuffle(base.begin(), base.end(), gen);
        population.push_back(buildFinalSequence(base, gen));
    }
    return population;
}

// --- 2. Greedy Time ---
vector<Chromosome> initGreedyTimePDP(
    int populationSize, const PDPData& data, mt19937& gen)
{
    g_init_data = &data;
    auto dist = buildDistanceMatrix(data);
    vector<Chromosome> population;
    population.reserve(populationSize);

    for (int p = 0; p < populationSize; ++p) {
        vector<int> base;
        vector<bool> visited(data.numNodes, false);
        visited[data.depotIndex] = true;

        double current_time = 0.0;
        int    current_pos  = data.depotIndex;
        int    total_custs  = (int)getAllCustomerNodes(data).size();

        while ((int)base.size() < total_custs) {
            vector<pair<double,int>> candidates;
            for (int i = 0; i < data.numNodes; ++i) {
                if (visited[i] || !data.isCustomer(i)) continue;
                double travel = dist[current_pos][i] / data.truckSpeed * 60.0;
                double start  = max(current_time + travel, (double)data.readyTimes[i]);
                candidates.push_back({start, i});
            }
            if (candidates.empty()) break;
            sort(candidates.begin(), candidates.end());

            int pool = min(5, max((int)(candidates.size() * 0.1), 1));
            uniform_int_distribution<int> pick(0, pool - 1);
            int chosen = candidates[pick(gen)].second;

            base.push_back(chosen);
            visited[chosen] = true;
            double travel   = dist[current_pos][chosen] / data.truckSpeed * 60.0;
            current_time    = max(current_time + travel, (double)data.readyTimes[chosen])
                              + data.truckServiceTime;
            current_pos     = chosen;
        }
        population.push_back(buildFinalSequence(base, gen));
    }
    return population;
}

// --- 3. Sweep ---
vector<Chromosome> initSweepPDP(
    int populationSize, const PDPData& data, mt19937& gen)
{
    g_init_data = &data;
    auto depotCoords = data.coordinates[data.depotIndex];
    vector<Chromosome> population;
    population.reserve(populationSize);

    for (int p = 0; p < populationSize; ++p) {
        uniform_real_distribution<double> angle_rng(0.0, 2.0 * M_PI);
        double base_angle = angle_rng(gen);

        vector<pair<double,int>> by_angle;
        for (int i = 0; i < data.numNodes; ++i) {
            if (!data.isCustomer(i)) continue;
            double a = polarAngle(depotCoords, data.coordinates[i], true);
            a = fmod(a - base_angle + 2.0 * M_PI, 2.0 * M_PI);
            by_angle.push_back({a, i});
        }
        sort(by_angle.begin(), by_angle.end());

        vector<int>  base;
        vector<bool> visited(data.numNodes, false);
        visited[data.depotIndex] = true;

        while ((int)base.size() < (int)by_angle.size()) {
            vector<pair<double,int>> cands;
            for (int si = 0; si < (int)by_angle.size(); ++si) {
                int aid = by_angle[si].second;
                if (!visited[aid])
                    cands.push_back({by_angle[si].first, aid});
            }
            if (cands.empty()) break;

            int pool = min(5, max((int)(cands.size() * 0.1), 1));
            uniform_int_distribution<int> pick(0, pool - 1);
            int chosen = cands[pick(gen)].second;

            base.push_back(chosen);
            visited[chosen] = true;
        }
        population.push_back(buildFinalSequence(base, gen));
    }
    return population;
}

// --- 4. Nearest Neighbor ---
vector<Chromosome> initNearestNeighborPDP(
    int populationSize, const PDPData& data, mt19937& gen)
{
    g_init_data = &data;
    auto dist = buildDistanceMatrix(data);
    auto all_customers = getAllCustomerNodes(data);
    vector<Chromosome> population;
    population.reserve(populationSize);

    for (int p = 0; p < populationSize; ++p) {
        vector<int>  base;
        vector<bool> visited(data.numNodes, false);
        visited[data.depotIndex] = true;

        // Random node bắt đầu
        uniform_int_distribution<int> start_rng(0, (int)all_customers.size() - 1);
        int current_pos = all_customers[start_rng(gen)];
        base.push_back(current_pos);
        visited[current_pos] = true;

        while ((int)base.size() < (int)all_customers.size()) {
            vector<pair<double,int>> cands;
            for (int i = 0; i < data.numNodes; ++i) {
                if (visited[i] || !data.isCustomer(i)) continue;
                cands.push_back({dist[current_pos][i], i});
            }
            if (cands.empty()) break;
            sort(cands.begin(), cands.end());

            int pool = min(5, max((int)(cands.size() * 0.1), 1));
            uniform_int_distribution<int> pick(0, pool - 1);
            int chosen = cands[pick(gen)].second;

            base.push_back(chosen);
            visited[chosen] = true;
            current_pos     = chosen;
        }
        population.push_back(buildFinalSequence(base, gen));
    }
    return population;
}

// ============================================================
// === STRUCTURED POPULATION (orchestrator) ===================
// ============================================================

vector<Chromosome> initStructuredPopulationPDP(
    int populationSize, const PDPData& data, int runNumber)
{
    unsigned int seed = (random_device{}()) + (unsigned)runNumber * 54321u;
    mt19937 gen(seed);

    int randomCount    = (int)(populationSize * 0.10);
    int greedyCount    = (int)(populationSize * 0.30);
    int sweepCount     = (int)(populationSize * 0.30);
    int nnCount        = populationSize - randomCount - greedyCount - sweepCount;

    cout << "PDP Population (Gene-based encoding):\n"
         << "   Random:           " << randomCount  << "\n"
         << "   Greedy Time:      " << greedyCount  << "\n"
         << "   Sweep:            " << sweepCount   << "\n"
         << "   Nearest Neighbor: " << nnCount      << "\n";

    vector<Chromosome> population;
    population.reserve(populationSize);

    auto append = [&](vector<Chromosome>&& sub) {
        for (auto& c : sub) population.push_back(std::move(c));
    };

    append(initRandomPDP(randomCount, data, gen));
    append(initGreedyTimePDP(greedyCount, data, gen));
    append(initSweepPDP(sweepCount, data, gen));
    append(initNearestNeighborPDP(nnCount, data, gen));

    cout << "Generated " << population.size()
         << " chromosomes (vector<Gene> with separator & depot-return nodes)\n";
    return population;
}