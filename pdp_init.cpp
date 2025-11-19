#include "pdp_init.h" // (File header mới)
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// ============ UTILITY FUNCTIONS (Nội bộ file này) ============

// (Hàm euclideanDistance và buildDistanceMatrix đã được chuyển 
//  sang pdp_reader.cpp và được gọi qua pdp_types.h)

// Hàm tiện ích chung cho cả truck và drone
double getDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    // nodeA_id và nodeB_id đã là 0-based index (depot=0, customers=1,2,3...)
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.distMatrix[nodeA_id][nodeB_id];
}

// Hàm polarAngle (giữ nguyên từ file của bạn)
double polarAngle(const pair<double, double>& depot, const pair<double, double>& customer, bool normalize = false) {
    double dx = customer.first - depot.first;
    double dy = customer.second - depot.second;
    double angle = atan2(dy, dx);
    
    if (normalize && angle < 0) {
        angle += 2 * M_PI;
    }
    
    return angle;
}

// Decode sequence -> routes 
// QUAN TRỌNG: Chromosome KHÔNG chứa depot (depot = node 0)
// Depot được tự động thêm vào đầu/cuối mỗi route
// Input: seq = [customer_nodes + separators] (VD: [1,2,3,4,5,11,6,7,8,9,10])
// Output: routes = [[depot,c1,c2,depot], [depot,c3,c4,depot]] (VD: [[0,1,2,3,4,5,0], [0,6,7,8,9,10,0]])
vector<vector<int>> decodeSeq(const vector<int>& seq, const PDPData& data) {
    vector<vector<int>> routes;
    vector<int> currentRoute;
    currentRoute.push_back(data.depotIndex); // Depot = 0

    for (int id : seq) {
        if (data.isSeparator(id)) {
            currentRoute.push_back(data.depotIndex); // Kết thúc route hiện tại
            if (currentRoute.size() > 2) routes.push_back(currentRoute);
            currentRoute.clear();
            currentRoute.push_back(data.depotIndex); // Bắt đầu route mới
        } else if (data.isCustomer(id)) { // Chỉ thêm customer nodes (1,2,3,...)
            currentRoute.push_back(id);
        }
    }
    // Thêm route cuối cùng
    if (currentRoute.size() > 1) { 
        currentRoute.push_back(data.depotIndex); // Kết thúc tại depot
        if (currentRoute.size() > 2) routes.push_back(currentRoute);
    }
    return routes;
}

// Repair separators (Hàm này bạn đã viết)
void repairSeparators(vector<int>& seq, const PDPData& data, mt19937& gen) {
    int sepStart = data.getSeparatorStart();
    for (auto it = seq.begin(); it != seq.end(); ) {
        if (data.isSeparator(*it)) {
            *it = sepStart; 
            if ((it + 1) != seq.end() && data.isSeparator(*(it + 1))) {
                it = seq.erase(it + 1);
                continue;
            }
        }
        ++it;
    }
    while (!seq.empty() && data.isSeparator(seq.front())) seq.erase(seq.begin());
    while (!seq.empty() && data.isSeparator(seq.back())) seq.pop_back();

    int sepCount = 0;
    for (int id : seq) if (data.isSeparator(id)) ++sepCount;
    int needSep = max(0, data.numTrucks - 1);
    if (sepCount == needSep) return;

    vector<vector<int>> routes;
    vector<int> cur;
    for (int id : seq) {
        if (data.isSeparator(id)) {
            if (!cur.empty()) { routes.push_back(cur); cur.clear(); }
        } else if (data.isCustomer(id)) {
            cur.push_back(id);
        }
    }
    if (!cur.empty()) routes.push_back(cur);

    // Merge (giữ nguyên logic của bạn)
    while (sepCount > needSep && routes.size() > 1) {
        int shortest = 0;
        for (size_t i = 1; i < routes.size(); ++i)
            if (routes[i].size() < routes[shortest].size()) shortest = i;
        
        // Sửa lỗi: Chèn vào tuyến trước đó (an toàn hơn)
        int target = (shortest > 0) ? shortest - 1 : shortest + 1;
        
        if (shortest < target && target < routes.size()) { // Chèn shortest vào target (sau)
             routes[target].insert(routes[target].begin(), routes[shortest].begin(), routes[shortest].end());
        } else if (target >= 0) { // Chèn shortest (sau) vào target (trước)
             routes[target].insert(routes[target].end(), routes[shortest].begin(), routes[shortest].end());
        } else {
             // Trường hợp chỉ có 1 route, không làm gì
        }
        
        if(routes.size() > 1) routes.erase(routes.begin() + shortest);
        sepCount--;
    }
    // Split (giữ nguyên logic của bạn)
    while (sepCount < needSep) {
        if (routes.empty()) { routes.push_back({}); sepCount++; continue; }
        int longest = 0;
        for (size_t i = 1; i < routes.size(); ++i)
            if (routes[i].size() > routes[longest].size()) longest = i;
        if (routes[longest].size() <= 1) {
            routes.push_back({}); // Thêm route rỗng
        } else {
            int sp = routes[longest].size() / 2;
            vector<int> a(routes[longest].begin(), routes[longest].begin() + sp);
            vector<int> b(routes[longest].begin() + sp, routes[longest].end());
            routes[longest] = a;
            routes.insert(routes.begin() + longest + 1, b);
        }
        sepCount++;
    }

    // Rebuild (giữ nguyên logic của bạn)
    seq.clear();
    for (size_t i = 0; i < routes.size(); ++i) {
        for (int id : routes[i]) seq.push_back(id);
        if (i < routes.size() - 1) {
            seq.push_back(sepStart + (int)(i % data.numTrucks));
        }
    }
}

// ============ REPAIR FUNCTIONS (Từ file của bạn) ============

void repairCustomer(vector<int>& seq, const PDPData& data, mt19937& gen) {
    vector<int> count(data.numNodes + 1, 0);
    for (int nodeId : seq) {
        if (data.isCustomer(nodeId)) count[nodeId]++;
    }
    vector<int> missing;
    for (int i = 0; i < (int)data.nodeTypes.size(); ++i) {
        int nodeId = i + 1;
        // SỬA LỖI: Bây giờ isCustomer đã bao gồm P, DL, và D
        if (data.isCustomer(nodeId) && count[nodeId] == 0) {
            missing.push_back(nodeId);
        }
    }
    shuffle(missing.begin(), missing.end(), gen);
    int missingIdx = 0;
    int sepId = data.getSeparatorStart();
    for (int& nodeId : seq) {
        if (data.isCustomer(nodeId) && count[nodeId] > 1) {
            if (missingIdx < (int)missing.size()) {
                int oldNode = nodeId;
                int newNode = missing[missingIdx++];
                nodeId = newNode;
                count[oldNode]--;
                count[newNode]++;
            } else {
                count[nodeId]--;
                nodeId = sepId; 
            }
        }
    }
    // Thêm các khách hàng còn thiếu vào cuối (nếu có)
    for (int i = missingIdx; i < (int)missing.size(); ++i) {
        seq.push_back(missing[i]);
    }
}

void quickRepairPDP(vector<int>& seq, const PDPData& data, mt19937& gen) {
    repairCustomer(seq, data, gen);
    repairSeparators(seq, data, gen);

    vector<vector<int>> routes = decodeSeq(seq, data);
    for (auto& route : routes) {
        map<int, int> pickupPos, deliveryPos;
        for (size_t i = 1; i + 1 < route.size(); ++i) { // Bỏ qua depot
            int nodeIdx = route[i] - 1;
            if (nodeIdx >= 0 && nodeIdx < (int)data.pairIds.size()) {
                int pairId = data.pairIds[nodeIdx];
                string nodeType = data.nodeTypes[nodeIdx];
                if (pairId > 0) {
                    if (nodeType == "P") pickupPos[pairId] = (int)i;
                    else if (nodeType == "DL") deliveryPos[pairId] = (int)i;
                }
            }
        }
        for (const auto& pick : pickupPos) {
            int pid = pick.first;
            int ppos = pick.second;
            auto dit = deliveryPos.find(pid);
            if (dit != deliveryPos.end()) {
                int dpos = dit->second;
                if (ppos >= dpos) {
                    swap(route[ppos], route[dpos]);
                }
            }
        }
    }

    seq.clear();
    int sepStart = data.getSeparatorStart();
    for (size_t r = 0; r < routes.size(); ++r) {
        for (size_t j = 1; j + 1 < routes[r].size(); ++j) { // Chỉ chèn khách
            seq.push_back(routes[r][j]);
        }
        if (r + 1 < routes.size()) {
            seq.push_back(sepStart + (int)(r % data.numTrucks));
        }
    }
}

// ============ INITIALIZATION METHODS (Giữ nguyên) ============

// SỬA LỖI: Các hằng số này nằm trong file .cpp gốc của bạn
const double W_MAX = 30.0;   // minutes, wasted-wait threshold
const double T_MAX = 240.0;  // minutes, max route duration

// (Hàm này vẫn dùng `dist` (Euclidean) cũ, nhưng không sao
//  vì nó chỉ là để khởi tạo)
pair<double,double> simulateRouteMetrics(const vector<int>& customers,
                                                int candidateNode, // 1-based id or 0
                                                const PDPData& data,
                                                const vector<vector<double>>& dist) {
    double time = 0.0;
    int pos = data.depotIndex - 1;
    for (int cid : customers) {
        int idx = cid - 1;
        // Kiểm tra index hợp lệ
        if (idx < 0 || idx >= data.numNodes) continue;
        time += dist[pos][idx] / data.truckSpeed * 60.0;
        if (time < data.readyTimes[idx]) time = data.readyTimes[idx];
        time += data.truckServiceTime;
        pos = idx;
    }
    double wastedAtCandidate = 0.0;
    if (candidateNode > 0) {
        int cidx = candidateNode - 1;
        if (cidx < 0 || cidx >= data.numNodes) return {0, time};
        double travel = dist[pos][cidx] / data.truckSpeed * 60.0;
        double arrival = time + travel;
        wastedAtCandidate = max(0.0, (double)data.readyTimes[cidx] - arrival);
        time = arrival;
        if (time < data.readyTimes[cidx]) time = data.readyTimes[cidx];
        time += data.truckServiceTime;
        pos = cidx;
    }
    time += dist[pos][data.depotIndex - 1] / data.truckSpeed * 60.0;
    return {wastedAtCandidate, time};
}

// Lấy danh sách khách hàng (P, DL, D)
vector<int> getAllCustomerNodes(const PDPData& data) {
    vector<int> customers;
    for (int i = 0; i < data.nodeTypes.size(); i++) {
        if (data.isCustomer(i + 1)) { // Dùng hàm isCustomer đã sửa
            customers.push_back(i + 1); // 1-based indexing
        }
    }
    return customers;
}

// Hàm buildDistanceMatrix cũ (chỉ Euclidean) - Dùng cho các hàm init
// (Bỏ `static` để các hàm init khác có thể dùng)
vector<vector<double>> buildDistanceMatrix(const PDPData& data) {
    int n = data.numNodes;
    vector<vector<double>> dist(n, vector<double>(n));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) dist[i][j] = 0.0;
            else {
                dist[i][j] = euclideanDistance(
                    data.coordinates[i].first, data.coordinates[i].second,
                    data.coordinates[j].first, data.coordinates[j].second
                );
            }
        }
    }
    return dist;
}


vector<vector<int>> initRandomPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    
    // SỬA LỖI: Dùng getAllCustomerNodes để lấy P, DL, và D
    vector<int> customers = getAllCustomerNodes(data);
    vector<vector<double>> dist = buildDistanceMatrix(data); // Vẫn dùng Euclidean
    int sepStart = data.getSeparatorStart();

    for (int p = 0; p < populationSize; ++p) {
        shuffle(customers.begin(), customers.end(), gen);
        vector<vector<int>> routes;
        vector<int> cur;
        for (int cid : customers) {
            // SỬA LỖI: Dùng demand thật
            int demand = (cid-1 < data.demands.size()) ? data.demands[cid-1] : 0;
            int current_load = 0;
            for(int c : cur) current_load += (c-1 < data.demands.size()) ? data.demands[c-1] : 0;

            if (current_load + demand > data.truckCapacity) {
                routes.push_back(cur);
                cur.clear();
            }
            auto metrics = simulateRouteMetrics(cur, cid, data, dist);
            double wasted = metrics.first;
            double dur   = metrics.second;
            if (wasted >= W_MAX || dur > T_MAX) {
                if (!cur.empty()) routes.push_back(cur);
                cur.clear();
            }
            cur.push_back(cid);
        }
        if (!cur.empty()) routes.push_back(cur);
        while (routes.size() < (size_t)data.numTrucks) routes.push_back({});

        vector<int> seq;
        for (size_t i = 0; i < routes.size(); ++i) {
            for (int id : routes[i]) seq.push_back(id);
            if (i + 1 < routes.size()) seq.push_back(sepStart + (int)(i % data.numTrucks));
        }
        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }
    return population;
}

vector<vector<int>> initSweepPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    pair<double,double> depotCoords = data.coordinates[data.depotIndex - 1];
    vector<vector<double>> dist = buildDistanceMatrix(data);
    int sepStart = data.getSeparatorStart();

    vector<pair<int,double>> baseCustomers;
    // SỬA LỖI: Dùng getAllCustomerNodes
    vector<int> all_customers = getAllCustomerNodes(data);
    for (int id : all_customers) {
         baseCustomers.push_back({id, polarAngle(depotCoords, data.coordinates[id-1], true)});
    }

    for (int p = 0; p < populationSize; ++p) {
        auto customers = baseCustomers;
        if (p > 0) {
            uniform_real_distribution<double> angleDist(0.0, 2*M_PI);
            double off = angleDist(gen);
            for (auto &c : customers) c.second = fmod(c.second + off, 2*M_PI);
        }
        sort(customers.begin(), customers.end(), [](auto &a, auto &b){ return a.second < b.second; });

        vector<vector<int>> routes;
        vector<int> cur;
        for (auto &pr : customers) {
            int cid = pr.first;
            // SỬA LỖI: Dùng demand thật
            int demand = (cid-1 < data.demands.size()) ? data.demands[cid-1] : 0;
            int current_load = 0;
            for(int c : cur) current_load += (c-1 < data.demands.size()) ? data.demands[c-1] : 0;

            if (current_load + demand > data.truckCapacity) { routes.push_back(cur); cur.clear(); }
            auto metrics = simulateRouteMetrics(cur, cid, data, dist);
            if (metrics.first >= W_MAX || metrics.second > T_MAX) {
                if (!cur.empty()) routes.push_back(cur);
                cur.clear();
            }
            cur.push_back(cid);
        }
        if (!cur.empty()) routes.push_back(cur);
        while (routes.size() < (size_t)data.numTrucks) routes.push_back({});

        vector<int> seq;
        for (size_t i = 0; i < routes.size(); ++i) {
            for (int id : routes[i]) seq.push_back(id);
            if (i + 1 < routes.size()) seq.push_back(sepStart + (int)(i % data.numTrucks));
        }
        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }
    return population;
}

vector<vector<int>> initGreedyTimePDP(int populationSize, const PDPData& data) {
    // (Hàm này của bạn chỉ xử lý P/DL, chúng ta tạm giữ nguyên,
    //  nó sẽ tạo ra các chromosome chỉ có P/DL)
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    vector<vector<double>> dist = buildDistanceMatrix(data);
    int sepStart = data.getSeparatorStart();

    for (int p = 0; p < populationSize; ++p) {
        vector<vector<int>> routes;
        routes.emplace_back(); 
        vector<int> pairIdsList;
        map<int,pair<int,int>> pairNodes;
        for (int i = 0; i < (int)data.nodeTypes.size(); ++i) {
            if (data.nodeTypes[i] == "P" || data.nodeTypes[i] == "DL") {
                int pid = data.pairIds[i];
                int nid = i+1;
                if (pairNodes.find(pid) == pairNodes.end()) pairNodes[pid] = {-1,-1};
                if (data.nodeTypes[i] == "P") pairNodes[pid].first = nid;
                else pairNodes[pid].second = nid;
            }
        }
        for (auto &kv : pairNodes) if (kv.second.first>0 && kv.second.second>0) pairIdsList.push_back(kv.first);
        shuffle(pairIdsList.begin(), pairIdsList.end(), gen);

        for (int pid : pairIdsList) {
            int pick = pairNodes[pid].first;
            int del  = pairNodes[pid].second;
            vector<int> &cur = routes.back();
            // SỬA LỖI: Dùng demand thật
            int p_demand = (pick-1 < data.demands.size()) ? data.demands[pick-1] : 0;
            int d_demand = (del-1 < data.demands.size()) ? data.demands[del-1] : 0;
            int current_load = 0;
            for(int c : cur) current_load += (c-1 < data.demands.size()) ? data.demands[c-1] : 0;

            if (current_load + p_demand > data.truckCapacity) { // Chỉ check P
                routes.emplace_back();
            }
            auto m1 = simulateRouteMetrics(cur, pick, data, dist);
            if (m1.first >= W_MAX || m1.second > T_MAX) {
                routes.emplace_back();
            }
            routes.back().push_back(pick);
            auto m2 = simulateRouteMetrics(routes.back(), del, data, dist);
            if (m2.first >= W_MAX || m2.second > T_MAX) {
                routes.emplace_back();
                routes.back().push_back(del);
            } else {
                routes.back().push_back(del);
            }
        }
        while (routes.size() < (size_t)data.numTrucks) routes.push_back({});
        vector<int> seq;
        for (size_t i = 0; i < routes.size(); ++i) {
            for (int id : routes[i]) seq.push_back(id);
            if (i + 1 < routes.size()) seq.push_back(sepStart + (int)(i % data.numTrucks));
        }
        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }
    return population;
}

vector<vector<int>> initNearestNeighborPDP(int populationSize, const PDPData& data) {
    // (Hàm này cũng chỉ xử lý P/DL, chúng ta tạm giữ nguyên)
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    vector<vector<double>> dist = buildDistanceMatrix(data);
    int sepStart = data.getSeparatorStart();

    for (int p = 0; p < populationSize; ++p) {
        vector<bool> used(data.numNodes, false);
        for (int i = 0; i < data.numNodes; ++i) if (data.nodeTypes[i] == "D") used[i] = true; 
        used[data.depotIndex-1] = true;
        vector<vector<int>> routes;
        while (true) {
            vector<int> cur;
            int curPos = data.depotIndex - 1;
            vector<int> avail;
            for (int i = 0; i < (int)data.nodeTypes.size(); ++i)
                if ((data.nodeTypes[i]=="P" || data.nodeTypes[i]=="DL") && !used[i]) avail.push_back(i+1);
            if (avail.empty()) break;
            uniform_int_distribution<> sd(0,(int)avail.size()-1);
            int start = avail[sd(gen)];
            auto metrics = simulateRouteMetrics(cur, start, data, dist);
            if (metrics.first >= W_MAX || metrics.second > T_MAX) {}
            cur.push_back(start);
            used[start-1] = true;
            while (true) { // Sửa: Check capacity
                int best = -1;
                double bestDist = numeric_limits<double>::max();
                int pos = cur.empty() ? data.depotIndex - 1 : cur.back() - 1;
                
                int current_load = 0;
                for(int c : cur) current_load += (c-1 < data.demands.size()) ? data.demands[c-1] : 0;

                for (int i = 0; i < (int)data.nodeTypes.size(); ++i) {
                    if ((data.nodeTypes[i]=="P" || data.nodeTypes[i]=="DL") && !used[i]) {
                        int demand = (i < data.demands.size()) ? data.demands[i] : 0;
                        if (current_load + demand > data.truckCapacity) continue; // Bỏ qua nếu quá tải

                        double d = dist[pos][i];
                        if (d < bestDist) { bestDist = d; best = i+1; }
                    }
                }
                if (best == -1) break;
                auto met = simulateRouteMetrics(cur, best, data, dist);
                if (met.first >= W_MAX || met.second > T_MAX) break;
                cur.push_back(best);
                used[best-1] = true;
            }
            routes.push_back(cur);
            if (routes.size() >= (size_t)max(1, data.numTrucks * 2)) break;
        }
        for (int i = 0; i < (int)data.nodeTypes.size(); ++i) {
            if ((data.nodeTypes[i]=="P" || data.nodeTypes[i]=="DL") && !used[i]) {
                routes.push_back({i+1});
            }
        }
        while (routes.size() < (size_t)data.numTrucks) routes.push_back({});
        vector<int> seq;
        for (size_t i = 0; i < routes.size(); ++i) {
            for (int id : routes[i]) seq.push_back(id);
            if (i + 1 < routes.size()) seq.push_back(sepStart + (int)(i % data.numTrucks));
        }
        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }
    return population;
}

// Giữ nguyên hàm structured population của bạn
vector<vector<int>> initStructuredPopulationPDP(int populationSize, const PDPData& data, int runNumber) {
    random_device rd;
    unsigned int seed = rd() + runNumber * 54321;
    mt19937 gen(seed);
    
    int greedyTimeCount = (int)(populationSize * 0.40); 
    int sweepCount = (int)(populationSize * 0.25);      
    int nnCount = (int)(populationSize * 0.20);         
    int randomCount = populationSize - greedyTimeCount - sweepCount - nnCount; 
    
    cout << "PDP Population distribution:" << endl;
    cout << "   Greedy Time: " << greedyTimeCount << endl;
    cout << "   Sweep: " << sweepCount << endl;
    cout << "   Nearest Neighbor: " << nnCount << endl;
    cout << "   Random: " << randomCount << endl;
    
    vector<vector<int>> population;
    
    auto greedyTimePop = initGreedyTimePDP(greedyTimeCount, data);
    population.insert(population.end(), greedyTimePop.begin(), greedyTimePop.end());
    
    auto sweepPop = initSweepPDP(sweepCount, data);
    population.insert(population.end(), sweepPop.begin(), sweepPop.end());
    
    auto nnPop = initNearestNeighborPDP(nnCount, data);
    population.insert(population.end(), nnPop.begin(), nnPop.end());
    
    auto randomPop = initRandomPDP(randomCount, data);
    population.insert(population.end(), randomPop.begin(), randomPop.end());
    
    for (auto& seq : population) {
        quickRepairPDP(seq, data, gen);
    }
    
    cout << "Generated " << population.size() << " PDP individuals with structured methods" << endl;
    return population;
}


// =========================================================
// === HÀM ĐÁNH GIÁ (DECODER) MỚI THEO YÊU CẦU CỦA BẠN ===
// =========================================================

/**
 * @brief Hàm đánh giá (Decoder) Giai đoạn 1.
 * Mô phỏng lịch trình 100% Resupply, sử dụng logic "chọn xe rảnh sớm nhất" (select_truck).
 * @param seq Chromosome (đã được repair)
 * @param data Dữ liệu bài toán
 * @return PDPSolution chứa C_max (totalCost) và totalPenalty.
 */
PDPSolution decodeAndEvaluate(const vector<int>& seq, const PDPData& data) {
    PDPSolution sol;
    sol.totalCost = 0.0;     // Sẽ là C_max (makespan)
    sol.totalPenalty = 0.0;
    sol.isFeasible = true; // Giả định là true ban đầu

    // 1. Giải mã Chromosome thành các tuyến
    vector<vector<int>> routes = decodeSeq(seq, data);
    sol.routes = routes;

    // 2. Chuẩn bị "Người Mô phỏng" (Simulator)
    double C_max = 0.0;
    // Mảng theo dõi thời điểm rảnh của mỗi xe tải
    vector<double> Truck_Available_Time(data.numTrucks, 0.0);
    // (Giả định vô hạn drone, không cần theo dõi Drone_Available_Time)

    // 3. Lặp qua từng tuyến (route) và gán cho xe rảnh nhất
    for (auto& route : sol.routes) { // route là [Depot, c1, c2, ..., Depot]
        if (route.size() < 3) continue; // Bỏ qua route rỗng

        // *** HÀM SELECT_TRUCK (Logic bạn mô tả) ***
        // Tìm xe tải rảnh sớm nhất
        int truck_id = 0;
        for (int i = 1; i < data.numTrucks; ++i) {
            if (Truck_Available_Time[i] < Truck_Available_Time[truck_id]) {
                truck_id = i;
            }
        }
        
        double Current_Time = Truck_Available_Time[truck_id];
        double Current_Load = 0;
        set<int> Packages_on_Truck; // Theo dõi các gói C2 (Pickup-Delivery)
        int Previous_Node_ID = data.depotIndex; // (1-based)

        // a. Bắt đầu tại Depot
        // Xe tải đợi ở depot nếu nó phải quay về từ chuyến trước
        double T_Depart_Depot = (Current_Time == 0.0) ? 0.0 : (Current_Time + data.depotReceiveTime);
        Current_Time = T_Depart_Depot;

        // b. Lặp qua các nút trên tuyến (bỏ qua depot đầu/cuối)
        for (size_t i = 1; i < route.size() - 1; ++i) { 
            int v_id = route[i]; // ID đã là 0-based (depot=0, customers=1,2,3...)
            
            // Kiểm tra tính hợp lệ của index
            if (v_id < 0 || v_id >= data.numNodes) {
                sol.totalPenalty += 1e9; // ID không hợp lệ
                sol.isFeasible = false;
                continue;
            }

            const string& v_nodeType = data.nodeTypes[v_id]; // Sử dụng trực tiếp v_id làm index
            const int& v_readyTime = data.readyTimes[v_id]; 
            const int& v_pairId = data.pairIds[v_id];
            const int& v_demand = data.demands[v_id];
            
            // e_v: Thời gian sớm nhất được phục vụ tại nhà khách
            // (Hiện tại file data không có, nên ta giả định e_v = 0)
            double e_v = 0.0; 

            // Tính thời gian xe đến (dùng ma trận khoảng cách)
            double T_Arrival_Truck = Current_Time + getDistance(data, Previous_Node_ID, v_id) / data.truckSpeed * 60.0;
            double T_Start_Service = 0.0;
            double T_End_Service = 0.0;

            // ========================================================
            // TRƯỜNG HỢP B: v là C1 (Loại 'D') - 100% Resupply
            // (Theo logic bạn cung cấp)
            // ========================================================
            if (v_nodeType == "D" && v_readyTime > 0) {
                double R_v = v_readyTime; // Thời gian hàng sẵn sàng ở KHO
                double t_drone_fly = getDistance(data, data.depotIndex, v_id) / data.droneSpeed * 60.0;
                
                // Drone rời kho sau khi lấy hàng (δd)
                double T_Drone_Depart = max(0.0, (double)R_v) + data.depotDroneLoadTime;
                double T_Drone_Arrival_at_v = T_Drone_Depart + t_drone_fly;

                // [Đồng bộ hóa - Công thức của Giảng viên]
                T_Start_Service = max({T_Arrival_Truck, T_Drone_Arrival_at_v, e_v});
                
                // [Tính thời điểm kết thúc]
                T_End_Service = T_Start_Service + data.resupplyTime; // (Δ)

                // [Kiểm tra $L_d$ (Phạt nếu vi phạm)]
                double T_Drone_Wait = T_Start_Service - T_Drone_Arrival_at_v;
                double t_drone_return = getDistance(data, v_id, data.depotIndex) / data.droneSpeed * 60.0;
                double Total_Mission_Time = t_drone_fly + T_Drone_Wait + data.resupplyTime + t_drone_return;

                if (Total_Mission_Time > data.droneEndurance) {
                    sol.totalPenalty += 1000.0 * (Total_Mission_Time - data.droneEndurance);
                    sol.isFeasible = false;
                }

                // C_max phải bao gồm cả thời gian drone quay về
                double T_Drone_Return_Depot = T_End_Service + t_drone_return;
                C_max = max(C_max, T_Drone_Return_Depot);
            }
            // ========================================================
            // TRƯỜNG HỢP A: v là C2 (Pickup 'P' hoặc Delivery 'DL')
            // (Theo logic bạn cung cấp)
            // ========================================================
            else if (v_nodeType == "P") {
                T_Start_Service = max(T_Arrival_Truck, e_v);
                T_End_Service = T_Start_Service + data.truckServiceTime; // (δ)
                
                Current_Load += v_demand;
                Packages_on_Truck.insert(v_pairId);

                if (Current_Load > data.truckCapacity) {
                    sol.totalPenalty += 1000.0 * (Current_Load - data.truckCapacity);
                    sol.isFeasible = false;
                }
            }
            else if (v_nodeType == "DL") {
                if (Packages_on_Truck.find(v_pairId) == Packages_on_Truck.end()) {
                    sol.totalPenalty += 1e7; // Phạt nặng vi phạm thứ tự P->D
                    sol.isFeasible = false;
                }

                T_Start_Service = max(T_Arrival_Truck, e_v);
                T_End_Service = T_Start_Service + data.truckServiceTime; // (δ)
                
                Current_Load += v_demand; // Demand của DL là âm
                Packages_on_Truck.erase(v_pairId);

                if (Current_Load < 0) { // Lỗi logic
                   sol.totalPenalty += 1e7; 
                   sol.isFeasible = false;
                }
            }
            // (Bỏ qua các nút 'D' có readyTime = 0, vì chúng là depot)
            
            Current_Time = T_End_Service;
            Previous_Node_ID = v_id;
        }

        // c. Kết thúc Chuyến đi (Quay về Depot)
        double T_Return_Depot = Current_Time + getDistance(data, Previous_Node_ID, data.depotIndex) / data.truckSpeed * 60.0;
        Truck_Available_Time[truck_id] = T_Return_Depot;
        C_max = max(C_max, T_Return_Depot);
    }
    
    sol.totalCost = C_max; // Mục tiêu của Giai đoạn 1 là C_max
    
    // Nếu không có phạt, lời giải là hợp lệ
    if (sol.totalPenalty > 1.0) { // Dùng một ngưỡng nhỏ
        sol.isFeasible = false;
    }
    
    return sol;
}