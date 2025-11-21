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
#include "pdp_utils.h" // Thêm utils để lấy euclideanDistance

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// ============ UTILITY FUNCTIONS (Nội bộ file này) ============

// Hàm tiện ích chung cho truck (Manhattan)
double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.truckDistMatrix[nodeA_id][nodeB_id];
}

// Hàm tiện ích chung cho drone (Euclidean)
double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id) {
    if (nodeA_id < 0 || nodeA_id >= data.numNodes || nodeB_id < 0 || nodeB_id >= data.numNodes) 
        return numeric_limits<double>::infinity();
    return data.droneDistMatrix[nodeA_id][nodeB_id];
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

// Decode sequence -> routes (KHÔNG CÓ SEPARATOR)
// Input: seq = [c1,c2,c3,...,cn] (chỉ thứ tự khách hàng)
// Output: Một route duy nhất [[depot,c1,c2,...,cn,depot]]
vector<vector<int>> decodeSeq(const vector<int>& seq, const PDPData& data) {
    vector<vector<int>> routes;
    
    if (seq.empty()) return routes;
    
    // Tạo một route duy nhất chứa tất cả khách hàng theo thứ tự
    vector<int> route;
    route.push_back(data.depotIndex); // Bắt đầu từ depot
    
    for (int id : seq) {
        if (data.isCustomer(id)) { // Chỉ thêm customer nodes
            route.push_back(id);
        }
    }
    
    route.push_back(data.depotIndex); // Kết thúc tại depot
    
    if (route.size() > 2) { // Có ít nhất 1 khách hàng
        routes.push_back(route);
    }
    
    return routes;
}

// Lấy danh sách tất cả khách hàng (P, DL, D)
vector<int> getAllCustomerNodes(const PDPData& data) {
    vector<int> customers;
    for (int i = 0; i < data.numNodes; i++) {
        if (data.isCustomer(i)) { // Dùng hàm isCustomer
            customers.push_back(i); // 0-based index
        }
    }
    return customers;
}

// Hàm buildDistanceMatrix - Dùng truck distance (Manhattan)
vector<vector<double>> buildDistanceMatrix(const PDPData& data) {
    int n = data.numNodes;
    vector<vector<double>> dist(n, vector<double>(n));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) dist[i][j] = 0.0;
            else {
                dist[i][j] = getTruckDistance(data, i, j); 
            }
        }
    }
    return dist;
}

// Repair separators (KHÔNG CẦN THIẾT VÌ KHÔNG CÓ SEPARATOR)
void repairSeparators(vector<int>& seq, const PDPData& data, mt19937& gen) {
    // Không làm gì vì không có separator
    return;
}

// ============ REPAIR FUNCTIONS ============

void repairCustomer(vector<int>& seq, const PDPData& data, mt19937& gen) {
    vector<int> count(data.numNodes + 1, 0);
    for (int nodeId : seq) {
        if (data.isCustomer(nodeId)) count[nodeId]++;
    }
    vector<int> missing;
    for (int i = 0; i < (int)data.numNodes; ++i) {
        int nodeId = i;
        if (data.isCustomer(nodeId) && count[nodeId] == 0) {
            missing.push_back(nodeId);
        }
    }
    shuffle(missing.begin(), missing.end(), gen);
    int missingIdx = 0;
    
    // Loại bỏ duplicate và thay thế bằng missing
    for (int& nodeId : seq) {
        if (data.isCustomer(nodeId) && count[nodeId] > 1) {
            if (missingIdx < (int)missing.size()) {
                int oldNode = nodeId;
                int newNode = missing[missingIdx++];
                nodeId = newNode;
                count[oldNode]--;
                count[newNode]++;
            }
        }
    }
    
    // Thêm các khách hàng còn thiếu vào cuối
    for (int i = missingIdx; i < (int)missing.size(); ++i) {
        seq.push_back(missing[i]);
    }
}

void quickRepairPDP(vector<int>& seq, const PDPData& data, mt19937& gen) {
    repairCustomer(seq, data, gen);
    
    // Đảm bảo P đứng trước DL
    map<int, int> pairToNodeP;   // pairId -> nodeIdx của P
    map<int, int> pairToNodeDL;  // pairId -> nodeIdx của DL
    map<int, int> positionP;     // pairId -> vị trí của P trong seq
    map<int, int> positionDL;    // pairId -> vị trí của DL trong seq
    
    // Tìm vị trí của P và DL
    for (size_t i = 0; i < seq.size(); ++i) {
        int nodeIdx = seq[i];
        if (nodeIdx >= 0 && nodeIdx < (int)data.pairIds.size()) {
            int pairId = data.pairIds[nodeIdx];
            if (pairId > 0) {
                if (data.nodeTypes[nodeIdx] == "P") {
                    pairToNodeP[pairId] = nodeIdx;
                    positionP[pairId] = i;
                } else if (data.nodeTypes[nodeIdx] == "DL") {
                    pairToNodeDL[pairId] = nodeIdx;
                    positionDL[pairId] = i;
                }
            }
        }
    }
    
    // Swap nếu P đứng sau DL
    for (auto& pair : positionP) {
        int pairId = pair.first;
        int posP = pair.second;
        
        auto it = positionDL.find(pairId);
        if (it != positionDL.end()) {
            int posDL = it->second;
            if (posP > posDL) {
                // Swap P và DL
                swap(seq[posP], seq[posDL]);
            }
        }
    }
}

// ============ INITIALIZATION METHODS (VIẾT LẠI - KHÔNG CÓ SEPARATOR) ============

// Hàm Random: Chọn ngẫu nhiên khách hàng tiếp theo
vector<vector<int>> initRandomPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    
    vector<int> customers = getAllCustomerNodes(data);
    
    for (int p = 0; p < populationSize; ++p) {
        vector<int> seq = customers;
        shuffle(seq.begin(), seq.end(), gen);
        
        // Không có separator, chỉ có thứ tự khách hàng
        population.push_back(seq);
    }
    return population;
}

// Hàm Greedy Time: Tham lam theo ready_time + thời gian đi
vector<vector<int>> initGreedyTimePDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    vector<vector<double>> dist = buildDistanceMatrix(data);
    
    for (int p = 0; p < populationSize; ++p) {
        vector<int> seq;
        vector<bool> visited(data.numNodes, false);
        visited[data.depotIndex] = true;
        
        double current_time = 0.0;
        int current_pos = data.depotIndex;
        
        while (seq.size() < getAllCustomerNodes(data).size()) {
            // Tìm tất cả khách chưa được phục vụ
            vector<pair<double, int>> candidates; // {cost, nodeId}
            
            for (int i = 0; i < data.numNodes; ++i) {
                if (visited[i] || !data.isCustomer(i)) continue;
                
                // Tính cost = ready_time + travel_time
                double travel_time = dist[current_pos][i] / data.truckSpeed * 60.0;
                double arrival_time = current_time + travel_time;
                double start_time = max(arrival_time, (double)data.readyTimes[i]);
                double cost = start_time;
                
                candidates.push_back({cost, i});
            }
            
            if (candidates.empty()) break;
            
            // Sắp xếp theo cost tăng dần
            sort(candidates.begin(), candidates.end());
            
            // Chọn random trong top min(5, max(10% còn lại, 1))
            int pool_size = min(5, max((int)(candidates.size() * 0.1), 1));
            uniform_int_distribution<> dist_choice(0, pool_size - 1);
            int chosen_idx = dist_choice(gen);
            
            int chosen_node = candidates[chosen_idx].second;
            seq.push_back(chosen_node);
            visited[chosen_node] = true;
            
            // Cập nhật thời gian và vị trí
            double travel = dist[current_pos][chosen_node] / data.truckSpeed * 60.0;
            current_time = max(current_time + travel, (double)data.readyTimes[chosen_node]) 
                         + data.truckServiceTime;
            current_pos = chosen_node;
        }
        
        population.push_back(seq);
    }
    return population;
}

// Hàm Sweep: Tham lam theo góc (polar angle)
vector<vector<int>> initSweepPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    pair<double,double> depotCoords = data.coordinates[data.depotIndex];
    
    for (int p = 0; p < populationSize; ++p) {
        // Random một góc gốc
        uniform_real_distribution<double> angle_dist(0.0, 2*M_PI);
        double base_angle = angle_dist(gen);
        
        // Tính góc cho tất cả khách hàng
        vector<pair<double, int>> customers_by_angle;
        for (int i = 0; i < data.numNodes; ++i) {
            if (!data.isCustomer(i)) continue;
            
            double angle = polarAngle(depotCoords, data.coordinates[i], true);
            angle = fmod(angle - base_angle + 2*M_PI, 2*M_PI); // Normalize theo base_angle
            customers_by_angle.push_back({angle, i});
        }
        
        // Sắp xếp theo góc
        sort(customers_by_angle.begin(), customers_by_angle.end());
        
        // Xây dựng sequence với random selection
        vector<int> seq;
        vector<bool> visited(data.numNodes, false);
        visited[data.depotIndex] = true;
        
        while (seq.size() < customers_by_angle.size()) {
            // Tìm khách chưa được chọn
            vector<pair<double, int>> candidates;
            for (auto& p : customers_by_angle) {
                if (!visited[p.second]) {
                    candidates.push_back(p);
                }
            }
            
            if (candidates.empty()) break;
            
            // Chọn random trong top min(5, max(10% còn lại, 1))
            int pool_size = min(5, max((int)(candidates.size() * 0.1), 1));
            uniform_int_distribution<> dist_choice(0, pool_size - 1);
            int chosen_idx = dist_choice(gen);
            
            int chosen_node = candidates[chosen_idx].second;
            seq.push_back(chosen_node);
            visited[chosen_node] = true;
        }
        
        population.push_back(seq);
    }
    return population;
}

// Hàm Nearest Neighbor: Tham lam theo khoảng cách gần nhất
vector<vector<int>> initNearestNeighborPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    vector<vector<double>> dist = buildDistanceMatrix(data);
    
    for (int p = 0; p < populationSize; ++p) {
        vector<int> seq;
        vector<bool> visited(data.numNodes, false);
        visited[data.depotIndex] = true;
        
        // Random node bắt đầu
        vector<int> all_customers = getAllCustomerNodes(data);
        uniform_int_distribution<> start_dist(0, all_customers.size() - 1);
        int current_pos = all_customers[start_dist(gen)];
        seq.push_back(current_pos);
        visited[current_pos] = true;
        
        while (seq.size() < all_customers.size()) {
            // Tìm tất cả khách chưa được phục vụ và tính khoảng cách
            vector<pair<double, int>> candidates; // {distance, nodeId}
            
            for (int i = 0; i < data.numNodes; ++i) {
                if (visited[i] || !data.isCustomer(i)) continue;
                
                double distance = dist[current_pos][i];
                candidates.push_back({distance, i});
            }
            
            if (candidates.empty()) break;
            
            // Sắp xếp theo khoảng cách tăng dần
            sort(candidates.begin(), candidates.end());
            
            // Chọn random trong top min(5, max(10% còn lại, 1))
            int pool_size = min(5, max((int)(candidates.size() * 0.1), 1));
            uniform_int_distribution<> dist_choice(0, pool_size - 1);
            int chosen_idx = dist_choice(gen);
            
            int chosen_node = candidates[chosen_idx].second;
            seq.push_back(chosen_node);
            visited[chosen_node] = true;
            current_pos = chosen_node;
        }
        
        population.push_back(seq);
    }
    return population;
}

// Hàm kết hợp: 10% Random, 30% GreedyTime, 30% Sweep, 30% NN
vector<vector<int>> initStructuredPopulationPDP(int populationSize, const PDPData& data, int runNumber) {
    random_device rd;
    unsigned int seed = rd() + runNumber * 54321;
    mt19937 gen(seed);
    
    int randomCount = (int)(populationSize * 0.10);      // 10%
    int greedyTimeCount = (int)(populationSize * 0.30);  // 30%
    int sweepCount = (int)(populationSize * 0.30);       // 30%
    int nnCount = populationSize - randomCount - greedyTimeCount - sweepCount; // 30%
    
    cout << "PDP Population distribution (NO SEPARATOR):" << endl;
    cout << "   Random: " << randomCount << endl;
    cout << "   Greedy Time: " << greedyTimeCount << endl;
    cout << "   Sweep: " << sweepCount << endl;
    cout << "   Nearest Neighbor: " << nnCount << endl;
    
    vector<vector<int>> population;
    
    auto randomPop = initRandomPDP(randomCount, data);
    population.insert(population.end(), randomPop.begin(), randomPop.end());
    
    auto greedyTimePop = initGreedyTimePDP(greedyTimeCount, data);
    population.insert(population.end(), greedyTimePop.begin(), greedyTimePop.end());
    
    auto sweepPop = initSweepPDP(sweepCount, data);
    population.insert(population.end(), sweepPop.begin(), sweepPop.end());
    
    auto nnPop = initNearestNeighborPDP(nnCount, data);
    population.insert(population.end(), nnPop.begin(), nnPop.end());
    cout << "Generated " << population.size() << " PDP individuals (sequence only, no separators)" << endl;
    return population;
}