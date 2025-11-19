#include "pdp_reader.h"
#include "pdp_utils.h"  // Include for euclideanDistance
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>

using namespace std;

// === HÀM XÂY DỰNG MA TRẬN KHOẢNG CÁCH ===

// Xây dựng ma trận khoảng cách chung
void buildDistanceMatrix(PDPData& data) {
    int n = data.numNodes;
    data.distMatrix.assign(n, vector<double>(n, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const auto& c1 = data.coordinates[i];
            const auto& c2 = data.coordinates[j];
            
            // Cả truck và drone đều dùng Euclidean distance
            double eu_dist = euclideanDistance(c1.first, c1.second, c2.first, c2.second);
            
            data.distMatrix[i][j] = eu_dist;
            data.distMatrix[j][i] = eu_dist;
        }
    }
    cout << "Distance matrix built (Euclidean for both truck and drone)." << endl;
}


// === HÀM ĐỌC FILE CHÍNH ===

bool readPDPFile(const string& filename, PDPData& data) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "❌ Cannot open file: " << filename << endl;
        return false;
    }
    
    string line;
    
    // Bỏ qua header
    getline(file, line); 
    cout << "📖 Reading: " << filename << endl;
    
    // Clear data
    data = PDPData(); // Khởi tạo với giá trị mặc định từ pdp_types.h
    data.coordinates.clear();
    data.nodeTypes.clear();
    data.readyTimes.clear();
    data.pairIds.clear();
    data.demands.clear(); 
    
    int firstDepotId = -1;

    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        istringstream iss(line);
        int id;
        double x, y;
        string type;
        int readyTime, pairId, demand; 
        
        // Giả định file data chỉ có 6 cột, demand tự suy
        if (iss >> id >> x >> y >> type >> readyTime >> pairId) {
            // TÔI ĐANG GIẢ ĐỊNH DEMAND, BẠN CẦN CHỈNH LẠI
            // (Theo file README: Order_weight: qi = 1, ∀i ∈ N)
            if (type == "P") demand = 1; 
            else if (type == "DL") demand = -1; // (Phải là số âm)
            else if (type == "D" && readyTime > 0) demand = 1; 
            else demand = 0; // Depot
            
            data.coordinates.push_back({x, y});
            data.nodeTypes.push_back(type);
            data.readyTimes.push_back(readyTime);
            data.pairIds.push_back(pairId);
            data.demands.push_back(demand); // Lưu demand
            
            // Tìm depot
            if (type == "D" && readyTime == 0 && firstDepotId == -1) {
                firstDepotId = id;
            }
        }
    }
    
    // SETUP DEPOT WITH FIXED COORDINATES FROM README
    // Depot sẽ là node 0, customers từ file sẽ là nodes 1,2,3,...
    data.coordinates.insert(data.coordinates.begin(), 
        data.useDepotCenter ? data.depotCenter : data.depotBorder);
    data.nodeTypes.insert(data.nodeTypes.begin(), "D");
    data.readyTimes.insert(data.readyTimes.begin(), 0);
    data.pairIds.insert(data.pairIds.begin(), 0);
    data.demands.insert(data.demands.begin(), 0);
    
    data.depotIndex = 0; // Depot là node 0 (0-based indexing)
    data.numNodes = data.coordinates.size();
    
    // Cập nhật lại pair_id cho các customer nodes (vì đã shift +1)
    for (int i = 1; i < data.numNodes; i++) {
        if (data.pairIds[i] > 0) {
            data.pairIds[i] += 1; // Shift pair_id vì depot đã được insert
        }
    }
    file.close();

    // Đếm lại số khách hàng (P, DL, D)
    data.numCustomers = 0;
    for(int i=1; i <= data.numNodes; ++i) {
        if(data.isCustomer(i)) { // (Sử dụng hàm isCustomer đã sửa)
            data.numCustomers++;
        }
    }
    
    // Sau khi đọc xong, xây dựng ma trận
    buildDistanceMatrix(data);

    cout << "Loaded " << data.numNodes << " nodes, " << data.numCustomers << " customers." << endl;
    return true;
}

void showPDPInfo(const PDPData& data) {
    cout << "\nPDP Problem Summary:" << endl;
    cout << "├─ Total nodes: " << data.numNodes << endl;
    cout << "├─ Customer nodes (P, DL, D): " << data.numCustomers << endl;
    cout << "├─ Depot index: " << data.depotIndex << endl;
    cout << "└─ Trucks: " << data.numTrucks << " (Capacity: " << data.truckCapacity << ")" << endl;
    
    int depots = 0, pickups = 0, deliveries = 0, c1_deliveries = 0;
    for (int i = 0; i < data.nodeTypes.size(); ++i) {
        if (data.nodeTypes[i] == "P") pickups++;
        else if (data.nodeTypes[i] == "DL") deliveries++;
        else if (data.nodeTypes[i] == "D") {
            if (data.readyTimes[i] > 0) c1_deliveries++;
            else depots++;
        }
    }
    
    cout << "Node types: " << depots << " Depot(s), " << c1_deliveries << " D (C1-Resupply), " 
         << pickups << " P (C2), " << deliveries << " DL (C2)" << endl;
    
    map<int, pair<int, int>> pairs; // pairId -> {P_id, DL_id}
    for (int i = 0; i < data.pairIds.size(); ++i) {
        if (data.pairIds[i] > 0) {
            int pairId = data.pairIds[i];
            int nodeId = i + 1;
            if (data.nodeTypes[i] == "P") pairs[pairId].first = nodeId;
            else pairs[pairId].second = nodeId;
        }
    }
    
    cout << "P-D pairs (C2): ";
    for (auto& pair : pairs) {
        cout << "[" << pair.second.first << "(P)→" << pair.second.second << "(DL)] ";
    }
    cout << endl;
}