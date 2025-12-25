#include "pdp_reader.h"
#include "pdp_utils.h"  
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>

using namespace std;

// === C├üC H├ÇM TIß╗åN ├ìCH KHOß║óNG C├üCH (Manhattan/Euclidean) ===
double manhattanDistance(double x1, double y1, double x2, double y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}
// euclideanDistance ─æ├ú ─æ╞░ß╗úc khai b├ío trong pdp_utils.h

// X├óy dß╗▒ng Cß║ó HAI ma trß║¡n khoß║úng c├ích
void buildAllDistanceMatrices(PDPData& data) {
    int n = data.numNodes;
    data.truckDistMatrix.assign(n, vector<double>(n, 0.0));
    data.droneDistMatrix.assign(n, vector<double>(n, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const auto& c1 = data.coordinates[i];
            const auto& c2 = data.coordinates[j];
            
            // 1. Drone: Euclidean (Bay chim)
            double eu_dist = euclideanDistance(c1.first, c1.second, c2.first, c2.second);
            data.droneDistMatrix[i][j] = eu_dist;
            data.droneDistMatrix[j][i] = eu_dist;
            
            // 2. Truck: Manhattan (─É╞░ß╗¥ng phß╗æ)
            double mh_dist = manhattanDistance(c1.first, c1.second, c2.first, c2.second);
            data.truckDistMatrix[i][j] = mh_dist;
            data.truckDistMatrix[j][i] = mh_dist;
        }
    }
    cout << "Truck (Manhattan) and Drone (Euclidean) distance matrices built." << endl;
}


// === H├ÇM ─Éß╗îC FILE CH├ìNH ===

bool readPDPFile(const string& filename, PDPData& data) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open file: " << filename << endl;
        return false;
    }
    
    string line;
    
    // Bß╗Å qua header
    getline(file, line); 
    cout << "Reading: " << filename << endl;
    
    // Clear data
    data = PDPData(); 
    
    vector<pair<double, double>> tempCoords;
    vector<string> tempTypes;
    vector<int> tempReadyTimes;
    vector<int> tempPairIds;

    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        istringstream iss(line);
        int id;
        double x, y;
        string type;
        int readyTime, pairId; 
        
        if (iss >> id >> x >> y >> type >> readyTime >> pairId) {
            tempCoords.push_back({x, y});
            tempTypes.push_back(type);
            tempReadyTimes.push_back(readyTime);
            tempPairIds.push_back(pairId);
        }
    }
    file.close();
    
    // B╞»ß╗ÜC 1: X├üC ─Éß╗èNH V├Ç CH├êN DEPOT V├ÇO NODE 0 (0-BASED)
    data.coordinates.push_back(data.useDepotCenter ? data.depotCenter : data.depotBorder);
    data.nodeTypes.push_back("D");
    data.readyTimes.push_back(0);
    data.pairIds.push_back(0);
    data.demands.push_back(0); // Depot demand = 0
    
    // B╞»ß╗ÜC 2: CH├êN CUSTOMER V├ÇO SAU DEPOT (ARRAY INDEX 1, 2, 3...)
    for (size_t i = 0; i < tempCoords.size(); ++i) {
        data.coordinates.push_back(tempCoords[i]);
        data.nodeTypes.push_back(tempTypes[i]);
        data.readyTimes.push_back(tempReadyTimes[i]);
        data.pairIds.push_back(tempPairIds[i]);
        
        // SUY LUß║¼N DEMAND (Theo file README: qi = 1)
        int demand = 0;
        if (tempTypes[i] == "P") demand = 1; 
        else if (tempTypes[i] == "DL") demand = -1; 
        else if (tempTypes[i] == "D" && tempReadyTimes[i] > 0) demand = 1; 
        data.demands.push_back(demand);
    }

    data.depotIndex = 0; // Depot l├á node 0 (0-based indexing)
    data.numNodes = data.coordinates.size();
    
    // B╞»ß╗ÜC 3: X├éY Dß╗░NG MA TRß║¼N KHOß║óNG C├üCH
    buildAllDistanceMatrices(data);

    data.numCustomers = 0;
    for(int i=1; i < data.numNodes; ++i) {
        if(data.isCustomer(i)) { 
            data.numCustomers++;
        }
    }

    cout << "Loaded " << data.numNodes << " nodes, " << data.numCustomers << " customers." << endl;
    return true;
}

void showPDPInfo(const PDPData& data) {
    cout << "\nPDP Problem Summary:" << endl;
    cout << "Γö£ΓöÇ Total nodes: " << data.numNodes << endl;
    cout << "Γö£ΓöÇ Customer nodes (P, DL, D): " << data.numCustomers << endl;
    cout << "Γö£ΓöÇ Depot array index: " << data.depotIndex << endl;
    cout << "ΓööΓöÇ Trucks: " << data.numTrucks << " (Capacity: " << data.truckCapacity << ")" << endl;
    
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
    
    map<int, pair<int, int>> pairs; 
    for (int i = 0; i < data.pairIds.size(); ++i) {
        if (data.pairIds[i] > 0) {
            int pairId = data.pairIds[i];
            int nodeId = i; // 0-based index
            if (data.nodeTypes[i] == "P") pairs[pairId].first = nodeId;
            else pairs[pairId].second = nodeId;
        }
    }
    
    cout << "P-D pairs (C2): ";
    for (auto& pair : pairs) {
        if (pair.second.first > 0 && pair.second.second > 0) {
            cout << "[" << pair.second.first << "(P)ΓåÆ" << pair.second.second << "(DL)] ";
        }
    }
    cout << endl;
}
