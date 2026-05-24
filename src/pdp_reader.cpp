/**
 * @file pdp_reader.cpp
 * @brief Parse PDP instance files and build distance matrices.
 *
 * NODE ID CONVENTIONS:
 *   Physical array (0-based): index 0 = depot, 1..N-1 = customers
 *   Chromosome Gene encoding:
 *     node_id  > 0  →  Customer (maps 1:1 to physical array index)
 *     node_id == 0  →  Separator (vach ngan Truck 1 / Truck 2)
 *     node_id == -1 →  Depot Return (lenh quay ve depot ao)
 */

#include "pdp_reader.h"
#include "pdp_utils.h"   // euclideanDistance
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <cmath>

using namespace std;

// ============================================================
// === DISTANCE HELPERS =======================================
// ============================================================

double manhattanDistance(double x1, double y1, double x2, double y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void buildAllDistanceMatrices(PDPData& data) {
    int n = data.numNodes;
    // Allocate n×n matrices (row/col 0 = physical depot, 1..n-1 = customers)
    data.truckDistMatrix.assign(n, vector<double>(n, 0.0));
    data.droneDistMatrix.assign(n, vector<double>(n, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const auto& c1 = data.coordinates[i];
            const auto& c2 = data.coordinates[j];

            // Drone: Euclidean → minutes (rounded)
            double eu   = euclideanDistance(c1.first, c1.second, c2.first, c2.second);
            double tDrn = round((eu / data.droneSpeed) * 60.0);
            data.droneDistMatrix[i][j] = data.droneDistMatrix[j][i] = tDrn;

            // Truck: Manhattan → minutes (rounded)
            double mh   = manhattanDistance(c1.first, c1.second, c2.first, c2.second);
            double tTrk = round((mh / data.truckSpeed) * 60.0);
            data.truckDistMatrix[i][j] = data.truckDistMatrix[j][i] = tTrk;
        }
    }
    cout << "[READER] Distance matrices built: "
         << n << "x" << n
         << " | Truck=Manhattan->min | Drone=Euclidean->min (rounded)\n";
}

// ============================================================
// === readPDPFile ============================================
// ============================================================

bool readPDPFile(const string& filename, PDPData& data) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "[READER] ERROR: Cannot open file: " << filename << "\n";
        return false;
    }
    cout << "[READER] Reading: " << filename << "\n";

    // Preserve runtime settings set before calling readPDPFile
    int savedDepotMode = data.depotMode;
    data = PDPData();
    data.depotMode = savedDepotMode;

    // ---- Raw parse: read all customer rows into temp buffers ----
    string line;
    getline(file, line);  // skip header line

    vector<pair<double,double>> tempCoords;
    vector<string>              tempTypes;
    vector<int>                 tempReadyTimes;
    vector<int>                 tempPairIds;

    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        istringstream iss(line);
        int    id, readyTime, pairId;
        double x, y;
        string type;

        // Format: <id> <x> <y> <type> <readyTime> <pairId>
        if (iss >> id >> x >> y >> type >> readyTime >> pairId) {
            tempCoords.push_back({x, y});
            tempTypes.push_back(type);
            tempReadyTimes.push_back(readyTime);
            tempPairIds.push_back(pairId);
            // Note: the `id` field from the file is the 1-based customer ID in the
            // raw file, but we do NOT use it for indexing — we use tempCoords order.
        }
    }
    file.close();

    // ================================================================
    // STEP 1: Insert DEPOT at physical array index 0
    //         depotIndex is always 0 (0-based, never changes)
    // ================================================================
    pair<double,double> selectedDepot;
    switch (data.depotMode) {
        case 1:  selectedDepot = data.depotBorder;  break;
        case 2:  selectedDepot = data.depotOutside; break;
        default: selectedDepot = data.depotCenter;  break;  // mode 0 = center
    }

    data.coordinates.push_back(selectedDepot);  // index 0 = depot
    data.nodeTypes.push_back("D");
    data.readyTimes.push_back(0);
    data.pairIds.push_back(0);
    data.demands.push_back(0);   // depot demand = 0

    data.depotIndex = 0;  // Always 0-based, always index 0

    // ================================================================
    // STEP 2: Append customer nodes at indices 1 .. N-1
    //         These are the only nodes that can appear as Gene(node_id > 0)
    // ================================================================
    for (size_t i = 0; i < tempCoords.size(); ++i) {
        data.coordinates.push_back(tempCoords[i]);
        data.nodeTypes.push_back(tempTypes[i]);
        data.readyTimes.push_back(tempReadyTimes[i]);
        data.pairIds.push_back(tempPairIds[i]);

        // Infer demand from node type (qi = 1 as per problem spec)
        int demand = 0;
        if      (tempTypes[i] == "P")                                    demand =  1;
        else if (tempTypes[i] == "DL")                                   demand = -1;
        else if (tempTypes[i] == "D" && tempReadyTimes[i] > 0)           demand =  1;
        data.demands.push_back(demand);
    }

    data.numNodes = (int)data.coordinates.size();

    // ================================================================
    // STEP 3: Build distance matrices (0-based, depot at [0][*])
    // ================================================================
    buildAllDistanceMatrices(data);

    // ================================================================
    // STEP 4: Count customers (only indices > 0, isCustomer() returns
    //         false for id <= 0 with the updated rule in pdp_types.h)
    // ================================================================
    data.numCustomers = 0;
    for (int i = 1; i < data.numNodes; ++i)
        if (data.isCustomer(i)) data.numCustomers++;

    cout << "[READER] Loaded " << data.numNodes << " nodes total | "
         << "Depot: index 0 | Customers: " << data.numCustomers << "\n";
    return true;
}

// ============================================================
// === showPDPInfo ============================================
// ============================================================

void showPDPInfo(const PDPData& data) {
    cout << "\n+-------------------------------------------------------+\n"
         << "|              PDP Problem Summary                      |\n"
         << "+-------------------------------------------------------+\n";

    cout << "  Total nodes      : " << data.numNodes      << "\n"
         << "  Customer nodes   : " << data.numCustomers  << "  (P, DL, D with readyTime>0)\n"
         << "  Depot array idx  : " << data.depotIndex    << "  (always 0-based)\n"
         << "  Trucks           : " << data.numTrucks
         << "  (capacity=" << data.truckCapacity << ")\n"
         << "  Drones           : " << data.numDrones
         << "  (capacity=" << data.getDroneCapacity()
         << ", endurance=" << data.droneEndurance << " min)\n"
         << "  Truck speed      : " << data.truckSpeed    << " km/h\n"
         << "  Drone speed      : " << data.droneSpeed    << " km/h\n";

    // Node type breakdown
    int nDepot=0, nP=0, nDL=0, nD_resupply=0;
    for (int i = 0; i < data.numNodes; ++i) {
        if      (data.nodeTypes[i] == "P")  nP++;
        else if (data.nodeTypes[i] == "DL") nDL++;
        else if (data.nodeTypes[i] == "D") {
            if (data.readyTimes[i] > 0) nD_resupply++;
            else                        nDepot++;
        }
    }
    cout << "\n  Node type breakdown:\n"
         << "    Depot(s)        : " << nDepot      << "\n"
         << "    P  (C2 pickup)  : " << nP          << "\n"
         << "    DL (C2 delivery): " << nDL         << "\n"
         << "    D  (C1 resupply): " << nD_resupply << "\n";

    // P-DL pairs
    map<int, pair<int,int>> pairs;  // pairId -> {P_index, DL_index}
    for (int i = 0; i < (int)data.pairIds.size(); ++i) {
        if (data.pairIds[i] <= 0) continue;
        int pid = data.pairIds[i];
        if      (data.nodeTypes[i] == "P")  pairs[pid].first  = i;
        else if (data.nodeTypes[i] == "DL") pairs[pid].second = i;
    }
    if (!pairs.empty()) {
        cout << "\n  P-DL pairs (C2): ";
        for (auto& kv : pairs) {
            if (kv.second.first > 0 && kv.second.second > 0)
                cout << "[" << kv.second.first << "(P)->"
                     << kv.second.second << "(DL)] ";
        }
        cout << "\n";
    }

    // ---- Encoding mapping block ----
    cout << "\n+-------------------------------------------------------+\n"
         << "|  [ENCODING MAPPING]                                   |\n"
         << "|  Gene( 0)  ->  Separator   (vach ngan Truck 1/2)     |\n"
         << "|  Gene(-1)  ->  Depot Return (lenh quay ve depot ao)   |\n"
         << "|  Gene(>0)  ->  Customer     (index 1.." << setw(3) << (data.numNodes-1)
         << " in array)     |\n"
         << "+-------------------------------------------------------+\n\n";
}
