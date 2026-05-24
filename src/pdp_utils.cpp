/**
 * @file pdp_utils.cpp
 * @brief Utility functions cho VRP-Drone Resupply project.
 *
 * Thay đổi so với phiên bản cũ:
 *  - validatePDPConstraints: dùng node_id trực tiếp (0-based), bổ sung kiểm tra resupply ordering.
 *  - calculateSolutionCost: dùng route[i] trực tiếp (0-based), bỏ "- 1".
 *  - Thêm printChromosome(): hiển thị vector<Gene> theo định dạng dễ debug.
 *  - printSolution(): gọi printChromosome(solution.original_sequence) ở cuối.
 */

#include "pdp_utils.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <map>
#include <set>

using namespace std;

// ============================================================
// === euclideanDistance ======================================
// ============================================================

double euclideanDistance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2, dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

// ============================================================
// === validatePDPConstraints =================================
// ============================================================

bool validatePDPConstraints(const PDPSolution& solution, const PDPData& data) {
    bool valid = true;

    // ---- Kiểm tra 1: Precedence (P trước DL) trên từng tuyến xe ----
    for (const auto& route : solution.routes) {
        map<int, int> pickupPos, deliveryPos;

        for (int i = 0; i < (int)route.size(); i++) {
            int nodeIdx = route[i];  // 0-based trực tiếp (không cần -1)
            if (nodeIdx < 0 || nodeIdx >= data.numNodes) continue;

            const string& type  = data.nodeTypes[nodeIdx];
            int           pairId = data.pairIds[nodeIdx];

            if (pairId <= 0) continue;
            if (type == "P")  pickupPos[pairId]   = i;
            if (type == "DL") deliveryPos[pairId] = i;
        }

        for (auto& kv : pickupPos) {
            int pairId = kv.first;
            int posP   = kv.second;
            auto it = deliveryPos.find(pairId);
            if (it != deliveryPos.end() && posP >= it->second) {
                cerr << "[VALIDATE] Precedence violation: P(pair=" << pairId
                     << ") pos=" << posP << " >= DL pos=" << it->second << "\n";
                valid = false;
            }
        }
    }

    // ---- Kiểm tra 2: Resupply ordering ----
    // Với mỗi Gene có resupply_vector không rỗng tại vị trí seqPos trong original_sequence:
    //   Gói hàng j phải được giao tại một node nằm sau (hoặc tại) seqPos trong chuỗi.
    if (!solution.original_sequence.empty()) {
        const Chromosome& seq = solution.original_sequence;

        // Xây dựng bản đồ: node_id -> vị trí đầu tiên trong seq (0-based)
        map<int, int> nodeSeqPos;
        for (int i = 0; i < (int)seq.size(); i++) {
            if (seq[i].node_id > 0 && !nodeSeqPos.count(seq[i].node_id))
                nodeSeqPos[seq[i].node_id] = i;
        }

        for (int i = 0; i < (int)seq.size(); i++) {
            const Gene& g = seq[i];
            if (g.node_id <= 0 || g.resupply_vector.empty()) continue;

            for (int pkg : g.resupply_vector) {
                // Vị trí điểm giao của gói pkg trong seq
                auto it = nodeSeqPos.find(pkg);
                if (it == nodeSeqPos.end()) {
                    cerr << "[VALIDATE] Resupply pkg=" << pkg
                         << " trong resupply_vector của node=" << g.node_id
                         << " nhưng pkg không xuất hiện trong seq.\n";
                    valid = false;
                    continue;
                }
                int deliverySeqPos = it->second;
                // node i phải đứng trước hoặc tại điểm giao của pkg
                if (i > deliverySeqPos) {
                    cerr << "[VALIDATE] Resupply ordering violation: "
                         << "node=" << g.node_id << " (seqPos=" << i << ") "
                         << "nhận pkg=" << pkg << " nhưng delivery seqPos=" << deliverySeqPos
                         << " đứng trước node này.\n";
                    valid = false;
                }
            }
        }
    }

    return valid;
}

// ============================================================
// === printChromosome ========================================
// ============================================================

void printChromosome(const vector<Gene>& seq) {
    if (seq.empty()) {
        cout << "  [chromosome rong]\n";
        return;
    }

    cout << "  Chromosome (" << seq.size() << " genes):\n  ";

    for (int i = 0; i < (int)seq.size(); i++) {
        const Gene& g = seq[i];

        // Nhãn node
        if (g.node_id == 0) {
            cout << "[Node: 0 (SEPARATOR)]";
        } else if (g.node_id == -1) {
            cout << "[Node: -1 (DEPOT)]";
        } else {
            cout << "[Node: " << g.node_id;
            // In resupply_vector
            cout << " | Resupply: {";
            for (int k = 0; k < (int)g.resupply_vector.size(); k++) {
                if (k > 0) cout << ", ";
                cout << g.resupply_vector[k];
            }
            cout << "}]";
        }

        if (i + 1 < (int)seq.size()) cout << " -> ";

        // Xuống dòng mỗi 5 gene để dễ đọc
        if ((i + 1) % 5 == 0 && i + 1 < (int)seq.size())
            cout << "\n  ";
    }
    cout << "\n";
}

// ============================================================
// === printSolution ==========================================
// ============================================================

void printSolution(const PDPSolution& solution, const PDPData& data) {
    cout << "\n+=============================================================+\n"
         << "|              SOLUTION DETAILS (Thoi gian thuc te)          |\n"
         << "+=============================================================+\n";

    cout << "\n TONG QUAN:\n"
         << "   Total Cost (C_max): " << fixed << setprecision(2)
         << solution.totalCost << " phut\n"
         << "   Total Penalty:      " << solution.totalPenalty << "\n"
         << "   Feasible:           " << (solution.isFeasible ? "YES" : " NO") << "\n";

    // ---- Chi tiết từng xe tải ----
    if (!solution.truck_details.empty()) {
        cout << "\nCHI TIET CAC XE TAI:\n";
        for (const auto& td : solution.truck_details) {
            cout << "\n   +-- Xe " << td.truck_id
                 << " -----------------------------------------\n"
                 << "   | Hoan thanh: " << fixed << setprecision(2)
                 << td.completion_time << " phut\n"
                 << "   | Route: ";

            for (int i = 0; i < (int)td.route.size(); i++) {
                int nid = td.route[i];
                if (nid >= 0 && nid < data.numNodes)
                    cout << nid << "(" << data.nodeTypes[nid] << ")";
                else
                    cout << nid;
                if (i + 1 < (int)td.route.size()) cout << " -> ";
            }
            cout << "\n   | Timeline:\n";

            for (int i = 0; i < (int)td.route.size(); i++) {
                int nid = td.route[i];
                string ntype = (nid >= 0 && nid < data.numNodes)
                               ? data.nodeTypes[nid] : "?";
                cout << "   |   " << setw(2) << i
                     << ". Node " << setw(2) << nid
                     << " (" << setw(2) << ntype << "): "
                     << "Den=" << setw(6) << fixed << setprecision(1)
                     << td.arrival_times[i] << "'  "
                     << "Roi=" << setw(6) << fixed << setprecision(1)
                     << td.departure_times[i] << "'\n";
            }
            cout << "   +-----------------------------------------------------\n";
        }
    }

    // ---- Chi tiết drone resupply ----
    if (!solution.resupply_events.empty()) {
        cout << "\nCHI TIET DRONE RESUPPLY (WITH CONSOLIDATION):\n";
        for (int i = 0; i < (int)solution.resupply_events.size(); i++) {
            const ResupplyEvent& ev = solution.resupply_events[i];
            cout << "\n   +-- Resupply Trip #" << (i + 1)
                 << " ----------------------------------\n"
                 << "   | Drone: " << ev.drone_id << " | Xe: " << ev.truck_id << "\n"
                 << "   | So khach hang: " << ev.customer_ids.size() << "\n"
                 << "   | Customers: ";
            for (int j = 0; j < (int)ev.customer_ids.size(); j++) {
                if (j > 0) cout << ", ";
                cout << "Node " << ev.customer_ids[j];
            }
            cout << "\n"
                 << "   | Total flight time: " << fixed << setprecision(1)
                 << ev.total_flight_time << " phut\n"
                 << "   | Timeline:\n"
                 << "   |   - Drone roi depot:     " << setw(6) << setprecision(1) << ev.drone_depart_time   << " phut\n"
                 << "   |   [Resupply Point: Node " << ev.resupply_point << "]\n"
                 << "   |     - Drone den:         " << setw(6) << ev.drone_arrive_time   << " phut\n"
                 << "   |     - Xe den:            " << setw(6) << ev.truck_arrive_time   << " phut\n"
                 << "   |     - Bat dau resupply:  " << setw(6) << ev.resupply_start_time << " phut\n"
                 << "   |     - Ket thuc:          " << setw(6) << ev.resupply_end_time   << " phut\n"
                 << "   |     - Truck giao xong:   " << setw(6) << ev.truck_delivery_end  << " phut\n"
                 << "   |   - Drone ve depot:      " << setw(6) << ev.drone_return_time   << " phut\n"
                 << "   +-----------------------------------------------------\n";
        }
    }

    // ---- Thời gian hoàn thành drone ----
    if (!solution.drone_completion_times.empty()) {
        cout << "\n THOI GIAN HOAN THANH CAC DRONE:\n";
        for (int i = 0; i < (int)solution.drone_completion_times.size(); i++) {
            cout << "   Drone " << i << ": " << fixed << setprecision(2)
                 << solution.drone_completion_times[i] << " phut\n";
        }
    }

    // ---- In chromosome gốc ----
    cout << "\n CHROMOSOME GOC (original_sequence):\n";
    printChromosome(solution.original_sequence);

    cout << "\n=============================================================\n";
}

// ============================================================
// === calculateSolutionCost ==================================
// ============================================================

double calculateSolutionCost(const PDPSolution& solution,
                             const vector<vector<double>>& distMatrix) {
    double total = 0.0;
    for (const auto& route : solution.routes) {
        for (int i = 0; i < (int)route.size() - 1; i++) {
            int from = route[i];       // 0-based trực tiếp
            int to   = route[i + 1];   // 0-based trực tiếp
            if (from >= 0 && from < (int)distMatrix.size() &&
                to   >= 0 && to   < (int)distMatrix[from].size())
            {
                total += distMatrix[from][to];
            }
        }
    }
    return total;
}
