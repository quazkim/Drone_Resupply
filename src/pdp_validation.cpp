/**
 * @file pdp_validation.cpp
 * @brief Kiểm tra tính hợp lệ của lời giải VRP-Drone Resupply.
 *
 * Cấu trúc 7 block:
 *   [1] Customer Coverage      — Tất cả khách hàng được phục vụ đúng 1 lần
 *   [2] Drone Endurance        — Mỗi chuyến bay drone không vượt quá giới hạn thời gian
 *   [3] Timeline Consistency   — Thời gian đến/rời theo đúng thứ tự thời gian
 *   [4] Drone-Truck Sync       — Drone và xe cùng có mặt tại điểm hẹn
 *   [5] C_max Verification     — Makespan báo cáo khớp với giá trị tính lại
 *   [6] Depot Returns          — Đếm lượt quay về depot, đối chiếu với lệnh -1 trong chromosome
 *   [7] Chromosome Encoding &  — Kiểm tra separator (0), resupply ordering (gói p phải giao
 *       Resupply Logic           sau hoặc tại điểm nhận, cùng xe tải)
 */

#include "pdp_validation.h"
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <cmath>

using namespace std;

// ============================================================
// === validateSolution =======================================
// ============================================================

bool validateSolution(const PDPSolution& solution, const PDPData& data, bool verbose) {
    bool valid = true;

    if (verbose) {
        cout << "\n+========================================================+\n"
             << "|               SOLUTION VALIDATION                     |\n"
             << "+========================================================+\n";
    }

    // ================================================================
    // [1] CUSTOMER COVERAGE
    // ================================================================
    set<int> served;
    for (const auto& truck : solution.truck_details)
        for (int node : truck.route)
            if (data.isCustomer(node)) served.insert(node);

    if (verbose) {
        cout << "\n[1] CUSTOMER COVERAGE:\n"
             << "    Customers served: " << served.size()
             << "/" << data.numCustomers << "\n";
    }
    if ((int)served.size() != data.numCustomers) {
        if (verbose) cout << "    ❌ MISSING CUSTOMERS!\n";
        valid = false;
    } else {
        if (verbose) cout << "    ✓ All customers served\n";
    }

    // ================================================================
    // [2] DRONE ENDURANCE
    // ================================================================
    if (verbose) {
        cout << "\n[2] DRONE ENDURANCE (max "
             << fixed << setprecision(2) << data.droneEndurance << " min):\n";
    }
    for (const auto& ev : solution.resupply_events) {
        if (verbose) {
            cout << "    Trip to Node " << ev.resupply_point
                 << ": flight=" << fixed << setprecision(1)
                 << ev.total_flight_time << " min";
        }
        if (ev.total_flight_time > data.droneEndurance + 1e-6) {
            if (verbose) cout << "  ❌ EXCEEDED!\n";
            valid = false;
        } else {
            if (verbose) cout << "  ✓\n";
        }
    }
    if (solution.resupply_events.empty() && verbose)
        cout << "    (Khong co chuyen bay drone nao)\n";

    // ================================================================
    // [3] TIMELINE CONSISTENCY
    // ================================================================
    if (verbose) cout << "\n[3] TIMELINE CONSISTENCY:\n";
    for (const auto& truck : solution.truck_details) {
        bool ok = true;
        for (int i = 1; i < (int)truck.arrival_times.size(); i++) {
            if (truck.arrival_times[i] < truck.departure_times[i - 1] - 0.01) {
                ok = false; break;
            }
        }
        if (verbose) {
            cout << "    Truck " << truck.truck_id << ": "
                 << (ok ? "✓ Timeline OK" : "❌ Timeline inconsistent!") << "\n";
        }
        if (!ok) valid = false;
    }

    // ================================================================
    // [4] DRONE-TRUCK SYNCHRONIZATION
    // ================================================================
    if (verbose) cout << "\n[4] DRONE-TRUCK SYNCHRONIZATION:\n";
    for (int i = 0; i < (int)solution.resupply_events.size(); i++) {
        const ResupplyEvent& ev = solution.resupply_events[i];
        if (verbose)
            cout << "    Trip #" << (i + 1)
                 << " (Node " << ev.resupply_point << "): ";

        bool sync_ok = true;
        if (ev.drone_arrive_time > ev.resupply_start_time + 0.01) {
            if (verbose) cout << "❌ Drone arrives AFTER resupply start!\n";
            sync_ok = false;
        } else if (ev.truck_arrive_time > ev.resupply_start_time + 0.01) {
            if (verbose) cout << "❌ Truck arrives AFTER resupply start!\n";
            sync_ok = false;
        } else {
            if (verbose) cout << "✓ Sync OK\n";
        }
        if (!sync_ok) valid = false;
    }

    // ================================================================
    // [5] C_MAX VERIFICATION
    // ================================================================
    if (verbose) cout << "\n[5] C_MAX VERIFICATION:\n";
    double calc_cmax = 0.0;
    for (const auto& truck : solution.truck_details)
        calc_cmax = max(calc_cmax, truck.completion_time);
    for (const auto& ev : solution.resupply_events)
        calc_cmax = max(calc_cmax, ev.drone_return_time);

    if (verbose) {
        cout << "    Reported C_max:   " << fixed << setprecision(2)
             << solution.totalCost << " min\n"
             << "    Calculated C_max: " << calc_cmax << " min\n";
    }
    if (abs(calc_cmax - solution.totalCost) > 0.1) {
        if (verbose) cout << "    ❌ MISMATCH!\n";
        valid = false;
    } else {
        if (verbose) cout << "    ✓ Match\n";
    }

    // ================================================================
    // [6] DEPOT RETURNS — đối chiếu với lệnh -1 trong chromosome
    // ================================================================
    if (verbose) {
        cout << "\n[6] DEPOT RETURNS (kiem tra lenh -1 trong chromosome):\n";

        // Đếm lệnh -1 trong original_sequence để đối chiếu
        int depot_cmd_total = 0;
        for (const auto& g : solution.original_sequence)
            if (g.node_id == -1) depot_cmd_total++;

        for (const auto& truck : solution.truck_details) {
            // Đếm số lần depot xuất hiện ở giữa route (không tính đầu/cuối)
            int depot_returns = 0;
            for (int i = 1; i < (int)truck.route.size() - 1; i++)
                if (truck.route[i] == data.depotIndex) depot_returns++;

            cout << "    Truck " << truck.truck_id << ": "
                 << depot_returns << " depot return(s)";
            if (depot_returns > 0)
                cout << " (tuong duong " << depot_returns
                     << " lenh DEPOT AO [-1] trong chromosome)";
            cout << "\n";
        }
        cout << "    Tong lenh [-1] trong chromosome: " << depot_cmd_total << "\n";
        if (depot_cmd_total == 0)
            cout << "    (Khong co lenh quay depot ao nao trong chromosome)\n";
    }

    // ================================================================
    // [7] CHROMOSOME ENCODING & RESUPPLY LOGIC
    // ================================================================
    if (verbose) cout << "\n[7] CHROMOSOME ENCODING & RESUPPLY LOGIC:\n";

    const Chromosome& seq = solution.original_sequence;

    if (seq.empty()) {
        if (verbose) cout << "    (original_sequence rong, bo qua kiem tra chromosome)\n";
    } else {
        // ---- [7.1] Kiểm tra số separator (node_id == 0) ----
        int sep_count = 0;
        for (const auto& g : seq)
            if (g.node_id == 0) sep_count++;

        int expected_seps = data.numTrucks - 1;
        if (verbose) {
            cout << "    [7.1] Separator (node_id=0): tim thay " << sep_count
                 << " / can " << expected_seps << "  ";
        }
        if (sep_count != expected_seps) {
            if (verbose)
                cout << "❌ THIEU/THUA SEPARATOR! (can dung "
                     << expected_seps << " vach ngan cho "
                     << data.numTrucks << " xe tai)\n";
            valid = false;
        } else {
            if (verbose) cout << "✓\n";
        }

        // ---- [7.2] Kiểm tra resupply ordering ----
        // Chiến lược:
        //  - Duyệt seq, theo dõi truck_segment (0 = Truck 1, 1 = Truck 2, ...)
        //  - Với mỗi Gene[i] có resupply_vector, kiểm tra mỗi pkg p:
        //    - p phải xuất hiện ở vị trí k >= i trong seq
        //    - p phải thuộc cùng truck_segment (không vượt qua separator)
        if (verbose) cout << "    [7.2] Resupply ordering:\n";

        // Xây dựng bản đồ: node_id -> (seq_index, truck_segment)
        // Duyệt 1 lần để lấy vị trí xuất hiện đầu tiên của mỗi customer
        map<int, pair<int,int>> customerInfo; // node_id -> {seq_idx, segment}
        {
            int seg = 0;
            for (int i = 0; i < (int)seq.size(); i++) {
                int nid = seq[i].node_id;
                if (nid == 0) { seg++; continue; }   // separator → sang segment tiếp
                if (nid < 0)  continue;               // -1: bỏ qua
                // customer
                if (!customerInfo.count(nid))
                    customerInfo[nid] = {i, seg};
            }
        }

        // Duyệt lần 2: kiểm tra resupply
        int resupply_errors = 0;
        {
            int seg = 0;
            for (int i = 0; i < (int)seq.size(); i++) {
                const Gene& g = seq[i];
                if (g.node_id == 0) { seg++; continue; }
                if (g.node_id <= 0) continue;
                if (g.resupply_vector.empty()) continue;

                for (int pkg : g.resupply_vector) {
                    auto it = customerInfo.find(pkg);

                    // pkg không có trong seq
                    if (it == customerInfo.end()) {
                        if (verbose)
                            cout << "        ❌ RESUPPLY LOGIC ERROR: Goi hang [" << pkg
                                 << "] duoc giao tai node " << g.node_id
                                 << " nhung khong ton tai trong chromosome!\n";
                        valid = false; resupply_errors++;
                        continue;
                    }

                    int pkg_seq_idx = it->second.first;
                    int pkg_seg     = it->second.second;

                    // pkg thuộc xe tải khác
                    if (pkg_seg != seg) {
                        if (verbose)
                            cout << "        ❌ RESUPPLY LOGIC ERROR: Goi hang [" << pkg
                                 << "] (seg=" << pkg_seg
                                 << ") duoc giao khong hop ly cho xe tai (seg=" << seg
                                 << ")! Goi hang phai thuoc cung mot xe.\n";
                        valid = false; resupply_errors++;
                        continue;
                    }

                    // pkg đứng trước vị trí nhận (đã đi qua rồi mới drone giao)
                    if (pkg_seq_idx < i) {
                        if (verbose)
                            cout << "        ❌ RESUPPLY LOGIC ERROR: Goi hang [" << pkg
                                 << "] o vi tri " << pkg_seq_idx
                                 << " < vi tri nhan " << i
                                 << " (node=" << g.node_id
                                 << "). Da di qua diem giao roi moi duoc drone mang den!\n";
                        valid = false; resupply_errors++;
                        continue;
                    }

                    if (verbose)
                        cout << "        ✓ Node " << g.node_id
                             << " nhan goi [" << pkg << "] hop le"
                             << " (giao tai seq[" << pkg_seq_idx << "], seg=" << seg << ")\n";
                }
            }
        }

        if (resupply_errors == 0 && verbose)
            cout << "        ✓ Tat ca resupply_vector hop le\n";
    }

    // ================================================================
    // RESULT
    // ================================================================
    if (verbose) {
        cout << "\n=========================================\n"
             << "VALIDATION RESULT: " << (valid ? "✓ VALID" : "❌ INVALID") << "\n"
             << "=========================================\n";
    }

    return valid;
}

// ============================================================
// === printSolutionSummary ===================================
// ============================================================

void printSolutionSummary(const PDPSolution& solution,
                          double costBeforeLS, double costAfterLS) {
    cout << "\n=========================================\n"
         << "SUMMARY:\n"
         << "  Cost before Local Search: " << fixed << setprecision(2)
         << costBeforeLS << " minutes\n"
         << "  Cost after  Local Search: " << costAfterLS << " minutes\n";

    if (costAfterLS < costBeforeLS) {
        double imp = costBeforeLS - costAfterLS;
        cout << "  Improvement by LS: " << imp << " minutes ("
             << fixed << setprecision(1)
             << (imp / costBeforeLS * 100.0) << "%)\n";
    }
    cout << "  Total Penalty:    " << solution.totalPenalty << "\n"
         << "  Feasible:         " << (solution.isFeasible ? "YES" : "NO") << "\n"
         << "  Resupply Events:  " << solution.resupply_events.size() << "\n"
         << "=========================================\n";
}
