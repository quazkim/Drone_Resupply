#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <iomanip>
#include <chrono>

#include "pdp_types.h"
#include "pdp_reader.h"
#include "pdp_init.h"
#include "pdp_fitness.h"  // Thêm file mới
#include "pdp_utils.h" // Để dùng hàm printSolution

using namespace std;

int main(int argc, char* argv[]) {
    cout << "==========================================================" << endl;
    cout << "   PDP SOLVER - INITIALIZATION TEST (PHASE 1 PREVIEW)" << endl;
    cout << "==========================================================" << endl;

    // 1. Đọc tham số đầu vào
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <filename> [pop_size]" << endl;
        return 1;
    }
    string filename = argv[1];
    int popSize = (argc > 2) ? stoi(argv[2]) : 100;

    try {
        // 2. Tải dữ liệu
        PDPData data;
        cout << "\n--- [1] LOADING DATA ---" << endl;
        if (!readPDPFile(filename, data)) {
            return 1;
        }
        
        // Hiển thị thông tin cơ bản để kiểm tra
        cout << "Loaded: " << data.numNodes << " nodes (" 
             << data.numCustomers << " customers)." << endl;
        cout << "Trucks: " << data.numTrucks << ", Drones: " << data.numDrones << endl;

        // 3. Chạy Khởi tạo Quần thể
        cout << "\n--- [2] RUNNING INITIALIZATION ALGORITHMS ---" << endl;
        auto start = chrono::high_resolution_clock::now();
        
        // Gọi hàm khởi tạo hỗn hợp (Random + Sweep + Greedy + NN)
        vector<vector<int>> population = initStructuredPopulationPDP(popSize, data, 1);
        
        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = end - start;
        
        cout << ">> Generated " << population.size() << " individuals in " 
             << fixed << setprecision(3) << elapsed.count() << "s." << endl;

        // 4. Đánh giá quần thể ban đầu
        cout << "\n--- [3] EVALUATING INITIAL POPULATION ---" << endl;
        
        PDPSolution bestSol;
        bestSol.totalCost = numeric_limits<double>::max();
        vector<int> bestSeq;
        
        int feasibleCount = 0;
        double sumCost = 0;

        for (size_t i = 0; i < population.size(); ++i) {
            // Gọi hàm đánh giá (Decoder Giai đoạn 1: 100% Resupply)
            PDPSolution sol = decodeAndEvaluate(population[i], data);
            
            double fitness = sol.totalCost + sol.totalPenalty;
            sumCost += sol.totalCost; // Chỉ tính C_max cho thống kê
            
            if (sol.isFeasible) feasibleCount++;

            // Cập nhật tốt nhất (Ưu tiên Feasible, sau đó đến Cost thấp nhất)
            bool isNewBest = false;
            if (sol.isFeasible) {
                if (!bestSol.isFeasible || sol.totalCost < bestSol.totalCost) {
                    isNewBest = true;
                }
            } else if (!bestSol.isFeasible && fitness < bestSol.totalCost + bestSol.totalPenalty) {
                // Nếu chưa có giải pháp feasible nào, lấy giải pháp ít lỗi nhất
                isNewBest = true;
            }

            if (isNewBest) {
                bestSol = sol;
                bestSeq = population[i];
            }
        }

        // 5. Báo cáo kết quả
        cout << "\n--- [4] RESULTS ---" << endl;
        cout << "Population Size: " << population.size() << endl;
        cout << "Feasible Solutions: " << feasibleCount << " (" 
             << (population.size() > 0 ? (feasibleCount * 100.0 / population.size()) : 0.0) << "%)" << endl;
        cout << "Average C_max: " << (population.size() > 0 ? (sumCost / population.size()) : 0.0) << endl;
        
        cout << "\n🏆 BEST INITIAL SOLUTION FOUND:" << endl;
        
        // In sequence (thứ tự khách hàng)
        cout << "\n📋 SEQUENCE (Thứ tự phục vụ khách hàng):" << endl;
        cout << "   [";
        for (size_t i = 0; i < bestSeq.size(); ++i) {
            cout << bestSeq[i];
            if (i < bestSeq.size() - 1) cout << ", ";
            if ((i + 1) % 20 == 0 && i < bestSeq.size() - 1) cout << "\n    "; // Xuống dòng mỗi 20 phần tử
        }
        cout << "]" << endl;
        cout << "   Total: " << bestSeq.size() << " customers" << endl;
        
        printSolution(bestSol, data);

    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}