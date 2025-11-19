#ifndef PDP_TYPES_H
#define PDP_TYPES_H

#include <vector>
#include <string>
#include <utility>

using namespace std;

struct PDPData {
    // --- VRP Instance (Giữ nguyên của bạn) ---
    int numNodes = 0;                    
    int numCustomers = 0; // Sẽ được cập nhật để đếm cả P, DL, D
    int numTrucks = 2;                   
    int truckCapacity = 50; // Giữ nguyên 50 từ file của bạn             
    double truckSpeed = 30.0;            
    double truckServiceTime = 3.0; // (δ) Thời gian phục vụ P/DL (phút)       
    double depotReceiveTime = 5.0; // (δt) Thời gian xe tải ở depot (phút)       
    int depotIndex = 1;                  
    vector<pair<double,double>> coordinates; 
    vector<string> nodeTypes;               
    vector<int> readyTimes;                 
    vector<int> pairIds;                    

    // --- DEPOT CONFIGURATION (FROM README) ---
    pair<double, double> depotCenter = {10.0, 10.0};  // Center depot
    pair<double, double> depotBorder = {0.0, 10.0};   // Border depot
    bool useDepotCenter = true; // true = center (10,10), false = border (0,10)

    // --- CÁC TRƯỜNG MỚI (BẮT BUỘC) ---
    vector<int> demands;                    // (Cần cho hàm fitness)
    
    // --- Drone Params (BẮT BUỘC) ---
    int numDrones = 2;                   // (d) Số lượng drone (giả định vô hạn trong Giai đoạn 1)
    double droneSpeed = 60.0;            // km/h
    double droneEndurance = 90.0;        // (L_d) Thời gian bay tối đa (phút)
    double resupplyTime = 5.0;           // (Δ) Thời gian resupply (phút)
    double depotDroneLoadTime = 5.0;     // (δd) Thời gian drone lấy hàng ở depot (phút)

    // --- Distance Matrix (BẮT BUỘC) ---
    vector<vector<double>> distMatrix; // Ma trận khoảng cách chung cho cả truck và drone (Euclidean)


    // --- Helpers (Cập nhật) ---
    int getSeparatorStart() const {
        return numCustomers + 1; // Separator bắt đầu từ số khách + 1
    }
    bool isSeparator(int id) const {
        int s = getSeparatorStart();
        return (id >= s && id < s + numTrucks);
    }
    
    // SỬA LỖI QUAN TRỌNG: isCustomer phải bao gồm cả 3 loại NHƯNG LOẠI TRỪ DEPOT
    bool isCustomer(int id) const {
        if (id <= 0 || id >= numNodes) return false; // 0-based: depot=0, customers=1,2,3,...
        if (id == depotIndex) return false; // Depot không phải customer
        string t = nodeTypes[id]; // id là array index (0-based)
        // Chỉ tính là "customer" nếu là P, DL, hoặc D (có ready time > 0)
        return (t == "P" || t == "DL" || (t == "D" && readyTimes[id] > 0));
    }
    
    // Kiểm tra có phải depot không
    bool isDepot(int id) const {
        return (id == depotIndex); // depotIndex = 0
    }
    
    // Lấy node ID dạng 1-based để hiển thị
    int getDisplayNodeId(int arrayIndex) const {
        return arrayIndex + 1; // Array index 0→1, 1→2, etc.
    }
};

// Cấu trúc cho một lời giải đã được giải mã
struct PDPSolution {
    vector<vector<int>> routes; // (Giữ nguyên)
    double totalCost = 0.0;     // (Sẽ là C_max)
    double totalPenalty = 0.0;  // (Phạt)
    bool isFeasible = false;    // (Giữ nguyên)
};

#endif