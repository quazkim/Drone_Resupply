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
    int depotIndex = 0; // Index 0-based của depot
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

    // --- Distance Matrix (SỬA LỖI LOGIC: Cần 2 ma trận) ---
    vector<vector<double>> truckDistMatrix; // Manhattan (Truck)
    vector<vector<double>> droneDistMatrix; // Euclidean (Drone)


    // --- Helpers (Cập nhật) ---
    int getSeparatorStart() const {
        return numNodes; // Separator bắt đầu từ số nút (total nodes)
    }
    bool isSeparator(int id) const {
        int s = getSeparatorStart();
        return (id >= s && id < s + numTrucks);
    }
    
    // isCustomer: ID là 0-based array index
    bool isCustomer(int id) const {
        if (id < 0 || id >= numNodes) return false; 
        if (id == depotIndex) return false; 
        string t = nodeTypes[id]; 
        return (t == "P" || t == "DL" || (t == "D" && readyTimes[id] > 0));
    }
    
    bool isDepot(int id) const {
        return (id == depotIndex); // depotIndex = 0
    }
    
};

// Cấu trúc lưu thông tin chi tiết về resupply
struct ResupplyEvent {
    int customer_id;          // ID khách hàng được resupply
    int drone_id;             // Drone nào phục vụ
    int truck_id;             // Xe tải nào gặp
    double drone_depart_time; // Drone rời depot lúc nào
    double drone_arrive_time; // Drone đến khách lúc nào
    double truck_arrive_time; // Xe đến khách lúc nào
    double resupply_start;    // Bắt đầu resupply lúc nào
    double resupply_end;      // Kết thúc resupply lúc nào
    double drone_return_time; // Drone về depot lúc nào
};

// Cấu trúc lưu thông tin chi tiết về route của từng xe
struct TruckRouteInfo {
    int truck_id;
    vector<int> route;        // Thứ tự các node (bao gồm depot)
    vector<double> arrival_times;  // Thời gian đến mỗi node
    vector<double> departure_times; // Thời gian rời mỗi node
    double completion_time;   // Thời gian hoàn thành (về depot)
};

// Cấu trúc cho một lời giải đã được giải mã
struct PDPSolution {
    vector<vector<int>> routes; // (Giữ nguyên để tương thích)
    double totalCost = 0.0;     // (Sẽ là C_max)
    double totalPenalty = 0.0;  // (Phạt)
    bool isFeasible = false;    // (Giữ nguyên)
    
    // Thông tin chi tiết (MỚI)
    vector<TruckRouteInfo> truck_details;
    vector<ResupplyEvent> resupply_events;
    vector<double> drone_completion_times; // Thời gian hoàn thành của mỗi drone
};

#endif