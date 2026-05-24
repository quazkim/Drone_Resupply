#ifndef PDP_TYPES_H
#define PDP_TYPES_H

#include <vector>
#include <string>
#include <utility>
#include <stdexcept>

using namespace std;

// ============================================================
// === CẤU TRÚC MÃ HÓA LỘ TRÌNH MỚI (New Encoding) ===
// ============================================================

/**
 * @brief Một phần tử trong chuỗi mã hóa lộ trình (seq).
 *
 * Quy ước node_id:
 *   node_id > 0  : Điểm khách hàng bình thường
 *   node_id == 0 : Vách ngăn chia xe (Separator) — đứng trước thuộc Truck 1, sau thuộc Truck 2
 *   node_id == -1: Lệnh quay về Depot — xe tải tính thời gian về depot, dỡ C2, nhận C1 mới
 *
 * resupply_vector:
 *   Danh sách ID các gói hàng (customer IDs loại D) mà drone sẽ mang tới
 *   tiếp tế cho xe tải ngay tại node_id này. Rỗng nếu không có drone.
 *   Điểm nào xuất hiện trước trong seq → drone bay trước.
 */
struct SeqStop {
    int node_id;                  // ID điểm dừng (0 = separator, -1 = depot return, >0 = customer)
    vector<int> resupply_vector;  // IDs các gói hàng drone mang đến điểm này

    // Constructors
    SeqStop() : node_id(0) {}                                           // default (node_id=0)
    explicit SeqStop(int nid) : node_id(nid) {}
    SeqStop(int nid, vector<int> rv) : node_id(nid), resupply_vector(std::move(rv)) {}
};

/**
 * @brief Exception thrown khi vi phạm ràng buộc tải trọng nghiêm trọng.
 * Được ném ra trong decode_sequence() khi current_load > M_T.
 */
struct InfeasibleException : public std::runtime_error {
    explicit InfeasibleException(const std::string& msg)
        : std::runtime_error("[INFEASIBLE] " + msg) {}
};

/**
 * @brief Type alias cho SeqStop dùng trong ngữ cảnh chromosome của GA.
 *
 * Hai tên phân biệt ngữ cảnh sử dụng:
 *   - SeqStop    : API giải mã lộ trình  (decode_sequence, print_decoded_routes)
 *   - Gene       : API quần thể GA       (pdp_init, pdp_ga, crossover, mutation)
 *
 * Quy ước gene đặc biệt (node_id):
 *   node_id  > 0  →  điểm khách hàng bình thường
 *   node_id == 0  →  vách ngăn chia Truck 1 / Truck 2
 *   node_id == -1 →  lệnh quay về Depot
 */
using Gene = SeqStop;

/// Chromosome: một cá thể trong quần thể GA
using Chromosome = std::vector<Gene>;

/**
 * @brief Core data structure encapsulating the Pickup-Delivery Problem (PDP) instance.
 * 
 * This structure stores:
 * - Network topology: nodes, customers, depot location
 * - Vehicle fleet: trucks and drones with their specifications
 * - Distance matrices: Manhattan (trucks) and Euclidean (drones)
 * - Physical constraints: capacities, speeds, endurance limits
 */
struct PDPData {
    // === INSTANCE TOPOLOGY ===
    int numNodes = 0;                           // Total nodes in the network (depot + customers)
    int numCustomers = 0;                       // Number of pickup-delivery customer pairs (P, DL, D nodes)
    int numTrucks = 2;                          // Number of available trucks for ground delivery
    int truckCapacity = 50;                     // Load capacity per truck (kg/units)
    double truckSpeed = 30.0;                   // Truck cruising speed (km/h)
    double truckServiceTime = 3.0;              // Time to process each pickup/delivery location (minutes)
    double depotReceiveTime = 5.0;              // Time for truck to receive resupply at depot (minutes)
    int depotIndex = 0;                         // Central depot index (0-based, typically at origin)
    
    // === NODE ATTRIBUTES ===
    vector<pair<double,double>> coordinates;    // (X, Y) geographic coordinates for each node
    vector<string> nodeTypes;                   // Node classification: "DEPOT", "P" (pickup), "DL" (delivery), "D" (optional)
    vector<int> readyTimes;                     // Ready time constraints for time-window feasibility
    vector<int> pairIds;                        // Pairing information for pickup-delivery constraints

    // === DEPOT CONFIGURATION ===
    pair<double, double> depotCenter = {10.0, 10.0};  // Center depot coordinates (primary resupply point)
    pair<double, double> depotBorder = {0.0, 10.0};   // Border depot coordinates (alternative resupply point)
    pair<double, double> depotOutside = {-10.0, 10.0};  // Outside depot coordinates (peripheral resupply point)
    int depotMode = 0;                                 // Depot selection: 0=center, 1=border, 2=outside

    // === CUSTOMER DEMAND ===
    vector<int> demands;                        // Demand/payload weight for each customer (kg/units)
    
    // === DRONE PARAMETERS ===
    int numDrones = 2;                      // Number of aerial drones available for resupply missions
    int droneCapacity = 2;                  // Payload capacity per drone flight (orders/packages)
    double droneSpeed = 60.0;               // Drone cruising speed (km/h)
    double droneEndurance = 90.0;           // Maximum flight duration (minutes) for range constraint
    double resupplyTime = 5.0;              // Time to transfer goods from drone to truck (minutes/location)
    double depotDroneLoadTime = 5.0;        // Time for drone to load payload at depot (minutes)

    // === DISTANCE MATRICES ===
    vector<vector<double>> truckDistMatrix; // Manhattan distance matrix (truck routing, urban/grid model)
    vector<vector<double>> droneDistMatrix; // Euclidean distance matrix (drone routing, straight-line model)

    // === UTILITY METHODS ===
    
    /**
     * @brief Get the starting index for truck separator nodes in the chromosome encoding.
     * Separators mark points where truck routes begin.
     */
    int getSeparatorStart() const {
        return numNodes;
    }
    
    /**
     * @brief Check if a Gene node_id represents the truck separator.
     * Quy ước mã hóa mới: Gene(0) = vách ngăn chia Truck 1 / Truck 2.
     * @param id node_id của Gene
     * @return true nếu id == 0
     */
    bool isSeparator(int id) const {
        return (id == 0);
    }

    /**
     * @brief Check if a Gene node_id represents a virtual depot-return command.
     * Quy ước mã hóa mới: Gene(-1) = lệnh xe quay về depot ảo giữa hành trình.
     * @param id node_id của Gene
     * @return true nếu id == -1
     */
    bool isDepotReturn(int id) const {
        return (id == -1);
    }
    
    /**
     * @brief Verify if a node ID corresponds to a valid customer node (0-based).
     * Trả về false nếu id <= 0 (depot vật lý, separator, depot-return)
     * hoặc id >= numNodes (ngoài vùng hợp lệ).
     * Trả về true cho các node khách hàng: P, DL, D (C1-resupply có readyTime > 0).
     * @param id 0-based node index
     * @return true if and only if id > 0, id < numNodes, and node type is customer
     */
    bool isCustomer(int id) const {
        if (id <= 0 || id >= numNodes) return false;  // loại 0 (depot), -1 (depot-return), ngoài vùng
        const string& t = nodeTypes[id];
        return (t == "P" || t == "DL" || (t == "D" && readyTimes[id] > 0));
    }
    
    /**
     * @brief Check if a node represents the central depot.
     * @param id Node identifier
     * @return true if id matches the depot index
     */
    bool isDepot(int id) const {
        return (id == depotIndex);
    }
    
    /**
     * @brief Determine drone capacity based on instance size (paper specification).
     * Small instances (≤20 customers) use capacity 2; larger instances use capacity 10.
     * @return Drone capacity for current instance
     */
    int getDroneCapacity() const {
        if (numCustomers <= 20) return 2;   // Small instances
        return 10;                          // Large instances
    }
};

// Structure storing detailed information about resupply events
// ONE RENDEZVOUS MODEL: Drone bay đến 1 điểm hẹn duy nhất, giao TẤT CẢ packages cho truck
// Sau đó truck tự đi giao cho các customers
struct ResupplyEvent {
    vector<int> customer_ids;      // List of customer IDs whose packages are delivered in this trip
    int resupply_point;            // The SINGLE rendezvous point (first customer in list)
    int drone_id;                  // ID of the drone performing resupply
    int truck_id;                  // ID of the truck meeting the drone
    double drone_depart_time;      // Time when drone departs from depot
    double drone_arrive_time;      // Time when drone arrives at resupply point
    double truck_arrive_time;      // Time when truck arrives at resupply point
    double resupply_start_time;    // Time when resupply starts (max of drone & truck arrival)
    double resupply_end_time;      // Time when resupply ends (after handover)
    double drone_return_time;      // Time when drone returns to depot
    double total_flight_time;      // Total flight duration (for endurance constraint check)
    double truck_delivery_end;     // Time when truck finishes delivering all packages to customers
};

// Structure storing detailed information about each truck's route
struct TruckRouteInfo {
    int truck_id;
    vector<int> route;        // Sequence of visited nodes (including depot)
    vector<double> arrival_times;  // Arrival time at each node
    vector<double> departure_times; // Departure time from each node
    double completion_time;   // Route completion time (return to depot)
};

// Structure representing a complete PDP solution
struct PDPSolution {
    vector<vector<int>> routes; // Vehicle routes (general principle for consistency)
    double totalCost = 0.0;     // Total cost (C_max)
    double totalPenalty = 0.0;  // Penalty for constraint violations
    bool isFeasible = false;    // Solution feasibility flag
    
    // Detailed solution information
    vector<TruckRouteInfo> truck_details;
    vector<ResupplyEvent> resupply_events;
    vector<double> drone_completion_times; // Completion time for each drone
    
    // Original chromosome (for local search to re-decode and for debug printing)
    Chromosome original_sequence;  // vector<Gene> — encoding tạo ra lời giải này

    // Thứ tự bay của drone (danh sách resupply events theo đúng thứ tự thực hiện)
    // Được điền bởi decode_sequence(), mỗi phần tử = {drone_id, customer_ids, resupply_node}
    // (Khác với resupply_events vốn không đảm bảo thứ tự)
    vector<int> drone_order;  // Danh sách resupply_event indices theo thứ tự bay thực tế
};

#endif
