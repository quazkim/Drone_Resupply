#ifndef PDP_TYPES_H
#define PDP_TYPES_H

#include <vector>
#include <string>
#include <utility>

using namespace std;

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
    bool useDepotCenter = true;                        // Selection flag: true=center, false=border

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
     * @brief Check if a node ID represents a truck separator in chromosome encoding.
     * @param id Node identifier
     * @return true if id is a separator node for some truck
     */
    bool isSeparator(int id) const {
        int s = getSeparatorStart();
        return (id >= s && id < s + numTrucks);
    }
    
    /**
     * @brief Verify if a node ID corresponds to a valid customer node.
     * Excludes depot and invalid indices. Returns true for P, DL, and D nodes.
     * @param id 0-based node index
     * @return true if the node is a customer (not depot, within bounds, valid type)
     */
    bool isCustomer(int id) const {
        if (id < 0 || id >= numNodes) return false; 
        if (id == depotIndex) return false; 
        string t = nodeTypes[id]; 
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
    
    // Original sequence (for local search to re-decode)
    vector<int> original_sequence;
};

#endif
