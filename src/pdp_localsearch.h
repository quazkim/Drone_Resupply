#ifndef PDP_LOCALSEARCH_H
#define PDP_LOCALSEARCH_H

#include "pdp_types.h"
#include <vector>
#include <random>
#include <string>
#include <map>

/**
 * Full Integrated Local Search with Adaptive Operator Selection
 * Toi uu dong thoi truck routes va drone trips truc tiep tren solution
 * (khong di qua decode)
 * 
 * TRUCK OPERATORS:
 * - 2-opt: Dao nguoc mot doan trong route
 * - Or-opt: Di chuyen mot chuoi 1-3 nodes den vi tri khac
 * - Swap: Hoan doi 2 nodes trong route
 * - Relocate: Di chuyen 1 node sang vi tri khac
 * - Cross-exchange: Hoan doi segments giua 2 trucks
 * 
 * DRONE OPERATORS:
 * - Merge: Gop 2 drone trips thanh 1
 * - Split: Tach 1 drone trip lon thanh nhieu trips nho
 * - Move: Di chuyen 1 customer giua drone trips
 * - Swap: Hoan doi customers giua 2 drone trips
 * - Re-assign: Doi drone phuc vu mot trip
 * - Reorder: Thay doi thu tu customers trong 1 trip
 * - InsertIntoTrip: Them customer vao trip dang co (NEW)
 * 
 * ADAPTIVE SELECTION:
 * - Theo doi success rate cua tung operator
 * - Uu tien operators hieu qua hon
 * - Dieu chinh trong so dong theo thoi gian
 */

// Enum cho c├íc operator types
enum class OperatorType {
    // Truck operators
    TRUCK_2OPT,
    TRUCK_SWAP,
    TRUCK_RELOCATE,
    TRUCK_CROSS_EXCHANGE,
    // Drone operators
    DRONE_REORDER,
    DRONE_SWAP,
    DRONE_MOVE,
    DRONE_MERGE,
    DRONE_CONSOLIDATE,  // NEW: Merge trips serving same truck
    DRONE_SPLIT,
    DRONE_REASSIGN,
    DRONE_INSERT_INTO_TRIP,
    // Total count
    NUM_OPERATORS
};

// Structure de track operator performance
struct OperatorStats {
    int attempts = 0;
    int successes = 0;
    double total_improvement = 0.0;
    double weight = 1.0;  // Selection weight (adaptive)
    
    double getSuccessRate() const {
        return (attempts > 0) ? (double)successes / attempts : 0.0;
    }
    
    double getAvgImprovement() const {
        return (successes > 0) ? total_improvement / successes : 0.0;
    }
};

class IntegratedLocalSearch {
public:
    IntegratedLocalSearch(const PDPData& data, int maxIterations = 500);
    
    // Main entry point
    PDPSolution run(PDPSolution initialSolution);
    
    // NEW: Local search focusing on longest route only
    PDPSolution runLongestRoute(PDPSolution initialSolution);
    
    // Get operator statistics (for analysis)
    void printOperatorStats() const;
    
    // ============ DRONE-LEVEL OPTIMIZATION (post-decode) ============
    // These are PUBLIC so they can be called from main
    
    // Reassign drone: Thu tat ca drone cho moi trip, chon tot nhat
    bool droneReassignOptimal(PDPSolution& sol);
    
    // Swap drone assignments: Hoan doi drone assignments giua 2 trips
    bool droneSwapAssignments(PDPSolution& sol);
    
    // Recalculate drone trip time and return C_max
    double recalcDroneAndGetCmax(PDPSolution& sol);
    
    // ============ NEW: LONGEST ROUTE OPTIMIZATION ============
    
    // Find the longest truck route (by distance/time)
    int findLongestRoute(const PDPSolution& sol) const;
    
    // Optimize only the longest route (single-route operators only)
    bool optimizeLongestRoute(PDPSolution& sol);
    
    // Perturbation operators for escaping local optima (drone-focused)
    bool applyPerturbationMove(PDPSolution& sol);
    
private:
    const PDPData& data;
    int maxIterations;
    std::mt19937 rng;
    
    // Statistics
    int truck_improvements = 0;
    int drone_improvements = 0;
    
    // Simulated Annealing parameters
    double sa_temperature = 0.0;
    double sa_cooling_rate = 0.995;
    double sa_min_temperature = 0.01;
    bool acceptBySA(double delta);  // Accept worse move with probability e^(-delta/T)
    void initSATemperature(double initial_cmax);  // Set initial temperature based on objective
    
    // Adaptive operator selection
    std::map<OperatorType, OperatorStats> operatorStats;
    void initOperatorStats();
    void updateOperatorStats(OperatorType op, bool success, double improvement);
    void updateOperatorWeights();
    OperatorType selectOperator(const std::vector<OperatorType>& operators);
    std::string getOperatorName(OperatorType op) const;
    
    // ============ HELPER FUNCTIONS ============
    
    // Tinh travel time cho truck (Manhattan distance)
    double getTruckTravelTime(int from, int to) const;
    
    // Tinh travel time cho drone (Euclidean distance) 
    double getDroneTravelTime(int from, int to) const;
    
    // Recalculate truck completion times after route change
    void recalculateTruckTimes(PDPSolution& sol);
    
    // Recalculate drone trip times considering truck rendezvous
    void recalculateDroneTimes(PDPSolution& sol);
    
    // Tinh lai C_max tu solution
    double calculateCmax(const PDPSolution& sol) const;
    
    // Kiem tra feasibility cua truck route (capacity, precedence)
    bool isTruckRouteFeasible(const std::vector<int>& route, int truck_id) const;
    
    // Kiem tra feasibility cua drone trip (endurance, capacity, constraints)
    bool isDroneTripFeasible(const ResupplyEvent& trip, const PDPSolution& sol) const;
    
    // Kiem tra precedence P-DL constraints
    bool checkPrecedenceConstraints(const PDPSolution& sol) const;
    
    // Kiem tra node type co phai type D (drone eligible)
    bool isDroneEligible(int node_id) const;
    
    // ============ TRUCK LOCAL SEARCH OPERATORS ============
    
    // 2-opt: Dao nguoc mot doan trong route
    bool truck2Opt(PDPSolution& sol);
    
    // Or-opt: Di chuyen 1-3 nodes lien tiep den vi tri khac
    bool truckOrOpt(PDPSolution& sol);
    
    // Swap: Hoan doi 2 nodes trong cung route
    bool truckSwap(PDPSolution& sol);
    
    // Relocate: Di chuyen 1 node den vi tri khac trong route
    bool truckRelocate(PDPSolution& sol);
    
    // Cross-exchange: Hoan doi segments giua 2 trucks
    bool truckCrossExchange(PDPSolution& sol);
    
    // Single-route operators (for longest route optimization)
    bool truck2OptSingleRoute(PDPSolution& sol, int truck_idx);
    bool truckSwapSingleRoute(PDPSolution& sol, int truck_idx);
    bool truckRelocateSingleRoute(PDPSolution& sol, int truck_idx);
    
    // ============ DRONE LOCAL SEARCH OPERATORS ============
    
    // Merge: Gop 2 drone trips thanh 1 (neu feasible)
    bool droneMergeTrips(PDPSolution& sol);
    
    // Split: Tach 1 drone trip lon thanh nhieu trips nho (giam waiting time)
    bool droneSplitTrip(PDPSolution& sol);
    
    // Move: Di chuyen 1 customer tu trip nay sang trip khac
    bool droneMoveCustomer(PDPSolution& sol);
    
    // Swap: Hoan doi customers giua 2 trips
    bool droneSwapCustomers(PDPSolution& sol);
    
    // Re-assign: Doi drone phuc vu mot trip
    bool droneReassign(PDPSolution& sol);
    
    // Reorder: Thay doi thu tu customers trong 1 trip
    bool droneReorderTrip(PDPSolution& sol);    
    // NEW: Drone Consolidation - Gop nhung trips nho thanh trips lon hon de giam sorties
    bool optimizeDroneConsolidation(PDPSolution& sol);    
    // InsertIntoTrip: Them customer dang duoc phuc vu rieng vao trip co san (NEW)
    bool droneInsertIntoTrip(PDPSolution& sol);
    
    // ============ RUIN AND RECREATE ============
    
    // Ruin: Remove 30-50% of customers from truck routes
    struct RuinInfo {
        std::vector<int> removed_customers;  // Customers removed from routes
        PDPSolution ruined_solution;         // Solution after removal
    };
    RuinInfo ruinSolution(const PDPSolution& sol, double removal_rate);
    
    // Recreate: Greedy re-insert removed customers
    PDPSolution recreateSolution(const RuinInfo& ruin_info);
    
    // Greedy insert a single customer into the best position in any truck route
    bool greedyInsertCustomer(PDPSolution& sol, int customer);
    
    // ============ SEQUENCE-BASED LOCAL SEARCH ============
    
    // Run sequence-based LS (modifies sequence then decodes)
    PDPSolution runSequenceBasedLS(const PDPSolution& initialSol, 
                                   const std::vector<int>& initialSequence,
                                   int max_iterations = 100);
    
    // ============ MAIN PHASES ============
    
    // Phase 1: Truck Local Search (with adaptive selection)
    bool optimizeTruckRoutes(PDPSolution& sol);
    
    // Phase 2: Drone Local Search (with adaptive selection)
    bool optimizeDroneTrips(PDPSolution& sol);
    
    // Evaluate drone trip completion time
    double evaluateDroneTripTime(const std::vector<int>& customers, 
                                 int drone_id,
                                 int truck_id,
                                 double truck_available_time,
                                 int truck_position) const;
    
    // Find standalone type D customers that could be consolidated
    std::vector<int> findStandaloneTypeD(const PDPSolution& sol) const;
};

#endif // PDP_LOCALSEARCH_H
