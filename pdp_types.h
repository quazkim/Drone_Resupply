#ifndef PDP_TYPES_H
#define PDP_TYPES_H

#include <vector>
#include <string>
#include <utility>

using namespace std;

struct PDPData {
    // --- VRP Instance (Giß╗» nguy├¬n cß╗ºa bß║ín) ---
    int numNodes = 0;                    
    int numCustomers = 0; // Sß║╜ ─æ╞░ß╗úc cß║¡p nhß║¡t ─æß╗â ─æß║┐m cß║ú P, DL, D
    int numTrucks = 2;                   
    int truckCapacity = 50; // Giß╗» nguy├¬n 50 tß╗½ file cß╗ºa bß║ín             
    double truckSpeed = 30.0;            
    double truckServiceTime = 3.0; // (╬┤) Thß╗¥i gian phß╗Ñc vß╗Ñ P/DL (ph├║t)       
    double depotReceiveTime = 5.0; // (╬┤t) Thß╗¥i gian xe tß║úi ß╗ƒ depot (ph├║t)       
    int depotIndex = 0; // Index 0-based cß╗ºa depot
    vector<pair<double,double>> coordinates; 
    vector<string> nodeTypes;               
    vector<int> readyTimes;                 
    vector<int> pairIds;                    

    // --- DEPOT CONFIGURATION (FROM README) ---
    pair<double, double> depotCenter = {10.0, 10.0};  // Center depot
    pair<double, double> depotBorder = {0.0, 10.0};   // Border depot
    bool useDepotCenter = true; // true = center (10,10), false = border (0,10)

    // --- C├üC TR╞»ß╗£NG Mß╗ÜI (Bß║«T BUß╗ÿC) ---
    vector<int> demands;                    // (Cß║ºn cho h├ám fitness)
    
    // --- Drone Params (Bß║«T BUß╗ÿC) ---
    int numDrones = 2;                   // (d) Sß╗æ l╞░ß╗úng drone (giß║ú ─æß╗ïnh v├┤ hß║ín trong Giai ─æoß║ín 1)
    int droneCapacity = 2;               // (Q) Sß╗æ orders mß╗ùi trip (2 cho Γëñ20 customers, 10 cho large)
    double droneSpeed = 60.0;            // km/h
    double droneEndurance = 90.0;        // (L_d) Thß╗¥i gian bay tß╗æi ─æa (ph├║t)
    double resupplyTime = 5.0;           // (╬ö) Thß╗¥i gian resupply tß║íi Mß╗ûI ─æiß╗âm (ph├║t)
    double depotDroneLoadTime = 5.0;     // (╬┤d) Thß╗¥i gian drone lß║Ñy h├áng ß╗ƒ depot (ph├║t)

    // --- Distance Matrix (Sß╗¼A Lß╗ûI LOGIC: Cß║ºn 2 ma trß║¡n) ---
    vector<vector<double>> truckDistMatrix; // Manhattan (Truck)
    vector<vector<double>> droneDistMatrix; // Euclidean (Drone)


    // --- Helpers (Cß║¡p nhß║¡t) ---
    int getSeparatorStart() const {
        return numNodes; // Separator bß║»t ─æß║ºu tß╗½ sß╗æ n├║t (total nodes)
    }
    bool isSeparator(int id) const {
        int s = getSeparatorStart();
        return (id >= s && id < s + numTrucks);
    }
    
    // isCustomer: ID l├á 0-based array index
    bool isCustomer(int id) const {
        if (id < 0 || id >= numNodes) return false; 
        if (id == depotIndex) return false; 
        string t = nodeTypes[id]; 
        return (t == "P" || t == "DL" || (t == "D" && readyTimes[id] > 0));
    }
    
    bool isDepot(int id) const {
        return (id == depotIndex); // depotIndex = 0
    }
    
    // Tß╗▒ ─æß╗Öng set drone capacity theo instance size (paper spec)
    int getDroneCapacity() const {
        if (numCustomers <= 20) return 2;  // Small instances
        return 10;  // Large instances
    }
    
};

// Cß║Ñu tr├║c l╞░u th├┤ng tin chi tiß║┐t vß╗ü resupply
struct ResupplyEvent {
    vector<int> customer_ids;      // Danh s├ích kh├ích h├áng ─æ╞░ß╗úc resupply trong trip n├áy
    int drone_id;                  // Drone n├áo phß╗Ñc vß╗Ñ
    int truck_id;                  // Xe tß║úi n├áo gß║╖p
    double drone_depart_time;      // Drone rß╗¥i depot l├║c n├áo
    vector<double> arrive_times;   // Thß╗¥i gian ─æß║┐n tß╗½ng kh├ích h├áng
    vector<double> truck_arrive_times; // Thß╗¥i gian xe ─æß║┐n tß╗½ng ─æiß╗âm
    vector<double> resupply_starts;    // Bß║»t ─æß║ºu resupply tß║íi tß╗½ng ─æiß╗âm
    vector<double> resupply_ends;      // Kß║┐t th├║c resupply tß║íi tß╗½ng ─æiß╗âm
    double drone_return_time;      // Drone vß╗ü depot l├║c n├áo
    double total_flight_time;      // Tß╗òng thß╗¥i gian bay (─æß╗â check endurance)
};

// Cß║Ñu tr├║c l╞░u th├┤ng tin chi tiß║┐t vß╗ü route cß╗ºa tß╗½ng xe
struct TruckRouteInfo {
    int truck_id;
    vector<int> route;        // Thß╗⌐ tß╗▒ c├íc node (bao gß╗ôm depot)
    vector<double> arrival_times;  // Thß╗¥i gian ─æß║┐n mß╗ùi node
    vector<double> departure_times; // Thß╗¥i gian rß╗¥i mß╗ùi node
    double completion_time;   // Thß╗¥i gian ho├án th├ánh (vß╗ü depot)
};

// Cß║Ñu tr├║c m├ú ho├í lß╗¥i giß║úi
struct PDPSolution {
    vector<vector<int>> routes; // (Giß╗» nguy├¬n ─æß╗â t╞░╞íng th├¡ch)
    double totalCost = 0.0;     // (Sß║╜ l├á C_max)
    double totalPenalty = 0.0;  // (Phß║ít)
    bool isFeasible = false;    // (Giß╗» nguy├¬n)
    
    // Th├┤ng tin chi tiß║┐t (Mß╗ÜI)
    vector<TruckRouteInfo> truck_details;
    vector<ResupplyEvent> resupply_events;
    vector<double> drone_completion_times; // Thß╗¥i gian ho├án th├ánh cß╗ºa mß╗ùi drone
    
    // Sequence gß╗æc (─æß╗â local search c├│ thß╗â re-decode)
    vector<int> original_sequence;
};

#endif
