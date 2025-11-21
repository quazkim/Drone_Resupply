#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

// Giả lập tính toán dj1 vs dj2
void testScenario(string name, double dist_prev_to_customer, double dist_depot_to_customer,
                  double drone_available_time, double current_truck_time, double ready_time) {
    
    // Parameters
    double droneSpeed = 60.0;  // km/h
    double truckSpeed = 40.0;  // km/h
    double droneEndurance = 90.0; // minutes
    double depotDroneLoadTime = 5.0;
    double resupplyTime = 5.0;
    double depotReceiveTime = 5.0;
    double truckServiceTime = 2.0;
    
    cout << "\n========== " << name << " ==========" << endl;
    
    // DJ1: DRONE RESUPPLY
    double t_fly = dist_depot_to_customer / droneSpeed * 60.0;
    double T_Drone_Ready = max(drone_available_time, ready_time);
    double T_Drone_Arr = T_Drone_Ready + depotDroneLoadTime + t_fly;
    
    double T_Truck_Arr = current_truck_time + dist_prev_to_customer / truckSpeed * 60.0;
    double T_Start_Resupply = max(T_Truck_Arr, T_Drone_Arr);
    double T_End_Resupply = T_Start_Resupply + resupplyTime + truckServiceTime;
    
    double T_Wait = T_Start_Resupply - T_Drone_Arr;
    double t_return = dist_depot_to_customer / droneSpeed * 60.0;
    double total_drone_time = t_fly + T_Wait + resupplyTime + t_return;
    
    double Cost_Resupply = (total_drone_time <= droneEndurance) ? T_End_Resupply : 1e9;
    
    // DJ2: TRUCK RETURN TO DEPOT
    double dist_prev_to_depot = dist_depot_to_customer; // Giả sử tương tự
    double t_prev_to_depot = dist_prev_to_depot / truckSpeed * 60.0;
    double T_Arr_Depot = current_truck_time + t_prev_to_depot;
    double T_Ready_Leave_Depot = max(T_Arr_Depot + depotReceiveTime, ready_time);
    double t_depot_to_v = dist_depot_to_customer / truckSpeed * 60.0;
    double T_Arr_Customer = T_Ready_Leave_Depot + t_depot_to_v;
    double Cost_DepotReturn = T_Arr_Customer + truckServiceTime;
    
    // Compare
    cout << fixed << setprecision(2);
    cout << "DJ1 (Drone Resupply):" << endl;
    cout << "  Drone fly time: " << t_fly << " min (one way)" << endl;
    cout << "  Drone arrives at: " << T_Drone_Arr << " min" << endl;
    cout << "  Truck arrives at: " << T_Truck_Arr << " min" << endl;
    cout << "  Total drone time: " << total_drone_time << " min" << endl;
    cout << "  Cost DJ1: " << (Cost_Resupply < 1e9 ? to_string((int)Cost_Resupply) : "INFEASIBLE") << endl;
    
    cout << "\nDJ2 (Truck to Depot):" << endl;
    cout << "  Truck to depot: " << t_prev_to_depot << " min" << endl;
    cout << "  Depot to customer: " << t_depot_to_v << " min" << endl;
    cout << "  Cost DJ2: " << Cost_DepotReturn << endl;
    
    cout << "\n>>> DECISION: ";
    if (Cost_Resupply < 1e9 && Cost_Resupply < Cost_DepotReturn) {
        cout << "DJ1 WINS (Drone Resupply) ✓" << endl;
    } else if (Cost_DepotReturn <= Cost_Resupply) {
        cout << "DJ2 WINS (Truck to Depot) ★★★" << endl;
    } else {
        cout << "DJ2 WINS (Drone infeasible) ★★★" << endl;
    }
}

int main() {
    cout << "===== SCENARIOS WHERE DJ2 (TRUCK TO DEPOT) IS BETTER =====" << endl;
    
    // Scenario 1: Customer rất xa depot - Drone hết pin
    testScenario("1. Customer quá xa - Drone hết pin",
                 5.0,   // dist previous to customer
                 50.0,  // dist depot to customer (XA!)
                 0.0,   // drone available
                 100.0, // current truck time
                 0.0);  // ready time
    
    // Scenario 2: Cả 2 drone đang bận
    testScenario("2. Cả 2 drone đang bận",
                 3.0,   // dist previous to customer
                 10.0,  // dist depot to customer
                 200.0, // drone available (MUỘN!)
                 50.0,  // current truck time
                 0.0);  // ready time
    
    // Scenario 3: Xe đang gần depot hơn customer
    testScenario("3. Xe ở gần depot",
                 15.0,  // dist previous to customer (XA!)
                 5.0,   // dist depot to customer (GẦN depot)
                 10.0,  // drone available
                 50.0,  // current truck time
                 0.0);  // ready time
    
    // Scenario 4: Ready time rất muộn, drone phải chờ lâu
    testScenario("4. Ready time muộn - Drone chờ lâu",
                 10.0,  // dist previous to customer
                 10.0,  // dist depot to customer
                 0.0,   // drone available
                 50.0,  // current truck time
                 150.0); // ready time (MUỘN!)
    
    return 0;
}
