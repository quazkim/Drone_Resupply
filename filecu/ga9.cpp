#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <random>
#include <limits>
#include <iomanip>
#include <numeric>
#include <set>
#include <map>
#include <chrono>
#include <climits>
#include "math.h"
#include "tabu.h"

#include "untils.h"
#include "solution.h"
#include "vrp_parser.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// Hàm chuyển đổi từ GA sequence sang Tabu route (bỏ depot)
vector<vector<int>> seqToTabuRoutes(const vector<int>& seq, int depot) {
    vector<vector<int>> routes;
    vector<int> route;
    for (int id : seq) {
        if (id == 0) {  // Sửa: tách routes theo số 0, không phải depot value
            if (!route.empty()) {
                routes.push_back(route);
                route.clear();
            }
        } else {
            route.push_back(id);
        }
    }
    if (!route.empty()) routes.push_back(route);
    return routes;
}

// Hàm chọn 10 cá thể tốt nhất
vector<int> selectBestIndividuals(const vector<double>& costs, int num) {
    vector<int> idx(costs.size());
    iota(idx.begin(), idx.end(), 0);
    partial_sort(idx.begin(), idx.begin() + num, idx.end(),
        [&](int a, int b) { return costs[a] < costs[b]; });
    idx.resize(num);
    return idx;
}

// ======= UTILITY FUNCTIONS =======

double euclidDist(double x1, double y1, double x2, double y2) {
    double d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    return std::round(d * 100.0) / 100.0;
}

void readCVRP(const string& filename, int& n, int& capacity, vector<pair<double,double>>& coords, vector<int>& demand, int& depot, int& vehicles, double& maxDistance, double& serviceTime) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Khong the mo file " << filename << endl;
        exit(1);
    }
    string line;
    n = 0; capacity = 0; depot = 0; vehicles = 0; maxDistance = 0.0; serviceTime = 0.0;
    
    while (getline(file, line)) {
        if (line.find("DIMENSION") != string::npos) {
            size_t pos = line.find(":");

            if (pos != string::npos) n = stoi(line.substr(pos+1));
        } else if (line.find("CAPACITY") != string::npos) {
            size_t pos = line.find(":");
            if (pos != string::npos) capacity = stoi(line.substr(pos+1));
        } else if (line.find("VEHICLE") != string::npos) {
            size_t pos = line.find(":");
            if (pos != string::npos) vehicles = stoi(line.substr(pos+1));
        } else if (line.find("DISTANCE") != string::npos) {
            size_t pos = line.find(":");
            if (pos != string::npos) maxDistance = stod(line.substr(pos+1));
        } else if (line.find("SERVICE_TIME") != string::npos) {
            size_t pos = line.find(":");
            if (pos != string::npos) serviceTime = stod(line.substr(pos+1));
        } else if (line.find("NODE_COORD_SECTION") != string::npos) {
            break;
        }
    }
    if (n <= 0) {
        cerr << "DIMENSION khong hop le trong file." << endl;
        exit(1);
    }
    
    // Display problem configuration
    cout << "\n=== PROBLEM CONFIGURATION ===" << endl;
    cout << "Instance: " << filename << endl;
    cout << "Customers: " << (n-1) << endl;
    cout << "Vehicle capacity: " << capacity << endl;
    cout << "Number of vehicles: " << vehicles << endl;
    if (maxDistance > 0.0) {
        cout << "Maximum distance/time per route: " << maxDistance << endl;
    }
    if (serviceTime > 0.0) {
        cout << "Service time per customer: " << serviceTime << endl;
    }
    
    coords.assign(n+1, {0.0, 0.0});
    for (int i = 0; i < n; ++i) {
        int idx;
        double x, y;
        if (!(file >> idx >> x >> y)) {
            cerr << "Loi khi doc NODE_COORD_SECTION." << endl;
            exit(1);
        }
        if (idx >= 0 && idx <= n) coords[idx] = {x, y};
    }
    getline(file, line);
    while (getline(file, line)) {
        if (line.find("DEMAND_SECTION") != string::npos) break;
    }
    demand.assign(n+1, 0);
    for (int i = 0; i < n; ++i) {
        int idx, d;
        if (!(file >> idx >> d)) {
            cerr << "Loi khi doc DEMAND_SECTION." << endl;
            exit(1);
        }
        if (idx >= 0 && idx <= n) demand[idx] = d;
    }
    getline(file, line);
    while (getline(file, line)) {
        if (line.find("DEPOT_SECTION") != string::npos) break;
    }
    int val;
    depot = -1;
    while (file >> val) {
        if (val == -1) break;
        if (depot == -1) depot = val;
    }
    if (depot == -1) {
        cerr << "Khong tim thay DEPOT trong file." << endl;
        exit(1);
    }
    
    // If no vehicles information found in file, calculate minimum needed
    if (vehicles == 0) {
        int totalDemand = 0;
        for (int i = 2; i <= n; ++i) {
            totalDemand += demand[i];
        }
        vehicles = (int)ceil((double)totalDemand / capacity);
        cout << "No VEHICLE info in file. Calculated minimum vehicles: " << vehicles << endl;
    }
}

vector<vector<double>> buildDist(const vector<pair<double,double>>& coords) {
    int n = coords.size() - 1;
    vector<vector<double>> dist(n+1, vector<double>(n+1, 0));
    for (int i = 1; i <= n; ++i)
        for (int j = 1; j <= n; ++j)
            dist[i][j] = euclidDist(coords[i].first, coords[i].second, coords[j].first, coords[j].second);
    return dist;
}

int routeDemand(const vector<int>& route, const vector<int>& demand) {
    int sum = 0;
    for (size_t i = 1; i < route.size() - 1; ++i) {
        int customer = route[i];
        if (customer > 0 && customer < (int)demand.size()) {
            sum += demand[customer];
        }
    }
    return sum;
}

double routeCost(const vector<int>& route, const vector<vector<double>>& dist) {
    double cost = 0;
    for (size_t i = 0; i < route.size() - 1; ++i)
        cost += dist[route[i]][route[i+1]];
    return cost;
}

double totalCost(const vector<vector<int>>& routes, const vector<vector<double>>& dist) {
    double sum = 0;
    for (const auto& r : routes) 
        sum += routeCost(r, dist);
    return sum;
}

bool checkSolution(const vector<vector<int>>& routes, const vector<int>& demand, int capacity, int n) {
    vector<bool> visited(n+1, false);
    for (const auto& route : routes) {
        if (routeDemand(route, demand) > capacity) return false;
        for (size_t i = 1; i < route.size() - 1; ++i) {
            if (visited[route[i]]) return false;
            visited[route[i]] = true;
        }
    }
    for (int i = 2; i <= n; ++i)
        if (!visited[i]) return false;
    return true;
}

double polarAngle(const pair<double, double>& depot, const pair<double, double>& customer, bool normalize = false) {
    double dx = customer.first - depot.first;
    double dy = customer.second - depot.second;
    double angle = atan2(dy, dx);
    
    if (normalize && angle < 0) {
        angle += 2 * M_PI; // Chuyển từ [-π, π] sang [0, 2π]
    }
    
    return angle;
}

// ======= DECODING & FITNESS CALCULATION =======

// Hàm tính thời gian cho một tuyến
double calculateRouteTime(const vector<int>& route, const vector<vector<double>>& dist, double serviceTime) {
    if (route.size() < 2) return 0.0;
    
    double totalTime = 0.0;
    
    // Tính travel time (khoảng cách di chuyển)
    for (size_t i = 0; i < route.size() - 1; i++) {
        totalTime += dist[route[i]][route[i+1]];
    }
    
    // Thêm service time cho từng khách hàng (không tính depot)
    int customerCount = route.size() - 2; // Trừ depot đầu và cuối
    if (customerCount > 0) {
        totalTime += customerCount * serviceTime;
    }
    
    return totalTime;
}

// Hàm tìm vị trí tốt nhất để chèn customer vào route (minimal distance increase)
pair<int, double> findBestInsertPosition(const vector<int>& route, int customer, 
                                       const vector<vector<double>>& dist, int depot) {
    if (route.size() < 2) return {1, 0.0}; // Insert after depot
    
    int bestPos = 1;
    double minIncrease = numeric_limits<double>::max();
    
    // Thử chèn vào mỗi vị trí trong route (trừ depot đầu và cuối)
    for (size_t i = 1; i < route.size(); i++) {
        int prev = route[i-1];
        int next = route[i];
        
        // Tính cost tăng thêm khi chèn customer vào vị trí này
        double currentCost = dist[prev][next];
        double newCost = dist[prev][customer] + dist[customer][next];
        double increase = newCost - currentCost;
        
        if (increase < minIncrease) {
            minIncrease = increase;
            bestPos = i;
        }
    }
    
    return {bestPos, minIncrease};
}

vector<vector<int>> decodeSeq(const vector<int>& seq, int depot) {
    vector<vector<int>> routes;
    vector<int> currentRoute = {depot};
    for (int v : seq) {
        if (v == 0) {
            currentRoute.push_back(depot);
            routes.push_back(currentRoute);
            currentRoute = {depot};
        } else {
            currentRoute.push_back(v);
        }
    }
    if (currentRoute.size() > 1) {
        currentRoute.push_back(depot);
        routes.push_back(currentRoute);
    }
    return routes;
}

double calculateFitness(const vector<int>& seq, const vector<pair<double,double>>& coords,
                       const vector<int>& demand, int capacity, int depot,
                       const vector<vector<double>>& dist = {},
                       double maxDistance = 0.0, double serviceTime = 0.0) {
    
    if (seq.empty()) {
        return 1e6; // Heavy penalty for empty sequence
    }
    
    vector<vector<int>> routes = decodeSeq(seq, depot);
    if (routes.empty()) {
        return 1e6; // Heavy penalty for no routes
    }
    
    double totalCost = 0;
    double totalPenalty = 0;
    
    for (const auto& route : routes) {
        if (route.size() < 2) continue; // Skip invalid routes
        
        int routeDemand = 0;
        double routeCost = 0;
        
        // Calculate route demand
        for (size_t i = 1; i < route.size() - 1; i++) {
            int customer = route[i];
            if (customer > 0 && customer < (int)demand.size()) {
                routeDemand += demand[customer];
            }
        }
        
        // Capacity penalty
        if (routeDemand > capacity) {
            double violation = routeDemand - capacity;
            totalPenalty += 1000.0 * violation;
        }
        
        // Calculate route distance cost
        for (size_t i = 0; i < route.size() - 1; i++) {
            int from = route[i];
            int to = route[i + 1];
            
            if (from >= 0 && from < (int)coords.size() && 
                to >= 0 && to < (int)coords.size()) {
                routeCost += euclidDist(coords[from].first, coords[from].second,
                                      coords[to].first, coords[to].second);
            }
        }
        
        // Time constraint penalty (nếu có)
        if (maxDistance > 0.0 && !dist.empty()) {
            double routeTime = calculateRouteTime(route, dist, serviceTime);
            if (routeTime > maxDistance) {
                double timeViolation = routeTime - maxDistance;
                totalPenalty += 500.0 * timeViolation; // Time penalty
            }
        }
        
        totalCost += routeCost;
    }
    
    return totalCost + totalPenalty;
}

bool validateCapacity(const vector<int>& seq, const vector<int>& demand, int capacity, int depot, 
                     const vector<vector<double>>& dist = {}, double maxDistance = 0.0, double serviceTime = 0.0) {
    vector<vector<int>> routes = decodeSeq(seq, depot);
    
    for (const auto& route : routes) {
        // Kiểm tra capacity constraint
        int routeDemand = 0;
        for (size_t i = 1; i < route.size() - 1; i++) {
            int customer = route[i];
            if (customer > 0 && customer < (int)demand.size()) {
                routeDemand += demand[customer];
            }
        }
        
        if (routeDemand > capacity) {
            return false;
        }
        
        // Kiểm tra time constraint (nếu có)
        if (maxDistance > 0.0 && !dist.empty()) {
            double routeTime = calculateRouteTime(route, dist, serviceTime);
            if (routeTime > maxDistance) {
                return false;
            }
        }
    }
    
    return true;
}
vector<int> twoOptImproveCustomers(const vector<int>& customers, const vector<vector<double>>& dist, int depot, int maxIter = 50) {
    if (customers.size() < 3) return customers;
    
    // Kiểm tra bounds của dist matrix
    int maxNode = dist.size() - 1;
    for (int cust : customers) {
        if (cust < 0 || cust > maxNode) {
            return customers; // Return original if invalid customer
        }
    }
    
    vector<int> route = customers;
    bool improved = true;
    int iter = 0;
    
    while (improved && iter++ < maxIter) {
        improved = false;
        for (size_t i = 0; i + 1 < route.size(); ++i) {
            for (size_t k = i + 2; k < route.size(); ++k) {
                // Xác định các node liền kề an toàn
                int a = (i == 0) ? depot : route[i-1];
                int b = route[i];
                int c = route[k];
                int d = (k+1 == route.size()) ? depot : route[k+1];
                
                // Kiểm tra bounds trước khi truy cập dist
                if (a < 0 || a > maxNode || b < 0 || b > maxNode || 
                    c < 0 || c > maxNode || d < 0 || d > maxNode) {
                    continue;
                }
                
                double before = dist[a][b] + dist[c][d];
                double after = dist[a][c] + dist[b][d];
                
                if (after + 1e-9 < before) {
                    reverse(route.begin() + i, route.begin() + k + 1);
                    improved = true;
                }
                
                if (improved) break;
            }
            if (improved) break;
        }
    }
    return route;
}
void repairCustomer(vector<int>& seq, int n, mt19937& gen);
void repairZero(vector<int>& seq, int vehicle, mt19937& gen);
void repairCustomerWithLocalSearch(vector<int>& seq, int n, mt19937& gen,
                                 const vector<vector<double>>& dist, 
                                 const vector<int>& demand, int capacity, int depot) ;
// ======= INITIALIZATION METHODS (3) =======

// 1. Enhanced Random Initialization with Multiple Strategies
vector<vector<int>> initPopulationRandom(int vehicle, int n, int capacity, const vector<int>& demand, int populationSize) {
    vector<vector<int>> populationSeq;
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> strategyDist(0.0, 1.0);
    
    for (int p = 0; p < populationSize; ++p) {
        vector<int> customers;
        for (int i = 2; i <= n; ++i) customers.push_back(i);
        
        double strategy = strategyDist(gen);
        vector<vector<int>> groups;
        
        if (strategy < 0.4) {
            // Strategy 1: First-Fit Decreasing (40%)
            sort(customers.begin(), customers.end(), [&](int a, int b) {
                return demand[a] > demand[b]; // Sort by demand descending
            });
            
            vector<int> currentGroup;
            int currentLoad = 0;
            
            for (int cust : customers) {
                if (currentLoad + demand[cust] <= capacity) {
                    currentGroup.push_back(cust);
                    currentLoad += demand[cust];
                } else {
                    if (!currentGroup.empty()) groups.push_back(currentGroup);
                    currentGroup = {cust};
                    currentLoad = demand[cust];
                }
            }
            if (!currentGroup.empty()) groups.push_back(currentGroup);
            
        } else if (strategy < 0.7) {
            // Strategy 2: Best-Fit (30%)
            shuffle(customers.begin(), customers.end(), gen);
            vector<int> routeLoads(vehicle, 0);
            vector<vector<int>> routes(vehicle);
            
            for (int cust : customers) {
                int bestRoute = -1;
                int minWaste = capacity + 1;
                
                // Find route with minimum waste that can fit this customer
                for (int r = 0; r < vehicle; ++r) {
                    if (routeLoads[r] + demand[cust] <= capacity) {
                        int waste = capacity - (routeLoads[r] + demand[cust]);
                        if (waste < minWaste) {
                            minWaste = waste;
                            bestRoute = r;
                        }
                    }
                }
                
                if (bestRoute != -1) {
                    routes[bestRoute].push_back(cust);
                    routeLoads[bestRoute] += demand[cust];
                } else {
                    // Find route with minimum load
                    int minLoadRoute = 0;
                    for (int r = 1; r < vehicle; ++r) {
                        if (routeLoads[r] < routeLoads[minLoadRoute]) {
                            minLoadRoute = r;
                        }
                    }
                    routes[minLoadRoute].push_back(cust);
                    routeLoads[minLoadRoute] += demand[cust];
                }
            }
            
            for (const auto& route : routes) {
                if (!route.empty()) groups.push_back(route);
            }
            
        } else {
            // Strategy 3: Random Shuffle + First-Fit (30%)
            shuffle(customers.begin(), customers.end(), gen);
            
            vector<int> currentGroup;
            int currentLoad = 0;
            
            for (int cust : customers) {
                if (currentLoad + demand[cust] <= capacity) {
                    currentGroup.push_back(cust);
                    currentLoad += demand[cust];
                } else {
                    if (!currentGroup.empty()) groups.push_back(currentGroup);
                    currentGroup = {cust};
                    currentLoad = demand[cust];
                }
            }
            if (!currentGroup.empty()) groups.push_back(currentGroup);
        }
        
        // Ensure we have enough routes
        while ((int)groups.size() < vehicle) groups.push_back({});
        
        // Convert to sequence format
        vector<int> seq;
        for (size_t i = 0; i < groups.size(); ++i) {
            for (int cust : groups[i]) seq.push_back(cust);
            if (i != groups.size() - 1) seq.push_back(0);
        }
        
        populationSeq.push_back(seq);
    }
    
    return populationSeq;
}

// 2. Sweep Initialization
vector<vector<int>> initPopulationSweep(int vehicle, int n, int capacity, const vector<int>& demand, 
                                      const vector<pair<double, double>>& coords, int depot, int populationSize) {
    vector<vector<int>> populationSeq;
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> prob(0.0, 1.0);

    // Tính tổng demand để tính hệ số tightness
    int totalDemand = 0;
    for (int i = 2; i <= n; ++i) {
        totalDemand += demand[i];
    }
    
    // Tính hệ số tightness: T = TotalDemand / (Vehicle × Capacity)
    double tightness = (double)totalDemand / (vehicle * capacity);
    
    // Lấy tọa độ depot
    pair<double, double> depotCoords = coords[depot];

    for (int p = 0; p < populationSize; ++p) {
        // Tạo vector các khách hàng với góc polar
        vector<pair<int, double>> customers;
        for (int i = 2; i <= n; ++i) {
            double angle = polarAngle(depotCoords, coords[i], true);
            customers.emplace_back(i, angle);
        }

        // Sắp xếp khách hàng theo góc polar tăng dần (quét quanh depot)
        sort(customers.begin(), customers.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
            return a.second < b.second;
        });

        // Thêm diversity cho các cá thể khác cá thể đầu tiên
        if (p > 0) {
            uniform_real_distribution<double> angleDist(0.0, 2 * M_PI);
            double offsetAngle = angleDist(gen);
            
            // Áp dụng offset và normalize lại
            for (auto& customer : customers) {
                customer.second += offsetAngle;
                if (customer.second >= 2 * M_PI) {
                    customer.second -= 2 * M_PI;
                }
            }
            
            // Sắp xếp lại sau khi offset
            sort(customers.begin(), customers.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
                return a.second < b.second;
            });
        }

        // Tạo các routes sử dụng thuật toán Sweep
        vector<vector<int>> groups;
        vector<int> currentGroup;
        int currentLoad = 0;
        
        for (const auto& customer : customers) {
            int custIdx = customer.first;
            int custDemand = demand[custIdx];
            
            if (currentLoad + custDemand <= capacity) {
                // Khách hàng vừa đúng capacity - thêm vào route hiện tại
                currentGroup.push_back(custIdx);
                currentLoad += custDemand;
            } else {
                // Khách hàng vượt capacity
                // Finalize current group
                if (!currentGroup.empty()) {
                    groups.push_back(currentGroup);
                }
                
                // Start new group
                currentGroup = {custIdx};
                currentLoad = custDemand;
            }
        }
        
        // Thêm group cuối cùng nếu không rỗng
        if (!currentGroup.empty()) {
            groups.push_back(currentGroup);
        }

        // Đảm bảo số lượng xe đủ
        while ((int)groups.size() < vehicle) {
            groups.push_back({});
        }

        // Chuyển đổi thành sequence với separator 0
        vector<int> seq;
        for (size_t i = 0; i < groups.size(); ++i) {
            for (int cust : groups[i]) {
                seq.push_back(cust);
            }
            if (i != groups.size() - 1) {
                seq.push_back(0);
            }
        }
        
        populationSeq.push_back(seq);
    }

    return populationSeq;
}

// 3. Nearest Neighbor Initialization
vector<vector<int>> initPopulationNearestNeighbor(int vehicle, int n, int capacity, const vector<int>& demand,
                                                const vector<pair<double,double>>& coords, int depot, int populationSize) {
    vector<vector<int>> populationSeq;
    random_device rd;
    mt19937 gen(rd());
    
    // Build distance matrix
    vector<vector<double>> dist(n+1, vector<double>(n+1));
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= n; ++j) {
            dist[i][j] = euclidDist(coords[i].first, coords[i].second, 
                                   coords[j].first, coords[j].second);
        }
    }
    
    for (int p = 0; p < populationSize; ++p) {
        vector<bool> visited(n+1, false);
        visited[depot] = true; // Depot is always visited
        vector<vector<int>> routes;
        
        while (true) {
            // Find unvisited customers
            vector<int> unvisited;
            for (int i = 2; i <= n; ++i) {
                if (!visited[i]) unvisited.push_back(i);
            }
            
            if (unvisited.empty()) break;
            
            // Start new route
            vector<int> route;
            int currentLoad = 0;
            int currentPos = depot;
            
            // Choose random starting customer for diversity
            uniform_int_distribution<> startDist(0, unvisited.size()-1);
            int startIdx = startDist(gen);
            int startCustomer = unvisited[startIdx];
            
            route.push_back(startCustomer);
            visited[startCustomer] = true;
            currentLoad += demand[startCustomer];
            currentPos = startCustomer;
            
            // Build route using nearest neighbor with capacity constraint
            while (true) {
                int nearestCustomer = -1;
                double nearestScore = numeric_limits<double>::max();
                
                // Find best unvisited customer that fits in capacity
                for (int i = 2; i <= n; ++i) {
                    if (!visited[i] && currentLoad + demand[i] <= capacity) {
                        double distance = dist[currentPos][i];
                        double score;
                        
                        // For large problems, use efficiency score (distance/demand)
                        if (n > 100) {
                            score = distance / max(1.0, (double)demand[i]);
                        } else {
                            score = distance; // Standard nearest neighbor
                        }
                        
                        if (score < nearestScore) {
                            nearestScore = score;
                            nearestCustomer = i;
                        }
                    }
                }
                
                if (nearestCustomer == -1) break; // No more customers can fit
                
                route.push_back(nearestCustomer);
                visited[nearestCustomer] = true;
                currentLoad += demand[nearestCustomer];
                currentPos = nearestCustomer;
            }
            
            routes.push_back(route);
            
            if (routes.size() >= (size_t)vehicle) break; // Vehicle limit reached
        }
        
        // Convert routes to sequence
        vector<int> seq;
        for (size_t i = 0; i < routes.size(); ++i) {
            for (int customer : routes[i]) {
                seq.push_back(customer);
            }
            if (i < routes.size() - 1) {
                seq.push_back(0);
            }
        }
        
        populationSeq.push_back(seq);
    }
    
    return populationSeq;
}

// 4. Cluster-based Initialization
vector<vector<int>> initPopulationCluster(int vehicle, int n, int capacity, const vector<int>& demand, 
                                        const vector<pair<double,double>>& coords, int depot, int populationSize) {
    vector<vector<int>> populationSeq;
    random_device rd;
    mt19937 gen(rd());
    
    for (int p = 0; p < populationSize; ++p) {
        // Initialize centroids randomly
        vector<pair<double,double>> centroids;
        uniform_real_distribution<> coordDist(0, 100); // Adjust range as needed
        
        for (int k = 0; k < vehicle; ++k) {
            centroids.push_back({coordDist(gen), coordDist(gen)});
        }
        
        // Assign customers to clusters using K-means
        vector<int> assignment(n+1);
        for (int iter = 0; iter < 10; ++iter) { // 10 iterations of k-means
            // Assign customers to nearest centroid
            for (int i = 2; i <= n; ++i) {
                double minDist = numeric_limits<double>::max();
                int bestCluster = 0;
                
                for (int k = 0; k < vehicle; ++k) {
                    double d = euclidDist(coords[i].first, coords[i].second, 
                                        centroids[k].first, centroids[k].second);
                    if (d < minDist) { 
                        minDist = d; 
                        bestCluster = k;
                    }
                }
                
                assignment[i] = bestCluster;
            }
            
            // Update centroids
            vector<double> sumX(vehicle, 0), sumY(vehicle, 0);
            vector<int> count(vehicle, 0);
            
            for (int i = 2; i <= n; ++i) {
                sumX[assignment[i]] += coords[i].first;
                sumY[assignment[i]] += coords[i].second;
                count[assignment[i]]++;
            }
            
            for (int k = 0; k < vehicle; ++k) {
                if (count[k] > 0) {
                    centroids[k] = {sumX[k]/count[k], sumY[k]/count[k]};
                }
            }
        }
        
        // Create routes from clusters with capacity constraint
        vector<vector<int>> routes(vehicle);
        
        // First, assign customers to their clusters
        for (int i = 2; i <= n; ++i) {
            routes[assignment[i]].push_back(i);
        }
        
        // Handle capacity constraints
        vector<int> seq;
        for (int k = 0; k < vehicle; ++k) {
            vector<int> cluster = routes[k];
            
            // Sort customers by distance from centroid
            sort(cluster.begin(), cluster.end(), [&](int a, int b) {
                double da = euclidDist(coords[a].first, coords[a].second,
                                     centroids[k].first, centroids[k].second);
                double db = euclidDist(coords[b].first, coords[b].second,
                                     centroids[k].first, centroids[k].second);
                return da < db;
            });
            
            // Create feasible routes
            int clusterLoad = 0;
            vector<int> route;
            
            for (int customer : cluster) {
                if (clusterLoad + demand[customer] <= capacity) {
                    route.push_back(customer);
                    clusterLoad += demand[customer];
                } else {
                    // This customer doesn't fit, add to next vehicle
                    if (k+1 < vehicle) {
                        routes[k+1].push_back(customer);
                    } else {
                        // No more vehicles, just add at the end and repair later
                        route.push_back(customer);
                    }
                }
            }
            
            // Add route to sequence
            for (int customer : route) {
                seq.push_back(customer);
            }
            
            if (k < vehicle - 1) {
                seq.push_back(0); // Separator
            }
        }
        
        populationSeq.push_back(seq);
    }
    
    return populationSeq;
}

// ======= HYBRID INITIALIZATION =======
vector<vector<int>> initStructuredPopulation(int populationSize, int vehicle, int n, int capacity, const vector<int>& demand, const vector<pair<double,double>>& coords, const vector<vector<double>>& dist, int depot, int runNumber = 1) {
    random_device rd;
    unsigned int seed = rd() + runNumber * 54321;  // Different seed for initialization
    mt19937 gen(seed);
    
    // Calculate tightness factor
    int totalDemand = 0;
    for (int i = 2; i <= n; ++i) totalDemand += demand[i];
    double tightness = (double)totalDemand / (vehicle * capacity);
    
    // Adaptive distribution based on problem size
    // For large problems (n > 100), favor more structured methods
    int sweepCount, randomCount, nnCount, clusterCount;
    
    if (n > 100) {
        // Large problems: More sweep (40%) and nearest neighbor (35%), less random
        sweepCount = (int)(populationSize * 0.40);
        nnCount = (int)(populationSize * 0.35);
        randomCount = (int)(populationSize * 0.15);
        clusterCount = populationSize - sweepCount - randomCount - nnCount;
    } else {
        // Small-medium problems: Balanced distribution
        sweepCount = (int)(populationSize * 0.30);
        randomCount = (int)(populationSize * 0.25);
        nnCount = (int)(populationSize * 0.25);
        clusterCount = populationSize - sweepCount - randomCount - nnCount;
    }
    
    cout << " Population distribution: Sweep=" << sweepCount 
         << ", Random=" << randomCount 
         << ", NearestNeighbor=" << nnCount
         << ", Cluster=" << clusterCount << endl;
    
    vector<vector<int>> population;
    
    // 1. Create sweep-based solutions with 2-opt improvement
    for (int p = 0; p < sweepCount; ++p) {
        // Get depot coordinates
        pair<double, double> depotCoords = coords[depot];
        
        // Calculate polar angles for all customers
        vector<pair<int, double>> customers;
        for (int i = 2; i <= n; ++i) {
            double angle = polarAngle(depotCoords, coords[i], true);
            customers.emplace_back(i, angle);
        }
        
        // Add randomization with different starting angles
        uniform_real_distribution<double> angleDist(0.0, 2 * M_PI);
        double startAngle = angleDist(gen);
        
        // Apply offset and normalize
        for (auto& customer : customers) {
            customer.second = fmod(customer.second + startAngle, 2 * M_PI);
        }
        
        // Sort by angle
        sort(customers.begin(), customers.end(), 
             [](const auto& a, const auto& b) { return a.second < b.second; });
        
        // Create routes using sweep
        vector<vector<int>> routes;
        vector<int> currentRoute;
        int currentLoad = 0;
        
        for (const auto& customer : customers) {
    int custId = customer.first;
    double angle = customer.second;
    
    if (currentLoad + demand[custId] <= capacity) {
        currentRoute.push_back(custId);
        currentLoad += demand[custId];
    } else {
        if (!currentRoute.empty()) {
            // Apply 2-opt improvement to the route
            currentRoute = twoOptImproveCustomers(currentRoute, dist, depot, 30);
            routes.push_back(currentRoute);
        }
        currentRoute = {custId};
        currentLoad = demand[custId];
    }
}
        
        if (!currentRoute.empty()) {
            currentRoute = twoOptImproveCustomers(currentRoute, dist, depot, 30);
            routes.push_back(currentRoute);
        }
        
        // Convert to sequence with separators (with size limit)
        vector<int> seq;
        seq.reserve(n + vehicle); // Pre-allocate memory
        
        for (size_t i = 0; i < routes.size() && seq.size() < n + vehicle - 1; ++i) {
            for (int cust : routes[i]) {
                if (seq.size() < n + vehicle - 1) seq.push_back(cust);
            }
            if (i < routes.size() - 1 && seq.size() < n + vehicle - 1) seq.push_back(0);
        }
        
        // Limit the number of empty routes to prevent memory explosion
        int maxEmptyRoutes = 2; // Limit empty routes
        int emptyRoutesAdded = 0;
        while ((int)routes.size() < vehicle && emptyRoutesAdded < maxEmptyRoutes) {
            if (!seq.empty() && seq.back() != 0 && seq.size() < n + vehicle - 1) seq.push_back(0);
            if (seq.size() < n + vehicle - 1) seq.push_back(0);
            emptyRoutesAdded++;
        }
        
        repairCustomer(seq, n, gen);
        repairZero(seq, vehicle, gen);
        population.push_back(seq);
    }
    
    // 2. Add random solutions
    vector<vector<int>> randomPop = initPopulationRandom(vehicle, n, capacity, demand, randomCount);
    population.insert(population.end(), randomPop.begin(), randomPop.end());
    
    // 3. Add nearest neighbor solutions
    vector<vector<int>> nnPop = initPopulationNearestNeighbor(vehicle, n, capacity, demand, coords, depot, nnCount);
    for (auto& seq : nnPop) {
        repairCustomer(seq, n, gen);
        repairZero(seq, vehicle, gen);
    }
    population.insert(population.end(), nnPop.begin(), nnPop.end());
    
    // 4. Add cluster-based solutions with improvements
    vector<vector<int>> clusterPop = initPopulationCluster(vehicle, n, capacity, demand, coords, depot, clusterCount);
    
    // Apply 2-opt to cluster solutions
    for (auto& seq : clusterPop) {
        vector<vector<int>> routes = decodeSeq(seq, depot);
        for (auto& route : routes) {
            if (route.size() <= 2) continue;
            vector<int> customers(route.begin()+1, route.end()-1);
            customers = twoOptImproveCustomers(customers, dist, depot, 30);
            route = {depot};
            route.insert(route.end(), customers.begin(), customers.end());
            route.push_back(depot);
        }
        
        // Convert back to sequence
        seq.clear();
        for (size_t i = 0; i < routes.size(); ++i) {
            for (size_t j = 1; j < routes[i].size()-1; ++j) {
                seq.push_back(routes[i][j]);
            }
            if (i < routes.size() - 1) seq.push_back(0);
        }
        repairCustomer(seq, n, gen);
        repairZero(seq, vehicle, gen);
    }
    
    population.insert(population.end(), clusterPop.begin(), clusterPop.end());
    cout << "Generated " << population.size() << " individuals with improved methods" << endl;
    return population;
}

// ======= REPAIR OPERATORS =======

void repairZero(vector<int>& seq, int vehicle, mt19937& gen) {
    // Bước 1: Xóa các số 0 liền nhau
    for (auto it = seq.begin(); it != seq.end(); ) {
        if (*it == 0 && (it + 1) != seq.end() && *(it + 1) == 0) {
            // Nếu số hiện tại là 0 và số tiếp theo cũng là 0, xóa số hiện tại
            it = seq.erase(it);
        } else {
            ++it;
        }
    }
    
    // Bước 2: Xóa số 0 ở đầu chuỗi
    while (!seq.empty() && seq.front() == 0) {
        seq.erase(seq.begin());
    }
    
    // Bước 3: Xóa số 0 ở cuối chuỗi
    while (!seq.empty() && seq.back() == 0) {
        seq.pop_back();
    }
    
    int zeroCount = count(seq.begin(), seq.end(), 0);
    int needZero = max(0, vehicle - 1);
    
    if (zeroCount == needZero) {
        return;  // Không cần sửa
    }
    
    // Trích xuất các tuyến từ chuỗi
    vector<vector<int>> routes;
    vector<int> currentRoute;
    
    for (int v : seq) {
        if (v == 0) {
            if (!currentRoute.empty()) {
                routes.push_back(currentRoute);
                currentRoute.clear();
            }
        } 
        else {
            currentRoute.push_back(v);
        }
    }
    
    if (!currentRoute.empty()) {
        routes.push_back(currentRoute);
    }
    
    // Trường hợp 1: Quá nhiều số 0 - xóa từ tuyến ngắn nhất
    while (zeroCount > needZero && routes.size() > 1) {
        // Tìm tuyến ngắn nhất
        int shortestIdx = 0;
        size_t minLength = routes[0].size();
        
        for (size_t i = 1; i < routes.size(); ++i) {
            if (routes[i].size() < minLength) {
                minLength = routes[i].size();
                shortestIdx = i;
            }
        }
        
        // Hợp nhất tuyến ngắn nhất với tuyến tiếp theo
        if (shortestIdx + 1 < routes.size()) {
            routes[shortestIdx + 1].insert(routes[shortestIdx + 1].begin(), 
                                          routes[shortestIdx].begin(), 
                                          routes[shortestIdx].end());
            routes.erase(routes.begin() + shortestIdx);
        } else {
            // Nếu là tuyến cuối, hợp nhất với tuyến trước đó
            routes[shortestIdx - 1].insert(routes[shortestIdx - 1].end(), 
                                         routes[shortestIdx].begin(), 
                                         routes[shortestIdx].end());
            routes.erase(routes.begin() + shortestIdx);
        }
        
        zeroCount--;
    }
    
    // Trường hợp 2: Quá ít số 0 - thêm vào tuyến dài nhất
    while (zeroCount < needZero) {
        if (routes.empty()) {
            // Trường hợp đặc biệt: không có tuyến nào
            routes.push_back({});
            routes.push_back({});
            zeroCount++;
            continue;
        }
        
        // Tìm tuyến dài nhất
        int longestIdx = 0;
        size_t maxLength = routes[0].size();
        
        for (size_t i = 1; i < routes.size(); ++i) {
            if (routes[i].size() > maxLength) {
                maxLength = routes[i].size();
                longestIdx = i;
            }
        }
        
        if (maxLength <= 1) {
            // Tất cả tuyến đều rất ngắn, thêm một tuyến trống
            routes.push_back({});
        } else {
            // Chia tuyến dài nhất thành hai nửa
            int splitPoint = routes[longestIdx].size() / 2;
            vector<int> firstHalf(routes[longestIdx].begin(), 
                                 routes[longestIdx].begin() + splitPoint);
            vector<int> secondHalf(routes[longestIdx].begin() + splitPoint, 
                                  routes[longestIdx].end());
            
            routes[longestIdx] = firstHalf;
            routes.insert(routes.begin() + longestIdx + 1, secondHalf);
        }
        
        zeroCount++;
    }
    
    // Chuyển đổi lại sang định dạng chuỗi
    seq.clear();
    for (size_t i = 0; i < routes.size(); ++i) {
        for (int v : routes[i]) {
            seq.push_back(v);
        }
        if (i < routes.size() - 1) {
            seq.push_back(0);  // Thêm dấu phân cách giữa các tuyến
        }
    }
}

// Optimized repairCustomer function using count tracking
void repairCustomer(vector<int>& seq, int n, mt19937& gen) {
    // Đếm số lần xuất hiện của mỗi customer
    vector<int> count(n+1, 0);
    for (int v : seq) {
        if (v != 0 && v >= 2 && v <= n) {
            count[v]++;
        }
    }
    
    // Tìm danh sách customers bị thiếu (count = 0)
    vector<int> missing;
    for (int i = 2; i <= n; ++i) {
        if (count[i] == 0) {
            missing.push_back(i);
        }
    }
    
    // Shuffle để randomize việc thay thế
    shuffle(missing.begin(), missing.end(), gen);
    int missingIdx = 0;
    
    // Lặp qua sequence và sửa từng vị trí
    for (int& v : seq) {
        if (v != 0 && v >= 2 && v <= n) {
            if (count[v] > 1) {
                // Customer này bị lặp
                if (missingIdx < missing.size()) {
                    // Thay bằng customer bị thiếu
                    int oldCustomer = v;
                    int newCustomer = missing[missingIdx];
                    
                    v = newCustomer;
                    count[oldCustomer]--; // Giảm count của customer cũ
                    count[newCustomer]++; // Tăng count của customer mới
                    missingIdx++;
                } else {
                    // Không còn customer thiếu nào, thay bằng 0
                    count[v]--; // Giảm count khi thay bằng 0
                    v = 0;
                }
            }
        }
    }
    
    // Kiểm tra xem có customer nào còn thiếu không (count = 0)
    for (int i = 2; i <= n; ++i) {
        if (count[i] == 0) {
            // Tìm route ngắn nhất để thêm customer này
            vector<vector<int>*> routes;
            vector<int> currentRoute;
            
            // Tách sequence thành các routes (dùng pointer để modify)
            for (int& v : seq) {
                if (v == 0) {
                    if (!currentRoute.empty()) {
                        routes.push_back(new vector<int>(currentRoute));
                        currentRoute.clear();
                    }
                } else {
                    currentRoute.push_back(v);
                }
            }
            if (!currentRoute.empty()) {
                routes.push_back(new vector<int>(currentRoute));
            }
            
            if (routes.empty()) {
                // Không có route nào, thêm vào cuối sequence
                seq.push_back(i);
            } else {
                // Tìm route ngắn nhất
                int shortestIdx = 0;
                size_t minLength = routes[0]->size();
                
                for (size_t j = 1; j < routes.size(); ++j) {
                    if (routes[j]->size() < minLength) {
                        minLength = routes[j]->size();
                        shortestIdx = j;
                    }
                }
                
                // Thêm customer vào route ngắn nhất
                routes[shortestIdx]->push_back(i);
                
                // Rebuild sequence từ các routes
                seq.clear();
                for (size_t j = 0; j < routes.size(); ++j) {
                    for (int customer : *routes[j]) {
                        seq.push_back(customer);
                    }
                    if (j < routes.size() - 1) {
                        seq.push_back(0);
                    }
                }
            }
            
            // Cleanup memory
            for (auto* route : routes) {
                delete route;
            }
            
            count[i] = 1; // Cập nhật count
        }
    }
    
    // Kiểm tra cuối cùng: nếu vẫn còn customer nào có count > 1, thay bằng 0
    bool hasError = true;
    while (hasError) {
        hasError = false;
        for (int& v : seq) {
            if (v != 0 && v >= 2 && v <= n && count[v] > 1) {
                count[v]--; // Giảm count
                v = 0;      // Thay bằng 0
                hasError = true;
                break; // Chỉ thay 1 lần mỗi vòng lặp
            }
        }
    }
}
void repairCustomerWithLocalSearch(vector<int>& seq, int n, mt19937& gen,
                                 const vector<vector<double>>& dist, 
                                 const vector<int>& demand, int capacity, int depot,
                                 double maxDistance = 0.0, double serviceTime = 0.0, int maxVehicles = -1) {
    // Bước 1: Sử dụng hàm repairCustomer để sửa duplicate và missing customers
    repairCustomer(seq, n, gen);
    
    // Bước 2: Decode thành routes và kiểm tra time violations
    vector<vector<int>> routes = decodeSeq(seq, depot);
    
    // Bước 3: Aggressive repair cho time constraints
    if (maxDistance > 0.0) {
        bool hasTimeViolation = true;
        int maxIterations = 10; // Tăng số iterations
        
        for (int iter = 0; iter < maxIterations && hasTimeViolation; iter++) {
            hasTimeViolation = false;
            
            // Tìm route có violation lớn nhất để ưu tiên sửa
            int worstRouteIdx = -1;
            double maxViolation = 0.0;
            
            for (size_t i = 0; i < routes.size(); ++i) {
                double routeTime = calculateRouteTime(routes[i], dist, serviceTime);
                if (routeTime > maxDistance) {
                    double violation = routeTime - maxDistance;
                    if (violation > maxViolation) {
                        maxViolation = violation;
                        worstRouteIdx = i;
                        hasTimeViolation = true;
                    }
                }
            }
            
            if (worstRouteIdx == -1) break; // Không còn violations
            
            // Aggressive strategy: Di chuyển nhiều customers từ worst route
            vector<int>& worstRoute = routes[worstRouteIdx];
            bool routeFixed = false;
            
            // Strategy 1: Di chuyển từ cuối route (nhiều customers)
            int customersToMove = min(3, (int)worstRoute.size() - 3); // Tối đa 3 customers
            for (int moveCount = 0; moveCount < customersToMove && !routeFixed; moveCount++) {
                if (worstRoute.size() <= 3) break; // Không thể di chuyển thêm
                
                int customerToMove = worstRoute[worstRoute.size() - 2]; // Customer cuối
                int customerDemand = demand[customerToMove];
                
                // Tìm route có thể nhận customer (ưu tiên route có ít load)
                int bestTargetRoute = -1;
                double bestPriority = numeric_limits<double>::max();
                int bestPosition = -1;
                
                for (size_t j = 0; j < routes.size(); ++j) {
                    if (j == worstRouteIdx) continue; // Bỏ qua route hiện tại
                    
                    // Tính current load
                    int routeLoad = 0;
                    for (size_t k = 1; k < routes[j].size() - 1; k++) {
                        routeLoad += demand[routes[j][k]];
                    }
                    
                    // Kiểm tra capacity
                    if (routeLoad + customerDemand <= capacity) {
                        // Kiểm tra time constraint sau khi thêm
                        auto insertInfo = findBestInsertPosition(routes[j], customerToMove, dist, depot);
                        vector<int> tempRoute = routes[j];
                        tempRoute.insert(tempRoute.begin() + insertInfo.first, customerToMove);
                        
                        double newRouteTime = calculateRouteTime(tempRoute, dist, serviceTime);
                        if (newRouteTime <= maxDistance) {
                            // Priority = load% + distance_increase/100 (ưu tiên route ít tải và distance increase thấp)
                            double loadRatio = (double)routeLoad / capacity;
                            double priority = loadRatio + (insertInfo.second / 100.0);
                            
                            if (priority < bestPriority) {
                                bestPriority = priority;
                                bestTargetRoute = j;
                                bestPosition = insertInfo.first;
                            }
                        }
                    }
                }
                
                // Thực hiện di chuyển nếu tìm được route phù hợp
                if (bestTargetRoute != -1) {
                    worstRoute.erase(worstRoute.end() - 2); // Xóa customer
                    routes[bestTargetRoute].insert(routes[bestTargetRoute].begin() + bestPosition, customerToMove);
                    
                    // Kiểm tra xem route đã được sửa chưa
                    double newWorstRouteTime = calculateRouteTime(worstRoute, dist, serviceTime);
                    if (newWorstRouteTime <= maxDistance) {
                        routeFixed = true;
                    }
                }
            }
            
            // Strategy 2: DISABLED - Không split route để giữ nguyên số vehicles
            // (Chỉ di chuyển customers giữa các routes hiện có)
            
            // Strategy 3: DISABLED - Không tạo routes đơn để giữ nguyên số vehicles  
            // (Chỉ di chuyển customers giữa các routes hiện có)
        }
    }
    
    // Bước 4: Áp dụng 2-opt local search cho từng route
    for (auto& route : routes) {
        if (route.size() <= 3) continue; // Bỏ qua route quá ngắn
        
        // Trích xuất customers từ route (bỏ depot đầu và cuối)
        vector<int> customers;
        for (size_t i = 1; i < route.size() - 1; ++i) {
            customers.push_back(route[i]);
        }
        
        // Áp dụng 2-opt improvement
        customers = twoOptImproveCustomers(customers, dist, depot, 50);
        
        // Cập nhật lại route với customers đã được cải thiện
        route = {depot};
        route.insert(route.end(), customers.begin(), customers.end());
        route.push_back(depot);
    }
    
    // Bước 5: Chuyển đổi lại thành sequence format
    seq.clear();
    for (size_t i = 0; i < routes.size(); ++i) {
        // Thêm customers từ route (bỏ depot đầu và cuối)
        for (size_t j = 1; j < routes[i].size() - 1; ++j) {
            seq.push_back(routes[i][j]);
        }
        // Thêm separator giữa các routes (trừ route cuối)
        if (i < routes.size() - 1) {
            seq.push_back(0);
        }
    }
}

// ======= CROSSOVER OPERATORS (3) =======

// Helper for sequence conversion
// vector<int> convertToSequenceFormat(const vector<int>& customers, int vehicle, const vector<int>& demand, int capacity, mt19937& gen) {
//     vector<int> seq;
//     vector<vector<int>> routes;
//     vector<int> currentRoute;
//     int currentLoad = 0;
    
//     for (int customer : customers) {
//         if (currentLoad + demand[customer] <= capacity) {
//             currentRoute.push_back(customer);
//             currentLoad += demand[customer];
//         } else {
//             if (!currentRoute.empty()) routes.push_back(currentRoute);
//             currentRoute = {customer};
//             currentLoad = demand[customer];
//         }
//     }
    
//     if (!currentRoute.empty()) routes.push_back(currentRoute);
    
//     // Ensure we don't exceed vehicle count
//     while (routes.size() > (size_t)vehicle) {
//         vector<int> lastRoute = routes.back();
//         routes.pop_back();
//         bool merged = false;
        
//         for (auto& route : routes) {
//             int routeLoad = 0;
//             for (int c : route) routeLoad += demand[c];
            
//             int lastRouteLoad = 0;
//             for (int c : lastRoute) lastRouteLoad += demand[c];
            
//             if (routeLoad + lastRouteLoad <= capacity) {
//                 route.insert(route.end(), lastRoute.begin(), lastRoute.end());
//                 merged = true;
               
//                 break;
//             }
//         }
        
//         if (!merged && !routes.empty()) {
//             // If can't merge, just append to first route
//             routes[0].insert(routes[0].end(), lastRoute.begin(), lastRoute.end());
//         }
//     }
    
//     // Convert routes to sequence
//     for (size_t i = 0; i < routes.size(); ++i) {
//         for (int customer : routes[i]) seq.push_back(customer);
//         if (i < routes.size() - 1) seq.push_back(0);
//     }
    
//     return seq;
// }

// 1. One-Point Crossover
pair<vector<int>, vector<int>> crossoverOnePoint(const vector<int>& parent1, const vector<int>& parent2, 
                                              int n, int vehicle, const vector<int>& demand, 
                                              int capacity, int depot, mt19937& gen,
                                              const vector<vector<double>>& dist,
                                              double maxDistance = 0.0, double serviceTime = 0.0) {
    int len1 = parent1.size();
    int len2 = parent2.size();
    int len = min(len1, len2);
    
    if (len < 2) {
        return {parent1, parent2}; // Can't crossover, return parents
    }
    
    uniform_int_distribution<> dis(1, len - 1);
    int cut = dis(gen);
    
    vector<int> child1, child2;
    
    child1.insert(child1.end(), parent1.begin(), parent1.begin() + cut);
    child1.insert(child1.end(), parent2.begin() + cut, parent2.end());
    
    child2.insert(child2.end(), parent2.begin(), parent2.begin() + cut);
    child2.insert(child2.end(), parent1.begin() + cut, parent1.end());

    repairCustomerWithLocalSearch(child1, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
    repairZero(child1, vehicle, gen);

    repairCustomerWithLocalSearch(child2, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
    repairZero(child2, vehicle, gen);

    return {child1, child2};
}

// 2. Order Crossover (OX)
pair<vector<int>, vector<int>> crossoverOX(const vector<int>& parent1, const vector<int>& parent2, 
                                        int n, int vehicle, const vector<int>& demand, 
                                        int capacity, int depot, mt19937& gen, const vector<vector<double>>& dist,
                                        double maxDistance = 0.0, double serviceTime = 0.0) {
    // Bước 1: Trích xuất customers (bỏ hết số 0)
    vector<int> customers1, customers2;
    for (int v : parent1) if (v != 0) customers1.push_back(v);
    for (int v : parent2) if (v != 0) customers2.push_back(v);
    
    if (customers1.empty() || customers2.empty()) {
        return {parent1, parent2};
    }
    
    int minLen = min(customers1.size(), customers2.size());
    if (minLen < 2) {
        return {parent1, parent2};
    }
    
    // Bước 2: Lai ghép OX trên customer list
    uniform_int_distribution<> dis(0, minLen - 1);
    int cut1 = dis(gen);
    int cut2 = dis(gen);
    if (cut1 > cut2) swap(cut1, cut2);
    
    vector<int> child1Customers(customers1.size(), -1);
    vector<int> child2Customers(customers2.size(), -1);
    set<int> used1, used2;
    
    // Copy đoạn giữa
    for (int i = cut1; i <= cut2 && i < minLen; ++i) {
        child1Customers[i] = customers1[i];
        child2Customers[i] = customers2[i];
        used1.insert(customers1[i]);
        used2.insert(customers2[i]);
    }
    
    // Fill phần còn lại
    int pos1 = 0, pos2 = 0;
    for (int cust : customers2) {
        if (used1.find(cust) == used1.end()) {
            while (pos1 < (int)child1Customers.size() && child1Customers[pos1] != -1) pos1++;
            if (pos1 < (int)child1Customers.size()) child1Customers[pos1] = cust;
        }
    }
    for (int cust : customers1) {
        if (used2.find(cust) == used2.end()) {
            while (pos2 < (int)child2Customers.size() && child2Customers[pos2] != -1) pos2++;
            if (pos2 < (int)child2Customers.size()) child2Customers[pos2] = cust;
        }
    }
    
    // Bước 3: Chèn vehicle-1 số 0 ngẫu nhiên
    vector<int> child1 = child1Customers;
    vector<int> child2 = child2Customers;
    
    int zerosToAdd = vehicle - 1;
    
    // Pre-calculate insertion positions to avoid exponential growth
    vector<int> positions1, positions2;
    uniform_int_distribution<> posDis1(0, child1.size());
    uniform_int_distribution<> posDis2(0, child2.size());
    
    for (int i = 0; i < zerosToAdd; ++i) {
        positions1.push_back(posDis1(gen));
        positions2.push_back(posDis2(gen));
    }
    
    // Sort positions in descending order to insert from right to left
    sort(positions1.rbegin(), positions1.rend());
    sort(positions2.rbegin(), positions2.rend());
    
    for (int pos : positions1) {
        if (pos <= child1.size()) child1.insert(child1.begin() + pos, 0);
    }
    for (int pos : positions2) {
        if (pos <= child2.size()) child2.insert(child2.begin() + pos, 0);
    }
    
    // Repair customers first, then zeros
    repairCustomerWithLocalSearch(child1, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
    repairZero(child1, vehicle, gen);
    repairCustomerWithLocalSearch(child2, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
    repairZero(child2, vehicle, gen);

    return {child1, child2};
}

// 3. Partially Mapped Crossover (PMX)
pair<vector<int>, vector<int>> crossoverPMX(const vector<int>& parent1, const vector<int>& parent2,
                                         int n, int vehicle, const vector<int>& demand,
                                         int capacity, int depot, mt19937& gen, const vector<vector<double>>& dist,
                                         double maxDistance = 0.0, double serviceTime = 0.0) {
    // Bước 1: Trích xuất customers (bỏ hết số 0)
    vector<int> customers1, customers2;
    for (int v : parent1) if (v != 0) customers1.push_back(v);
    for (int v : parent2) if (v != 0) customers2.push_back(v);
    
    if (customers1.empty() || customers2.empty()) {
        return {parent1, parent2};
    }
    
    int minLen = min(customers1.size(), customers2.size());
    if (minLen < 2) {
        return {parent1, parent2};
    }
    
    // Bước 2: Lai ghép PMX trên customer list
    uniform_int_distribution<> dis(0, minLen - 1);
    int cut1 = dis(gen);
    int cut2 = dis(gen);
    if (cut1 > cut2) swap(cut1, cut2);
    
    vector<int> child1 = customers1;
    vector<int> child2 = customers2;
    
    // Tạo mapping và swap đoạn giữa
    map<int, int> mapping1to2, mapping2to1;
    for (int i = cut1; i <= cut2 && i < minLen; ++i) {
        mapping1to2[customers1[i]] = customers2[i];  
        mapping2to1[customers2[i]] = customers1[i];
        child1[i] = customers2[i];
        child2[i] = customers1[i];
    }
    
    // Áp dụng mapping để tránh duplicate
    for (int i = 0; i < minLen; ++i) {
        if (i < cut1 || i > cut2) {
            // Fix child1
            int val = child1[i];
            while (mapping1to2.find(val) != mapping1to2.end()) {
                val = mapping1to2[val];
            }
            child1[i] = val;
            
            // Fix child2
            val = child2[i];
            while (mapping2to1.find(val) != mapping2to1.end()) {
                val = mapping2to1[val];
            }
            child2[i] = val;
        }
    }
    
    // Bước 3: Chèn vehicle-1 số 0 ngẫu nhiên
    int zerosToAdd = vehicle - 1;
    
    // Pre-calculate insertion positions to avoid exponential growth
    vector<int> positions1, positions2;
    uniform_int_distribution<> posDis1(0, child1.size());
    uniform_int_distribution<> posDis2(0, child2.size());
    
    for (int i = 0; i < zerosToAdd; ++i) {
        positions1.push_back(posDis1(gen));
        positions2.push_back(posDis2(gen));
    }
    
    // Sort positions in descending order to insert from right to left
    sort(positions1.rbegin(), positions1.rend());
    sort(positions2.rbegin(), positions2.rend());
    
    for (int pos : positions1) {
        if (pos <= child1.size()) child1.insert(child1.begin() + pos, 0);
    }
    for (int pos : positions2) {
        if (pos <= child2.size()) child2.insert(child2.begin() + pos, 0);
    }
    
    // Repair zeros to ensure valid format
    repairZero(child1, vehicle, gen);
    repairCustomerWithLocalSearch(child1, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
    repairZero(child2, vehicle, gen);
    repairCustomerWithLocalSearch(child2, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);

    return {child1, child2};
}

// ======= MUTATION OPERATORS =======

// 1. Swap Mutation - Swaps two random customers
void mutateSwap(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> posDis(0, (int)seq.size()-1);
    int i = posDis(gen);
    int j = posDis(gen);
    int tries = 0;
    
    // Find two non-zero positions to swap
    while ((seq[i] == 0 || seq[j] == 0 || i == j) && tries < 50) {
        i = posDis(gen);
        j = posDis(gen);
        ++tries;
    }
    
    if (i != j && seq[i] != 0 && seq[j] != 0) {
        swap(seq[i], seq[j]);
    }
}

// 2. Inversion Mutation - Reverses a segment of the route
void mutateInversion(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 3) return;
    
    uniform_int_distribution<> posDis(0, (int)seq.size()-1);
    int start = posDis(gen);
    int end = posDis(gen);
    
    if (start > end) swap(start, end);
    if (end - start < 1) return;
    
    // Make sure we don't reverse depot positions
    while (start < seq.size() && seq[start] == 0) start++;
    while (end >= 0 && seq[end] == 0) end--;
    
    if (start < end && start < seq.size() && end >= 0) {
        reverse(seq.begin() + start, seq.begin() + end + 1);
    }
}

// 3. Insertion Mutation - Moves a customer to a different position
void mutateInsertion(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 3) return;
    
    uniform_int_distribution<> posDis(0, (int)seq.size()-1);
    int from = posDis(gen);
    int to = posDis(gen);
    
    // Find non-zero customer to move
    int tries = 0;
    while (seq[from] == 0 && tries < 50) {
        from = posDis(gen);
        tries++;
    }
    
    if (from != to && seq[from] != 0) {
        int customer = seq[from];
        seq.erase(seq.begin() + from);
        
        // Adjust insertion position if needed
        if (to >= seq.size()) to = seq.size();
        seq.insert(seq.begin() + to, customer);
    }
}

// 4. Or-opt Mutation - Moves a segment of 1-3 customers to another position
void mutateOrOpt(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 4) return;
    
    uniform_int_distribution<> segSizeDis(1, 3);
    uniform_int_distribution<> posDis(0, (int)seq.size()-1);
    
    int segSize = segSizeDis(gen);
    int start = posDis(gen);
    int insertPos = posDis(gen);
    
    // Ensure segment doesn't go out of bounds
    if (start + segSize >= seq.size()) return;
    
    // Check if segment contains only customers (no depots)
    bool validSegment = true;
    for (int i = start; i < start + segSize; i++) {
        if (seq[i] == 0) {
            validSegment = false;
            break;
        }
    }
    
    if (validSegment && insertPos != start) {
        // Extract segment
        vector<int> segment(seq.begin() + start, seq.begin() + start + segSize);
        seq.erase(seq.begin() + start, seq.begin() + start + segSize);
        
        // Adjust insertion position
        if (insertPos > start) insertPos -= segSize;
        if (insertPos >= seq.size()) insertPos = seq.size();
        
        // Insert segment at new position
        seq.insert(seq.begin() + insertPos, segment.begin(), segment.end());
    }
}

// 5. Scramble Mutation - Randomly shuffles a segment
void mutateScramble(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 3) return;
    
    uniform_int_distribution<> posDis(0, (int)seq.size()-1);
    int start = posDis(gen);
    int end = posDis(gen);
    
    if (start > end) swap(start, end);
    if (end - start < 1) return;
    
    // Make sure segment contains only customers
    vector<int> customers;
    for (int i = start; i <= end; i++) {
        if (seq[i] != 0) {
            customers.push_back(seq[i]);
        }
    }
    
    if (customers.size() > 1) {
        shuffle(customers.begin(), customers.end(), gen);
        
        // Put scrambled customers back
        int custIdx = 0;
        for (int i = start; i <= end; i++) {
            if (seq[i] != 0) {
                seq[i] = customers[custIdx++];
            }
        }
    }
}

// 6. Route Exchange Mutation - Exchanges segments between different routes
void mutateRouteExchange(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 5) return;
    
    // Find depot positions to identify routes
    vector<int> depotPos;
    for (int i = 0; i < seq.size(); i++) {
        if (seq[i] == 0) {
            depotPos.push_back(i);
        }
    }
    
    if (depotPos.size() < 3) return; // Need at least 2 routes
    
    uniform_int_distribution<> routeDis(0, depotPos.size()-2);
    int route1 = routeDis(gen);
    int route2 = routeDis(gen);
    
    if (route1 == route2) return;
    
    int start1 = depotPos[route1] + 1;
    int end1 = depotPos[route1 + 1] - 1;
    int start2 = depotPos[route2] + 1;
    int end2 = depotPos[route2 + 1] - 1;
    
    if (start1 <= end1 && start2 <= end2) {
        // Exchange single customers from each route
        if (end1 >= start1 && end2 >= start2) {
            uniform_int_distribution<> pos1Dis(start1, end1);
            uniform_int_distribution<> pos2Dis(start2, end2);
            
            int pos1 = pos1Dis(gen);
            int pos2 = pos2Dis(gen);
            
            swap(seq[pos1], seq[pos2]);
        }
    }
}

// Main mutation function with multiple operators
void mutate(vector<int>& seq, int n, int vehicle, const vector<int>& demand, 
           int capacity, mt19937& gen, const vector<vector<double>>& dist, int depot,
           double maxDistance = 0.0, double serviceTime = 0.0) {
    // Adaptive mutation rate based on problem size
    double mutationRate = (n > 100) ? 0.40 : 0.30; // Higher rate for large problems
    uniform_real_distribution<> prob(0.0, 1.0);
    
    if (prob(gen) < mutationRate) {
        if (seq.size() < 2) return;
        
         // Select mutation operator based on probability distribution
        double randValue = prob(gen);
        
        if (randValue < 0.25) {
            // 25% - Swap Mutation
            mutateSwap(seq, gen);
        } else if (randValue < 0.45) {
            // 20% - Inversion Mutation  
            mutateInversion(seq, gen);
        } else if (randValue < 0.60) {
            // 15% - Insertion Mutation
            mutateInsertion(seq, gen);
        } else if (randValue < 0.75) {
            // 15% - Or-opt Mutation
            mutateOrOpt(seq, gen);
        } else if (randValue < 0.90) {
            // 15% - Scramble Mutation
            mutateScramble(seq, gen);
        } else {
            // 10% - Route Exchange Mutation
            mutateRouteExchange(seq, gen);
        }
    }
    
    repairCustomerWithLocalSearch(seq, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
    repairZero(seq, vehicle, gen);
}

// ======= FEASIBILITY TRACKING =======



struct FeasibleSolution {
    vector<int> sequence;
    double cost;
    int generation;
    bool isFeasible;
    
    FeasibleSolution() : cost(numeric_limits<double>::max()), generation(-1), isFeasible(false) {}
    
    FeasibleSolution(const vector<int>& seq, double c, int gen, bool feasible) 
        : sequence(seq), cost(c), generation(gen), isFeasible(feasible) {}
};

// Hàm lấy feasible solution tốt nhất từ một generation
FeasibleSolution getBestFeasibleFromGeneration(const vector<vector<int>>& population,
                                            
                                             const vector<pair<double, int>>& fitnessIndex,
                                             const vector<int>& demand,
                                             int capacity, int depot, int generation,
                                             const vector<vector<double>>& dist = {},
                                             double maxDistance = 0.0, double serviceTime = 0.0) {
    
    FeasibleSolution bestFeasible;
    int feasibleCount = 0;
    
    // Duyệt population theo thứ tự fitness tốt nhất trước
    for (const auto& entry : fitnessIndex) {
        double fitness = entry.first;
        int index = entry.second;
        const vector<int>& individual = population[index];
        
        // Kiểm tra feasibility với cả capacity và distance constraints
        bool isFeasible = validateCapacity(individual, demand, capacity, depot, dist, maxDistance, serviceTime);
        
        if (isFeasible) {
            feasibleCount++;
            // Nếu đây là feasible solution tốt nhất cho đến nay
            if (fitness < bestFeasible.cost) {
                bestFeasible = FeasibleSolution(individual, fitness, generation, true);
            }
        }
    }
    
    // Log thông tin mỗi 10 generations
    if (generation % 10 == 0 || generation == 1) {
        cout << "   Generation " << generation << ": " << feasibleCount << "/" 
             << population.size() << " feasible solutions" << endl;
        
        if (bestFeasible.isFeasible) {
            cout << "   Best feasible cost: " << bestFeasible.cost << endl;
        }
    }
    
    return bestFeasible;
}

// Hàm update global best feasible
void updateGlobalBestFeasible(FeasibleSolution& globalBest, const FeasibleSolution& candidate) {
    if (candidate.isFeasible && candidate.cost < globalBest.cost) {
        globalBest = candidate;
        cout << " NEW GLOBAL BEST FEASIBLE: " << candidate.cost 
             << " (generation " << candidate.generation << ")" << endl;
    }
}

// ======= SOLUTION DISPLAY =======

void displaySolution(const vector<int>& sequence, const vector<pair<double,double>>& coords,
                    const vector<int>& demand, int capacity, int depot, 
                    const vector<vector<double>>& dist, double maxDistance = 0.0, double serviceTime = 0.0) {
    
    if (sequence.empty()) {
        cout << "Empty solution!" << endl;
        return;
    }
    
    vector<vector<int>> routes = decodeSeq(sequence, depot);
    double totalCost = 0;
    int totalVehicles = routes.size();
    bool allFeasible = true;
    
    cout << "Number of vehicles used: " << totalVehicles << endl;
    cout << "Vehicle capacity: " << capacity << endl;
    if (maxDistance > 0.0) {
        cout << "Maximum distance/time per route: " << maxDistance << endl;
    }
    if (serviceTime > 0.0) {
        cout << "Service time per customer: " << serviceTime << endl;
    }
    cout << "\nRoute details:" << endl;
    
    for (size_t i = 0; i < routes.size(); ++i) {
        // Calculate route demand
        int routeDemand = 0;
        for (size_t j = 1; j < routes[i].size() - 1; j++) {
            routeDemand += demand[routes[i][j]];
        }
        
        // Calculate route cost (travel distance)
        double routeCost = 0;
        for (size_t j = 0; j < routes[i].size() - 1; j++) {
            int from = routes[i][j];
            int to = routes[i][j+1];
            routeCost += dist[from][to];
        }
        
        // Calculate route time = travel time + service time
        double routeTime = calculateRouteTime(routes[i], dist, serviceTime);
        
        cout << "Route " << (i + 1) << ": ";
        for (int v : routes[i]) cout << v << " ";
        cout << "| Cost: " << fixed << setprecision(2) << routeCost;
        cout << " | Demand: " << routeDemand << "/" << capacity;
        
        if (maxDistance > 0.0) {
            cout << " | Time: " << fixed << setprecision(2) << routeTime << "/" << maxDistance;
        }
        
        bool routeFeasible = true;
        if (routeDemand > capacity) {
            cout << "  CAPACITY VIOLATION!";
            routeFeasible = false;
        }
        if (maxDistance > 0.0 && routeTime > maxDistance) {
            cout << "  TIME VIOLATION!";
            routeFeasible = false;
        }
        if (routeFeasible) {
            cout << " ✓";
        }
        
        allFeasible = allFeasible && routeFeasible;
        cout << endl;
        
        totalCost += routeCost;
    }
    
    cout << "\n SUMMARY:" << endl;
    cout << "Total cost: " << fixed << setprecision(2) << totalCost << endl;
    cout << "Total vehicles: " << totalVehicles << endl;
    cout << "Solution status: " << (allFeasible ? "FEASIBLE" : "INFEASIBLE") << endl;
    
    if (allFeasible) {
        // Calculate utilization
        int totalDemandServed = 0;
        for (size_t i = 0; i < routes.size(); i++) {
            for (size_t j = 1; j < routes[i].size() - 1; j++) {
                totalDemandServed += demand[routes[i][j]];
            }
        }
        double utilization = (double)totalDemandServed / (totalVehicles * capacity) * 100;
        cout << "Vehicle utilization: " << fixed << setprecision(1) << utilization << "%" << endl;
    }
}

// ======= GENETIC ALGORITHM =======

vector<vector<int>> newGeneration(const vector<vector<int>>& population, int depot, 
                                 const vector<vector<double>>& dist, int n, int vehicle, 
                                 const vector<int>& demand, int capacity, const vector<pair<double,double>>& coords,
                                 double maxDistance = 0.0, double serviceTime = 0.0) {
    
    // Calculate fitness once
    vector<pair<double, int>> fitnessIndex;
    fitnessIndex.reserve(population.size());
    
    for(size_t i = 0; i < population.size(); i++){
        double fitness = calculateFitness(population[i], coords, demand, capacity, depot, dist, maxDistance, serviceTime);
        fitnessIndex.push_back({fitness, (int)i});
    }
    sort(fitnessIndex.begin(), fitnessIndex.end());
    
    vector<vector<int>> newGen;
    int popSize = population.size();
    
    // Calculate number of individuals for each category
    int bestParentCount = max(1, (int)(popSize * 0.15)); // 15% best parents
    int randomParentCount = max(1, (int)(popSize * 0.15)); // 15% random parents
    int childrenCount = popSize - bestParentCount - randomParentCount; // ~70% children
    
    // 1. Keep best parents (15%)
    for (int i = 0; i < bestParentCount && i < (int)fitnessIndex.size(); ++i) {
        newGen.push_back(population[fitnessIndex[i].second]);
    }
    
    // 2. Add random parents (15%)
    vector<int> remainingIndices;
    for (size_t i = bestParentCount; i < fitnessIndex.size(); ++i) {
        remainingIndices.push_back(fitnessIndex[i].second);
    }
    shuffle(remainingIndices.begin(), remainingIndices.end(), mt19937(random_device{}()));
    
    for (int i = 0; i < randomParentCount && i < (int)remainingIndices.size(); ++i) {
        newGen.push_back(population[remainingIndices[i]]);
    }
    
    // 3. Prepare parent pool for crossover (use both best and random parents)
    vector<vector<int>> parentPool;
    // Add all best parents to the pool
    for (int i = 0; i < bestParentCount && i < (int)fitnessIndex.size(); ++i) {
        parentPool.push_back(population[fitnessIndex[i].second]);
    }
    // Add some random parents to ensure diversity
    for (int i = 0; i < min(10, (int)remainingIndices.size()); ++i) {
        parentPool.push_back(population[remainingIndices[i]]);
    }
    
    // 4. Generate children through crossover
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> parentDis(0, max(0, (int)parentPool.size()-1));
    uniform_real_distribution<> crossoverChoice(0.0, 1.0);
    uniform_real_distribution<> mutProb(0.0, 1.0);
    
    int childrenCreated = 0;
    
    while (childrenCreated < childrenCount && parentPool.size() >= 2) {
        // Select two different parents
        int idx1 = parentDis(gen);
        int idx2 = parentDis(gen);
        
        // Ensure parents are different
        int attempts = 0;
        while (idx2 == idx1 && parentPool.size() > 1 && attempts < 10) {
            idx2 = parentDis(gen);
            attempts++;
        }
        
        // Choose crossover operator (3 operators with balanced probabilities)
        pair<vector<int>, vector<int>> childPair;
        double choice = crossoverChoice(gen);
        
        if (choice < 0.33) {
            // 33% - One-Point Crossover
            childPair = crossoverOnePoint(parentPool[idx1], parentPool[idx2], 
                                         n, vehicle, demand, capacity, depot, gen, dist, maxDistance, serviceTime);
        } else if (choice < 0.67) {
            // 34% - Order Crossover (OX)
            childPair = crossoverOX(parentPool[idx1], parentPool[idx2], 
                                   n, vehicle, demand, capacity, depot, gen, dist, maxDistance, serviceTime);
        } else {
            // 33% - Partially Mapped Crossover (PMX)
            childPair = crossoverPMX(parentPool[idx1], parentPool[idx2], 
                                    n, vehicle, demand, capacity, depot, gen, dist, maxDistance, serviceTime);
        }
        
        // Apply mutation with adaptive probability
        double mutationProb = (n > 100) ? 0.30 : 0.20; // Higher for large problems
        if (mutProb(gen) < mutationProb) {
            mutate(childPair.first, n, vehicle, demand, capacity, gen, dist, depot, maxDistance, serviceTime);
        }
        
        if (mutProb(gen) < mutationProb) {
            mutate(childPair.second, n, vehicle, demand, capacity, gen, dist, depot, maxDistance, serviceTime);
        }
        
        // Add children to new population
        newGen.push_back(childPair.first);
        childrenCreated++;
        
        if (childrenCreated < childrenCount) {
            newGen.push_back(childPair.second);
            childrenCreated++;
        }
    }
    
    // In case we couldn't create enough children
    while (newGen.size() < popSize) {
        int randIdx = uniform_int_distribution<>(0, parentPool.size()-1)(gen);
        vector<int> individual = parentPool[randIdx];
        
        // Apply strong mutation to ensure diversity
        mutate(individual, n, vehicle, demand, capacity, gen, dist, depot);
        
        newGen.push_back(individual);
    }
    
    return newGen;
}
double globalBestCost;
FeasibleSolution globalBestFeasible;
struct GAResult {
    int vehiclesUsed;
    double bestCost;
    bool isFeasible;
    vector<int> bestSequence;
};

// ... (các include và code trước đó giữ nguyên)

GAResult runGA(int maxGenerations, int vehicle, int n, int capacity, int depot, 
          const vector<pair<double,double>>& coords, const vector<int>& demand, int populationSize, 
          double maxDistance = 0.0, double serviceTime = 0.0, int runNumber = 1) {
    
    // Loại bỏ khai báo VRPInstance instance vì chỉ dùng cho Tabu
    
    cout << "\n STARTING GENETIC ALGORITHM..." << endl;
    cout << "   Problem: " << n-1 << " customers, " << vehicle << " vehicles, capacity " << capacity << endl;
    cout << "   Running for " << maxGenerations << " generations" << endl;
    cout << "   Population size: " << populationSize << endl;

    vector<vector<double>> dist = buildDist(coords);
    
    // Use the enhanced structured initialization
    vector<vector<int>> population = initStructuredPopulation(populationSize, vehicle, n, capacity, demand, coords, dist, depot, runNumber);
    
    // Repair initial population with run-specific seed
    random_device rd;
    unsigned int seed = rd() + runNumber * 12345;  // Different seed for each run
    mt19937 gen(seed);
    
    cout << "   Run seed: " << seed << " (run #" << runNumber << ")" << endl;
    
    for (auto& seq : population) {
        repairCustomerWithLocalSearch(seq, n, gen, dist, demand, capacity, depot, maxDistance, serviceTime, vehicle);
        repairZero(seq, vehicle, gen);
    }
    
    // Initialize tracking variables
    double globalBestCost = numeric_limits<double>::max();
    vector<int> globalBestCostIndividual;
    FeasibleSolution globalBestFeasible;
    int stagnationCount = 0;  // Giữ lại để theo dõi stagnation, nhưng không dùng cho Tabu
    cout << "\nEVOLUTION PROGRESS:" << endl;
    
    for (int generation = 1; generation <= maxGenerations; generation++) {
        // Calculate fitness
        vector<pair<double, int>> fitnessIndex;
        for (size_t i = 0; i < population.size(); ++i) {
            double fitness = calculateFitness(population[i], coords, demand, capacity, depot, dist, maxDistance, serviceTime);
            fitnessIndex.push_back({fitness, (int)i});
        }
        sort(fitnessIndex.begin(), fitnessIndex.end());
        
        // Get best solution in this generation
        double bestCostInGen = fitnessIndex[0].first;
        int bestIdxInGen = fitnessIndex[0].second;
        
        // Update global best (may be infeasible)
        if (bestCostInGen < globalBestCost) {
            globalBestCost = bestCostInGen;
            globalBestCostIndividual = population[bestIdxInGen];
            stagnationCount = 0;
            
            cout << "Generation " << generation << ": New global best cost = " << bestCostInGen;
            if (validateCapacity(globalBestCostIndividual, demand, capacity, depot, dist, maxDistance, serviceTime)) {
                cout << " FEASIBLE";
            } else {
                cout << " INFEASIBLE";
            }
            cout << endl;
        } else {
            stagnationCount++;
            if (generation % 10 == 0) {
                cout << "Generation " << generation << ": Best = " << bestCostInGen 
                     << ", Global = " << globalBestCost << ", No improvement for " 
                     << stagnationCount << " generations" << endl;
            }
        }
        
        // Update best feasible solution
        FeasibleSolution bestFeasibleInGen = getBestFeasibleFromGeneration(
            population, fitnessIndex, demand, capacity, depot, generation, dist, maxDistance, serviceTime);
        updateGlobalBestFeasible(globalBestFeasible, bestFeasibleInGen);
        
        // Create next generation
        if (generation < maxGenerations) {
            population = newGeneration(population, depot, dist, n, vehicle, demand, capacity, coords, maxDistance, serviceTime);
        }
    }
    
    // Final results (giữ nguyên)
    cout << "\n==== FINAL RESULTS ====" << endl;
    cout << "Best solution cost: " << globalBestCost;
    bool globalBestIsFeasible = validateCapacity(globalBestCostIndividual, demand, capacity, depot, dist, maxDistance, serviceTime);
    cout << (globalBestIsFeasible ? " FEASIBLE" : "  INFEASIBLE") << endl;
    
    GAResult result;
    
    if (globalBestFeasible.isFeasible) {
        vector<vector<int>> routes = decodeSeq(globalBestFeasible.sequence, depot);
        result.vehiclesUsed = routes.size();
        result.bestCost = globalBestFeasible.cost;
        result.isFeasible = true;
        result.bestSequence = globalBestFeasible.sequence;
        
        cout << "\n BEST FEASIBLE SOLUTION:" << endl;
        cout << "Cost: " << globalBestFeasible.cost << endl;
        cout << "Found in generation: " << globalBestFeasible.generation << endl;
        
        // Display the routes
        displaySolution(globalBestFeasible.sequence, coords, demand, capacity, depot, dist, maxDistance, serviceTime);
    } else {
        vector<vector<int>> routes = decodeSeq(globalBestCostIndividual, depot);
        result.vehiclesUsed = routes.size();
        result.bestCost = globalBestCost;
        result.isFeasible = false;
        result.bestSequence = globalBestCostIndividual;
        
        cout << "\n NO FEASIBLE SOLUTION FOUND!" << endl;
        cout << "All solutions violated capacity constraints." << endl;
        
        // Display best infeasible solution as fallback
        cout << "\n Best infeasible solution:" << endl;
        displaySolution(globalBestCostIndividual, coords, demand, capacity, depot, dist, maxDistance, serviceTime);
    }
    
    return result;
}

// ... (phần còn lại của file giữ nguyên)
#include <fstream>

// Add this function to export results to CSV
// Function to extract optimal cost from VRP file comment
double extractOptimalCost(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Warning: Cannot open " << filename << " to read optimal cost" << endl;
        return -1.0;  // Indicate error
    }
    
    string line;
    while (getline(file, line)) {
        if (line.find("COMMENT :") != string::npos) {
            // Extract the number after "COMMENT :"
            size_t pos = line.find(":");
            if (pos != string::npos) {
                string costStr = line.substr(pos + 1);
                // Remove leading/trailing spaces
                costStr.erase(0, costStr.find_first_not_of(" \t"));
                costStr.erase(costStr.find_last_not_of(" \t") + 1);
                
                try {
                    return stod(costStr);
                } catch (const exception& e) {
                    cout << "Warning: Cannot parse optimal cost from: " << costStr << endl;
                    return -1.0;
                }
            }
        }
        // Stop reading after finding NODE_COORD_SECTION
        if (line.find("NODE_COORD_SECTION") != string::npos) {
            break;
        }
    }
    
    cout << "Warning: Optimal cost not found in " << filename << endl;
    return -1.0;
}

void exportToCSV(const string& instanceName, int totalVehicles, int populationSize, int maxGenerations, double bestCost, double optimalCost) {
    ofstream outFile("ga_results.csv", ios::app);  // Append mode
    
    // Check if file is empty, if so, add header
    ifstream checkFile("ga_results.csv");
    bool isEmpty = checkFile.peek() == ifstream::traits_type::eof();
    checkFile.close();
    
    if (isEmpty) {
        outFile << "Instance,Total_Vehicles,Population_Size,Max_Generations,Best_Cost,Optimal_Cost,GAP_Percent" << endl;
    }
    
    // Calculate GAP percentage: (bestCost - optimalCost) / optimalCost * 100
    double gap = -1.0;  // Default value for missing optimal cost
    if (optimalCost > 0) {
        gap = ((bestCost - optimalCost) / optimalCost) * 100.0;
    }
    
    // Write data row with fixed precision for cost
    outFile << instanceName << "," 
            << totalVehicles << "," 
            << populationSize << "," 
            << maxGenerations << ","
            << fixed << setprecision(2) << bestCost << ","
            << fixed << setprecision(2) << optimalCost << ","
            << fixed << setprecision(2) << gap << endl;
            
    outFile.close();
    
    cout << "\n=== RESULTS EXPORTED TO ga_results.csv ===" << endl;
    cout << "Instance: " << instanceName << endl;
    cout << "Best cost: " << fixed << setprecision(2) << bestCost << endl;
    cout << "Optimal cost: " << fixed << setprecision(2) << optimalCost << endl;
    if (gap >= 0) {
        cout << "GAP: " << fixed << setprecision(2) << gap << "%" << endl;
    } else {
        cout << "GAP: N/A (optimal cost not available)" << endl;
    }
}


// ======= MAIN FUNCTION =======

int main(int argc, char* argv[]) {
    cout << "🚛 CVRP SOLVER with GENETIC ALGORITHM" << endl;
    cout << "====================================" << endl;
    
    // Default parameters
    string filename = "CMT4.vrp";  
    int maxGenerations = 1000;     
    int populationSize = 800;
    int numRuns = 10;  // Number of times to run each instance
    
    // Parse command line arguments
    if (argc >= 2) {
        filename = argv[1];
        cout << " Input file: " << filename << endl;
    } else {
        cout << "Usage: " << argv[0] << " <VRP_FILE> [GENERATIONS] [POPULATION_SIZE] [NUM_RUNS]" << endl;
        cout << "Using default parameters..." << endl;
    }
    
    if (argc >= 3) {
        maxGenerations = atoi(argv[2]);
        if (maxGenerations <= 0) maxGenerations = 1000;
    }
    
    if (argc >= 4) {
        populationSize = atoi(argv[3]);
        if (populationSize <= 0) populationSize = 800;
    }
    
    if (argc >= 5) {
        numRuns = atoi(argv[4]);
        if (numRuns <= 0) numRuns = 1;
    }
    
    // Read VRP problem
    int n, capacity, depot, vehicles;
    double maxDistance, serviceTime;
    vector<pair<double,double>> coords;
    vector<int> demand;
    
    cout << "Reading problem file: " << filename << endl;
    readCVRP(filename, n, capacity, coords, demand, depot, vehicles, maxDistance, serviceTime);
    
    // Extract optimal cost from file
    double optimalCost = extractOptimalCost(filename);
    
    cout << "\nProblem Configuration:" << endl;
    cout << "   Customers: " << n-1 << endl;
    cout << "   Capacity: " << capacity << endl;
    cout << "   Depot: " << depot << endl;
    cout << "   Vehicles: " << vehicles << endl;
    cout << "   Optimal cost: " << (optimalCost > 0 ? to_string(optimalCost) : "Unknown") << endl;
    
    // Calculate total demand
    int totalDemand = 0;
    for (int i = 2; i <= n; i++) {
        totalDemand += demand[i];
    }
    cout << "   Total demand: " << totalDemand << endl;
    cout << "   Min vehicles needed: " << ceil(totalDemand / (double)capacity) << endl;
    
    cout << "\nAlgorithm Parameters:" << endl;
    cout << "   Generations: " << maxGenerations << endl;
    cout << "   Population size: " << populationSize << endl;
    cout << "   Number of runs: " << numRuns << endl;
    
    // Extract instance name without extension and path
    string instanceName = filename;
    size_t lastSlash = instanceName.find_last_of("/\\");
    if (lastSlash != string::npos) {
        instanceName = instanceName.substr(lastSlash + 1);
    }
    size_t dotPos = instanceName.find_last_of('.');
    if (dotPos != string::npos) {
        instanceName = instanceName.substr(0, dotPos);
    }
    
    // Variables for statistics
    vector<double> allCosts;
    vector<int> allVehicles;
    vector<bool> allFeasible;
    int feasibleCount = 0;
    
    cout << "\n" << string(60, '=') << endl;
    cout << "🏃 EXECUTING " << numRuns << " RUN" << (numRuns > 1 ? "S" : "") << endl;
    cout << string(60, '=') << endl;
    
    // Run GA multiple times
    for (int run = 1; run <= numRuns; ++run) {
        cout << "\nRUN " << run << "/" << numRuns;
        if (numRuns > 1) {
            cout << " (" << fixed << setprecision(1) << (100.0 * run / numRuns) << "% completed)";
        }
        cout << endl;
        cout << string(40, '-') << endl;
        
        auto start = chrono::high_resolution_clock::now();
        GAResult result = runGA(maxGenerations, vehicles, n, capacity, depot, coords, demand, populationSize, maxDistance, serviceTime, run);
        auto end = chrono::high_resolution_clock::now();
        
        auto duration = chrono::duration_cast<chrono::seconds>(end - start);
        
        cout << " Run " << run << " completed in " << duration.count() << "s" << endl;
        cout << "   Cost: " << fixed << setprecision(2) << result.bestCost << endl;
        cout << "   Vehicles: " << result.vehiclesUsed << endl;
        cout << "   Status: " << (result.isFeasible ? "FEASIBLE" : " INFEASIBLE") << endl;
        
        allCosts.push_back(result.bestCost);
        allVehicles.push_back(result.vehiclesUsed);
        allFeasible.push_back(result.isFeasible);
        
        if (result.isFeasible) {
            feasibleCount++;
        }
    }
    
    // Calculate and display statistics
    if (!allCosts.empty()) {
        double sumCost = accumulate(allCosts.begin(), allCosts.end(), 0.0);
        double meanCost = sumCost / allCosts.size();
        
        double sumVehicles = accumulate(allVehicles.begin(), allVehicles.end(), 0.0);
        double meanVehicles = sumVehicles / allVehicles.size();
        
        double minCost = *min_element(allCosts.begin(), allCosts.end());
        double maxCost = *max_element(allCosts.begin(), allCosts.end());
        
        int minVehicles = *min_element(allVehicles.begin(), allVehicles.end());
        int maxVehicles = *max_element(allVehicles.begin(), allVehicles.end());
        
        // Calculate standard deviation
        double variance = 0.0;
        for (double cost : allCosts) {
            variance += (cost - meanCost) * (cost - meanCost);
        }
        double stdDev = sqrt(variance / allCosts.size());
        
        // Display comprehensive summary
        cout << "\n" << string(70, '=') << endl;
        cout << " STATISTICAL SUMMARY (" << numRuns << " runs)" << endl;
        cout << string(70, '=') << endl;
        cout << "Instance: " << instanceName << endl;
        cout << "Success rate: " << feasibleCount << "/" << numRuns 
             << " (" << fixed << setprecision(1) << (100.0 * feasibleCount / numRuns) << "%)" << endl;
        
        cout << "\nCOST ANALYSIS:" << endl;
        cout << "   Best cost:    " << fixed << setprecision(2) << minCost;
        if (optimalCost > 0) {
            double bestGap = ((minCost - optimalCost) / optimalCost) * 100.0;
            cout << " (GAP: " << fixed << setprecision(2) << bestGap << "%)";
        }
        cout << endl;
        cout << "   Mean cost:    " << fixed << setprecision(2) << meanCost << endl;
        cout << "   Worst cost:   " << fixed << setprecision(2) << maxCost << endl;
        cout << "   Std dev:      " << fixed << setprecision(2) << stdDev << endl;
        if (optimalCost > 0) {
            cout << "   Optimal cost: " << fixed << setprecision(2) << optimalCost << endl;
        }
        
        cout << "\n🚛 VEHICLE USAGE:" << endl;
        cout << "   Best vehicles:  " << minVehicles << endl;
        cout << "   Mean vehicles:  " << fixed << setprecision(1) << meanVehicles << endl;
        cout << "   Max vehicles:   " << maxVehicles << endl;
        
        // Export best result to CSV with GAP calculation
        exportToCSV(instanceName, minVehicles, populationSize, maxGenerations, minCost, optimalCost);
        
        cout << "\nBEST SOLUTION FOUND:" << endl;
        auto bestIndex = min_element(allCosts.begin(), allCosts.end()) - allCosts.begin();
        cout << "   Run #" << (bestIndex + 1) << ": Cost = " << minCost 
             << ", Vehicles = " << allVehicles[bestIndex] 
             << ", Status = " << (allFeasible[bestIndex] ? "FEASIBLE" : "INFEASIBLE") << endl;
        
    } else {
        cout << "\n ERROR: No valid results obtained!" << endl;
        return 1;
    }
    
    cout << "\n Execution completed successfully!" << endl;
    return 0;
}
