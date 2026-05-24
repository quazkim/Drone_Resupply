#include "pdp_tabu.h"
#include "pdp_ga.h"
#include "pdp_fitness.h"
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <algorithm>
#include <iostream>
#include <climits>

using namespace std;

namespace {

struct Move {
    enum Type { RELOCATE, SWAP } type;
    int customer_i = -1;
    int from_route_i = -1;
    int from_pos_i = -1;
    int to_route_i = -1;
    int to_pos_i = -1;
    
    int customer_j = -1;
    int from_route_j = -1;
    int from_pos_j = -1;
};

double getPkgWeight(const PDPData& data, int p) {
    if (p <= 0 || p >= (int)data.demands.size()) return 0.0;
    return (double)data.demands[p];
}

double getFitness(const SolutionEncoding& sol, const PDPData& data) {
    PDPSolution decoded = decode_solution(sol, data, false);
    return decoded.totalCost + decoded.totalPenalty;
}

SolutionEncoding applyMove(const SolutionEncoding& baseSol, const Move& move, const PDPData& data) {
    // 1. Extract customer routes
    vector<vector<int>> routes = extractCustomerRoutes(baseSol);
    
    // 2. Apply move
    if (move.type == Move::RELOCATE) {
        int from_route = move.from_route_i;
        int to_route = move.to_route_i;
        int from_pos = move.from_pos_i;
        int to_pos = move.to_pos_i;
        
        if (from_route < 0 || from_route >= (int)routes.size() ||
            to_route < 0 || to_route >= (int)routes.size() ||
            from_pos < 0 || from_pos >= (int)routes[from_route].size()) {
            return baseSol;
        }
        
        int val = routes[from_route][from_pos];
        routes[from_route].erase(routes[from_route].begin() + from_pos);
        
        int adjusted_to_pos = to_pos;
        if (from_route == to_route && to_pos > from_pos) {
            adjusted_to_pos--;
        }
        
        if (adjusted_to_pos < 0 || adjusted_to_pos > (int)routes[to_route].size()) {
            return baseSol;
        }
        routes[to_route].insert(routes[to_route].begin() + adjusted_to_pos, val);
    } else { // SWAP
        if (move.from_route_i < 0 || move.from_route_i >= (int)routes.size() ||
            move.from_pos_i < 0 || move.from_pos_i >= (int)routes[move.from_route_i].size() ||
            move.from_route_j < 0 || move.from_route_j >= (int)routes.size() ||
            move.from_pos_j < 0 || move.from_pos_j >= (int)routes[move.from_route_j].size()) {
            return baseSol;
        }
        swap(routes[move.from_route_i][move.from_pos_i], routes[move.from_route_j][move.from_pos_j]);
    }
    
    // Repair C2 pairing on the customer routes before rebuilding empty routes
    repairC2Pairs(routes, data);

    // 3. Initialize empty encoded routes
    SolutionEncoding child = initializeEmptyEncodedRoutes(routes);
    
    // 4. Extract and inherit resupply candidates from baseSol
    vector<Trip> candidates = extractResupplyTrips(baseSol, 1);
    
    // Sort candidates: prefer larger trips first
    sort(candidates.begin(), candidates.end(), [](const Trip& a, const Trip& b) {
        return a.pkgs.size() > b.pkgs.size();
    });
    
    unordered_set<int> supplied;
    auto childIndex = buildCustomerIndex(child);
    const double QD = (double)data.getDroneCapacity();

    for (const auto& trip : candidates) {
        vector<int> valid = getValidPackageSubset(trip, childIndex, supplied);
        // Exclude C2 Pickup/Delivery packages from drone resupply
        vector<int> filteredValid;
        for (int p : valid) {
            if (p > 0 && p < (int)data.nodeTypes.size() && (data.nodeTypes[p] == "P" || data.nodeTypes[p] == "DL")) {
                continue;
            }
            filteredValid.push_back(p);
        }
        valid.swap(filteredValid);
        if (valid.empty()) continue;

        double load = 0.0;
        for (int p : valid) load += getPkgWeight(data, p);
        if (load > QD + 1e-9) continue;

        auto itH = childIndex.find(trip.h);
        if (itH == childIndex.end()) continue;
        Route& r = child[itH->second.route_id];

        for (auto& st : r) {
            if (st.node == trip.h) {
                st.packages.insert(st.packages.end(), valid.begin(), valid.end());
                break;
            }
        }
        for (int p : valid) supplied.insert(p);
    }
    
    // 5. Repair
    repairSolution(child, data);
    
    return child;
}

vector<Move> generateCandidateMoves(const SolutionEncoding& baseSol, int maxCandidates, mt19937& rng) {
    vector<Move> candidates;
    vector<vector<int>> routes = extractCustomerRoutes(baseSol);
    
    int numTrucks = (int)routes.size();
    if (numTrucks == 0) return candidates;
    
    int totalCustomers = 0;
    for (const auto& r : routes) totalCustomers += r.size();
    if (totalCustomers == 0) return candidates;
    
    unordered_set<string> seen;
    int maxAttempts = maxCandidates * 5;
    int attempts = 0;
    
    uniform_int_distribution<int> typeDist(0, 1); // 0 = Relocate, 1 = Swap
    uniform_int_distribution<int> truckDist(0, numTrucks - 1);
    
    while ((int)candidates.size() < maxCandidates && attempts < maxAttempts) {
        attempts++;
        int mType = typeDist(rng);
        
        if (mType == 0) { // RELOCATE
            int r1 = truckDist(rng);
            if (routes[r1].empty()) continue;
            
            uniform_int_distribution<int> pos1Dist(0, (int)routes[r1].size() - 1);
            int p1 = pos1Dist(rng);
            int cust = routes[r1][p1];
            
            int r2 = truckDist(rng);
            uniform_int_distribution<int> pos2Dist(0, (int)routes[r2].size());
            int p2 = pos2Dist(rng);
            
            if (r1 == r2 && (p2 == p1 || p2 == p1 + 1)) continue;
            
            string key = "R:" + to_string(cust) + ":" + to_string(r2) + ":" + to_string(p2);
            if (seen.count(key)) continue;
            seen.insert(key);
            
            Move mv;
            mv.type = Move::RELOCATE;
            mv.customer_i = cust;
            mv.from_route_i = r1;
            mv.from_pos_i = p1;
            mv.to_route_i = r2;
            mv.to_pos_i = p2;
            candidates.push_back(mv);
            
        } else { // SWAP
            int r1 = truckDist(rng);
            if (routes[r1].empty()) continue;
            
            uniform_int_distribution<int> pos1Dist(0, (int)routes[r1].size() - 1);
            int p1 = pos1Dist(rng);
            int cust1 = routes[r1][p1];
            
            int r2 = truckDist(rng);
            if (routes[r2].empty()) continue;
            
            uniform_int_distribution<int> pos2Dist(0, (int)routes[r2].size() - 1);
            int p2 = pos2Dist(rng);
            int cust2 = routes[r2][p2];
            
            if (r1 == r2 && p1 == p2) continue;
            
            int minCust = min(cust1, cust2);
            int maxCust = max(cust1, cust2);
            string key = "S:" + to_string(minCust) + ":" + to_string(maxCust);
            if (seen.count(key)) continue;
            seen.insert(key);
            
            Move mv;
            mv.type = Move::SWAP;
            mv.customer_i = cust1;
            mv.from_route_i = r1;
            mv.from_pos_i = p1;
            mv.to_route_i = r2;
            mv.to_pos_i = p2;
            
            mv.customer_j = cust2;
            mv.from_route_j = r2;
            mv.from_pos_j = p2;
            
            candidates.push_back(mv);
        }
    }
    
    return candidates;
}

} // namespace

TabuSearchPDP::TabuSearchPDP(const PDPData& data, int maxIterations, double globalBestFitness, int tabuTenure)
    : data(data), maxIterations(maxIterations), globalBestFitness(globalBestFitness), tabuTenure(tabuTenure) {}

SolutionEncoding TabuSearchPDP::run(const SolutionEncoding& initial) {
    SolutionEncoding current = initial;
    SolutionEncoding bestLocal = initial;
    
    double fitnessCurrent = getFitness(current, data);
    double fitnessBestLocal = fitnessCurrent;
    
    // Tabu list maps "customer,route" -> step_until_tabu_expires
    unordered_map<string, int> tabuList;
    
    mt19937 rng(42);
    int maxCandidates = 150;
    
    for (int step = 0; step < maxIterations; ++step) {
        vector<Move> neighborhood = generateCandidateMoves(current, maxCandidates, rng);
        if (neighborhood.empty()) break;
        
        SolutionEncoding bestCandidate;
        double fitnessBestCandidate = numeric_limits<double>::infinity();
        Move bestMove;
        bool foundValidCandidate = false;
        
        for (const auto& mv : neighborhood) {
            SolutionEncoding neighbor = applyMove(current, mv, data);
            double fitnessNeighbor = getFitness(neighbor, data);
            
            bool isTabu = false;
            if (mv.type == Move::RELOCATE) {
                string key = to_string(mv.customer_i) + "," + to_string(mv.to_route_i);
                auto it = tabuList.find(key);
                isTabu = (it != tabuList.end() && it->second > step);
            } else { // SWAP
                string key_i = to_string(mv.customer_i) + "," + to_string(mv.to_route_i);
                string key_j = to_string(mv.customer_j) + "," + to_string(mv.from_route_i);
                
                auto it_i = tabuList.find(key_i);
                bool tabu_i = (it_i != tabuList.end() && it_i->second > step);
                
                auto it_j = tabuList.find(key_j);
                bool tabu_j = (it_j != tabuList.end() && it_j->second > step);
                
                isTabu = tabu_i || tabu_j;
            }
            
            bool isAspiration = (fitnessNeighbor < globalBestFitness - 1e-5);
            
            if (!isTabu || isAspiration) {
                if (!foundValidCandidate || fitnessNeighbor < fitnessBestCandidate) {
                    bestCandidate = neighbor;
                    fitnessBestCandidate = fitnessNeighbor;
                    bestMove = mv;
                    foundValidCandidate = true;
                }
            }
        }
        
        if (!foundValidCandidate) {
            break;
        }
        
        current = bestCandidate;
        fitnessCurrent = fitnessBestCandidate;
        
        if (fitnessCurrent < fitnessBestLocal - 1e-5) {
            bestLocal = current;
            fitnessBestLocal = fitnessCurrent;
        }
        
        // Randomize L in [5,10] if tabuTenure matches this range (default 7)
        int tenure = tabuTenure;
        if (tabuTenure >= 5 && tabuTenure <= 10) {
            uniform_int_distribution<int> tenureDist(5, 10);
            tenure = tenureDist(rng);
        }
        
        if (bestMove.type == Move::RELOCATE) {
            string reverseKey = to_string(bestMove.customer_i) + "," + to_string(bestMove.from_route_i);
            tabuList[reverseKey] = step + tenure;
        } else {
            string reverseKey_i = to_string(bestMove.customer_i) + "," + to_string(bestMove.from_route_i);
            string reverseKey_j = to_string(bestMove.customer_j) + "," + to_string(bestMove.from_route_j);
            tabuList[reverseKey_i] = step + tenure;
            tabuList[reverseKey_j] = step + tenure;
        }
    }
    
    return bestLocal;
}

SolutionEncoding tabuSearchPDP(const SolutionEncoding& initial,
                               const PDPData& data,
                               int maxIterations,
                               double globalBestFitness,
                               int tabuTenure) {
    TabuSearchPDP ts(data, maxIterations, globalBestFitness, tabuTenure);
    return ts.run(initial);
}
