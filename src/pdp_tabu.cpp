#include "pdp_tabu.h"
#include "pdp_ga.h"
#include "pdp_fitness.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace {

struct Move {
    enum Type { RELOCATE, SWAP, OR_OPT } type;
    int customer_i = -1;     // first customer (RELOCATE/SWAP/OR_OPT head)
    int from_route_i = -1;
    int from_pos_i = -1;     // start position of chain
    int to_route_i = -1;
    int to_pos_i = -1;
    int chain_len = 1;       // 1=RELOCATE, 2=Or-opt-2, 3=Or-opt-3

    int customer_j = -1;     // SWAP only
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
    } else if (move.type == Move::SWAP) {
        if (move.from_route_i < 0 || move.from_route_i >= (int)routes.size() ||
            move.from_pos_i < 0 || move.from_pos_i >= (int)routes[move.from_route_i].size() ||
            move.from_route_j < 0 || move.from_route_j >= (int)routes.size() ||
            move.from_pos_j < 0 || move.from_pos_j >= (int)routes[move.from_route_j].size()) {
            return baseSol;
        }
        swap(routes[move.from_route_i][move.from_pos_i], routes[move.from_route_j][move.from_pos_j]);
    } else { // OR_OPT: relocate a chain of chain_len consecutive customers
        const int r1  = move.from_route_i;
        const int r2  = move.to_route_i;
        const int p1  = move.from_pos_i;
        const int len = move.chain_len;
        const int p2  = move.to_pos_i;

        if (r1 < 0 || r1 >= (int)routes.size() ||
            r2 < 0 || r2 >= (int)routes.size() ||
            p1 < 0 || p1 + len > (int)routes[r1].size()) {
            return baseSol;
        }

        // Extract chain
        vector<int> chain(routes[r1].begin() + p1, routes[r1].begin() + p1 + len);
        routes[r1].erase(routes[r1].begin() + p1, routes[r1].begin() + p1 + len);

        // Adjust insertion position when inserting into same route after extraction
        int adjP2 = p2;
        if (r1 == r2 && p2 > p1) adjP2 = max(0, p2 - len);
        adjP2 = max(0, min(adjP2, (int)routes[r2].size()));

        routes[r2].insert(routes[r2].begin() + adjP2, chain.begin(), chain.end());
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

// lastTruckIdx: index of the bottleneck truck (-1 = random/no focus).
// Paper insight: relocate customers FROM the last truck to reduce C_max.
// Strategy: 80% of moves source from lastTruck; 20% fully random for diversification.
vector<Move> generateCandidateMoves(const SolutionEncoding& baseSol,
                                    int maxCandidates,
                                    mt19937& rng,
                                    int lastTruckIdx) {
    vector<Move> candidates;
    vector<vector<int>> routes = extractCustomerRoutes(baseSol);

    int numTrucks = (int)routes.size();
    if (numTrucks == 0) return candidates;

    int totalCustomers = 0;
    for (const auto& r : routes) totalCustomers += r.size();
    if (totalCustomers == 0) return candidates;

    // Validate lastTruckIdx
    if (lastTruckIdx < 0 || lastTruckIdx >= numTrucks || routes[lastTruckIdx].empty())
        lastTruckIdx = -1;  // fall back to random

    unordered_set<string> seen;
    int maxAttempts = maxCandidates * 6;
    int attempts = 0;

    uniform_int_distribution<int> truckDist(0, numTrucks - 1);
    uniform_real_distribution<double> probDist(0.0, 1.0);
    // Type distribution: 50% RELOCATE, 20% OR_OPT-2, 10% OR_OPT-3, 20% SWAP
    uniform_int_distribution<int> typeDist(0, 9);

    while ((int)candidates.size() < maxCandidates && attempts < maxAttempts) {
        attempts++;
        const int t = typeDist(rng);
        // 0-4 → RELOCATE(1), 5-6 → OR_OPT-2, 7 → OR_OPT-3, 8-9 → SWAP
        const int chainLen = (t <= 4) ? 1 : (t <= 6) ? 2 : (t == 7) ? 3 : 0;
        const bool isSwap  = (t >= 8);

        if (!isSwap) { // RELOCATE or OR_OPT
            // Pick source truck: 80% from lastTruck, 20% random
            int r1;
            if (lastTruckIdx >= 0 && probDist(rng) < 0.8) {
                r1 = lastTruckIdx;
            } else {
                r1 = truckDist(rng);
            }
            if ((int)routes[r1].size() < chainLen) continue;

            // Pick starting position (chain must fit)
            uniform_int_distribution<int> pos1Dist(0, (int)routes[r1].size() - chainLen);
            int p1 = pos1Dist(rng);
            int headCust = routes[r1][p1];

            int r2 = truckDist(rng);
            uniform_int_distribution<int> pos2Dist(0, (int)routes[r2].size());
            int p2 = pos2Dist(rng);

            // Skip trivial same-position moves
            if (r1 == r2) {
                bool overlaps = (p2 >= p1 && p2 <= p1 + chainLen);
                if (overlaps) continue;
            }

            string key = (chainLen == 1 ? "R:" : (chainLen == 2 ? "O2:" : "O3:"))
                         + to_string(headCust) + ":" + to_string(r2) + ":" + to_string(p2);
            if (seen.count(key)) continue;
            seen.insert(key);

            Move mv;
            mv.type      = (chainLen == 1) ? Move::RELOCATE : Move::OR_OPT;
            mv.chain_len = chainLen;
            mv.customer_i  = headCust;
            mv.from_route_i = r1;
            mv.from_pos_i   = p1;
            mv.to_route_i   = r2;
            mv.to_pos_i     = p2;
            candidates.push_back(mv);

        } else { // SWAP
            // Pick r1: 80% from lastTruck
            int r1;
            if (lastTruckIdx >= 0 && probDist(rng) < 0.8) {
                r1 = lastTruckIdx;
            } else {
                r1 = truckDist(rng);
            }
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

            const int minC = min(cust1, cust2);
            const int maxC = max(cust1, cust2);
            string key = "S:" + to_string(minC) + ":" + to_string(maxC);
            if (seen.count(key)) continue;
            seen.insert(key);

            Move mv;
            mv.type = Move::SWAP;
            mv.customer_i   = cust1;
            mv.from_route_i = r1;
            mv.from_pos_i   = p1;
            mv.to_route_i   = r2;
            mv.to_pos_i     = p2;
            mv.customer_j   = cust2;
            mv.from_route_j = r2;
            mv.from_pos_j   = p2;
            candidates.push_back(mv);
        }
    }

    return candidates;
}

// Find the truck index with the highest completion time (C_max bottleneck).
int findLastTruckIndex(const SolutionEncoding& sol, const PDPData& data) {
    PDPSolution decoded = decode_solution(sol, data, false);
    int lastTruck = 0;
    double maxTime = -1.0;
    for (int k = 0; k < (int)decoded.truck_details.size(); ++k) {
        if (decoded.truck_details[k].completion_time > maxTime) {
            maxTime = decoded.truck_details[k].completion_time;
            lastTruck = k;
        }
    }
    return lastTruck;
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

    // Tabu tenure: θ = round(7.5 × log10(n)) as in paper
    const int n = data.numCustomers;
    const int paperTenure = max(2, (int)round(7.5 * log10(max(n, 2))));
    if (tabuTenure == 7) tabuTenure = paperTenure;  // override default with paper formula

    for (int step = 0; step < maxIterations; ++step) {
        // Recompute LastTruck (bottleneck) every iteration for accurate focusing
        int lastTruck = findLastTruckIndex(current, data);
        vector<Move> neighborhood = generateCandidateMoves(current, maxCandidates, rng, lastTruck);
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
            } else if (mv.type == Move::OR_OPT) {
                // Tabu if any customer in the chain is forbidden from the destination route
                vector<vector<int>> tmpRoutes = extractCustomerRoutes(current);
                for (int c = 0; c < mv.chain_len && !isTabu; ++c) {
                    int cust = tmpRoutes[mv.from_route_i][mv.from_pos_i + c];
                    string key = to_string(cust) + "," + to_string(mv.to_route_i);
                    auto it = tabuList.find(key);
                    if (it != tabuList.end() && it->second > step) isTabu = true;
                }
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
            // Forbid moving customer back to source route
            string reverseKey = to_string(bestMove.customer_i) + "," + to_string(bestMove.from_route_i);
            tabuList[reverseKey] = step + tenure;
        } else if (bestMove.type == Move::OR_OPT) {
            // Forbid each customer in the chain from returning to source route
            vector<vector<int>> tmpRoutes = extractCustomerRoutes(current);
            for (int c = 0; c < bestMove.chain_len; ++c) {
                int idx = bestMove.from_pos_i + c;
                if (idx < (int)tmpRoutes[bestMove.from_route_i].size()) {
                    int cust = tmpRoutes[bestMove.from_route_i][idx];
                    string reverseKey = to_string(cust) + "," + to_string(bestMove.from_route_i);
                    tabuList[reverseKey] = step + tenure;
                }
            }
        } else { // SWAP
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
