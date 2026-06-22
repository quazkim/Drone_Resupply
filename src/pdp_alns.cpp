#include "pdp_alns.h"
#include "pdp_lb.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>

using namespace std;

// ============================================================
// ALNSWeights
// ============================================================

static int rouletteSelect(const double* w, int n, mt19937& rng) {
    double total = 0.0;
    for (int i = 0; i < n; ++i) total += w[i];
    uniform_real_distribution<double> dist(0.0, total);
    double r = dist(rng);
    double cum = 0.0;
    for (int i = 0; i < n; ++i) {
        cum += w[i];
        if (r <= cum) return i;
    }
    return n - 1;
}

int ALNSWeights::selectDestroy(mt19937& rng) const {
    return rouletteSelect(destroy_w, 3, rng);
}
int ALNSWeights::selectRepair(mt19937& rng) const {
    return rouletteSelect(repair_w, 2, rng);
}

// score: 0 = no improvement, 1 = improvement, 2 = new global best
static const double SCORE_MULT[3] = {0.5, 1.0, 2.0};

void ALNSWeights::updateDestroy(int op, int score, double rho) {
    destroy_w[op] = (1 - rho) * destroy_w[op] + rho * SCORE_MULT[score];
    destroy_w[op] = max(0.01, destroy_w[op]);
}
void ALNSWeights::updateRepair(int op, int score, double rho) {
    repair_w[op] = (1 - rho) * repair_w[op] + rho * SCORE_MULT[score];
    repair_w[op] = max(0.01, repair_w[op]);
}

// ============================================================
// Helpers
// ============================================================

// Collect all customers in a VRPSolution (flat list)
static vector<int> allCustomers(const VRPSolution& vrp) {
    vector<int> c;
    for (const auto& route : vrp) {
        for (int n : route) c.push_back(n);
    }
    return c;
}

// Check if a customer is a C2 pickup (type "P")
static bool isP(const PDPData& d, int n) {
    if (n <= 0 || n >= d.numNodes) return false;
    return d.nodeTypes[n] == "P";
}
static bool isDL(const PDPData& d, int n) {
    if (n <= 0 || n >= d.numNodes) return false;
    return d.nodeTypes[n] == "DL";
}

// Find partner of a C2 node (P→DL or DL→P)
static int c2Partner(const PDPData& d, int n) {
    if (n <= 0 || n >= d.numNodes) return -1;
    bool p_node  = (d.nodeTypes[n] == "P");
    bool dl_node = (d.nodeTypes[n] == "DL");
    if (!p_node && !dl_node) return -1;
    for (int m = 1; m < d.numNodes; ++m) {
        if (m == n) continue;
        if (d.pairIds[m] != d.pairIds[n]) continue;
        if (p_node  && d.nodeTypes[m] == "DL") return m;
        if (dl_node && d.nodeTypes[m] == "P")  return m;
    }
    return -1;
}

// Remove a set of customers from routes (handles C2 pairs: remove both P and DL together)
// Returns: partial routes + actual removed customers
static DestroyResult removeFromRoutes(const PDPData& data,
                                       const VRPSolution& vrp,
                                       const vector<int>& to_remove_raw) {
    // Expand C2: if we remove P, also remove its DL, and vice versa
    vector<bool> remove_flag(data.numNodes, false);
    for (int n : to_remove_raw) {
        remove_flag[n] = true;
        int partner = c2Partner(data, n);
        if (partner > 0) remove_flag[partner] = true;
    }

    DestroyResult dr;
    dr.partial.resize(vrp.size());
    for (int v = 0; v < static_cast<int>(vrp.size()); ++v) {
        for (int n : vrp[v]) {
            if (remove_flag[n]) {
                dr.removed.push_back(n);
            } else {
                dr.partial[v].push_back(n);
            }
        }
    }
    // De-duplicate removed list
    sort(dr.removed.begin(), dr.removed.end());
    dr.removed.erase(unique(dr.removed.begin(), dr.removed.end()), dr.removed.end());
    return dr;
}

// ============================================================
// Destroy operators
// ============================================================

DestroyResult destroyRandomSegment(const PDPData& data,
                                   const VRPSolution& vrp,
                                   int segment_len,
                                   mt19937& rng) {
    // Pick a random non-empty truck, then remove a random contiguous segment
    vector<int> non_empty;
    for (int v = 0; v < static_cast<int>(vrp.size()); ++v) {
        if (!vrp[v].empty()) non_empty.push_back(v);
    }
    if (non_empty.empty()) return {vrp, {}};

    uniform_int_distribution<int> vdist(0, (int)non_empty.size() - 1);
    int v = non_empty[vdist(rng)];
    const auto& route = vrp[v];
    int len = min(segment_len, (int)route.size());
    if (len <= 0) return {vrp, {}};

    uniform_int_distribution<int> kdist(0, (int)route.size() - len);
    int start = kdist(rng);

    vector<int> to_remove;
    for (int k = start; k < start + len; ++k) {
        to_remove.push_back(route[k]);
    }
    return removeFromRoutes(data, vrp, to_remove);
}

DestroyResult destroyWorstRemoval(const PDPData& data,
                                  const VRPSolution& vrp,
                                  int n_remove,
                                  mt19937& rng) {
    // Compute "savings" if each customer is removed: lb_reduction or travel cost reduction
    // Use: removal cost = travel savings in route (detour cost of visiting that customer)
    struct RemovalCost { int customer; double cost; };
    vector<RemovalCost> costs;

    for (int v = 0; v < static_cast<int>(vrp.size()); ++v) {
        const auto& route = vrp[v];
        for (int k = 0; k < static_cast<int>(route.size()); ++k) {
            int n = route[k];
            if (isP(data, n) || isDL(data, n)) continue; // handle C2 as pair

            // Detour cost: t(prev, n) + t(n, next) - t(prev, next)
            int prev = (k > 0) ? route[k-1] : 0;
            int next = (k + 1 < (int)route.size()) ? route[k+1] : 0;
            double detour = data.truckDistMatrix[prev][n]
                          + data.truckDistMatrix[n][next]
                          - data.truckDistMatrix[prev][next];
            costs.push_back({n, detour});
        }
    }

    // Sort descending by cost (worst = highest detour → remove first)
    sort(costs.begin(), costs.end(), [](const RemovalCost& a, const RemovalCost& b){
        return a.cost > b.cost;
    });

    // Add randomization (roulette among top 3*n_remove)
    int pool = min((int)costs.size(), 3 * n_remove);
    uniform_int_distribution<int> sdist(0, max(1, pool - 1));

    vector<int> to_remove;
    vector<bool> used(data.numNodes, false);
    for (int i = 0; i < n_remove && !costs.empty(); ) {
        int idx = sdist(rng) % costs.size();
        int n = costs[idx].cost > 0 ? costs[idx].customer : costs[0].customer;
        // Just pick from top
        n = costs[min(idx, (int)costs.size()-1)].customer;
        if (!used[n]) {
            to_remove.push_back(n);
            used[n] = true;
            ++i;
        }
        costs.erase(costs.begin() + min(idx, (int)costs.size()-1));
        if (costs.empty()) break;
    }

    return removeFromRoutes(data, vrp, to_remove);
}

DestroyResult destroyRelatedRemoval(const PDPData& data,
                                    const VRPSolution& vrp,
                                    int n_remove,
                                    mt19937& rng) {
    // Start from a random customer, then remove most "related" (similar) customers
    // Relatedness: proximity in distance
    vector<int> all = allCustomers(vrp);
    if (all.empty()) return {vrp, {}};

    uniform_int_distribution<int> adist(0, (int)all.size() - 1);
    int seed = all[adist(rng)];

    vector<pair<double, int>> sorted_by_dist;
    for (int n : all) {
        if (n == seed) continue;
        double d = data.truckDistMatrix[seed][n];
        sorted_by_dist.push_back({d, n});
    }
    sort(sorted_by_dist.begin(), sorted_by_dist.end());

    vector<int> to_remove = {seed};
    for (int i = 0; i < n_remove - 1 && i < (int)sorted_by_dist.size(); ++i) {
        to_remove.push_back(sorted_by_dist[i].second);
    }

    return removeFromRoutes(data, vrp, to_remove);
}

// ============================================================
// Repair helpers
// ============================================================

// Cost of inserting customer n at position k in route v (after position k-1, before k)
static double insertionCost(const PDPData& data, const VRPSolution& partial,
                            int v, int k, int n) {
    const auto& route = partial[v];
    int prev = (k > 0) ? route[k-1] : 0;
    int next = (k < (int)route.size()) ? route[k] : 0;
    return data.truckDistMatrix[prev][n] + data.truckDistMatrix[n][next]
           - data.truckDistMatrix[prev][next];
}

struct InsertPos { int truck, pos; double cost; };

// Find best insert position for customer n, respecting C2 constraints
// If n is "P", its partner "DL" is assumed to follow it (inserted after)
static InsertPos findBestInsert(const PDPData& data, VRPSolution& partial, int n) {
    InsertPos best{-1, -1, 1e18};

    bool is_p = isP(data, n);
    int partner = c2Partner(data, n);

    for (int v = 0; v < static_cast<int>(partial.size()); ++v) {
        const auto& route = partial[v];
        int m = (int)route.size();

        for (int k = 0; k <= m; ++k) {
            // Check that inserting here doesn't violate C2 precedence:
            // If n is DL, its P must already be in route[v][0..k-1]
            if (isDL(data, n) && partner > 0) {
                bool p_before = false;
                for (int j = 0; j < k; ++j) {
                    if (route[j] == partner) { p_before = true; break; }
                }
                if (!p_before) continue;
            }

            double c = insertionCost(data, partial, v, k, n);
            if (is_p && partner > 0) {
                // Also consider inserting partner right after n
                // Temporary insert n at k, then find best insert for partner after k
                double c_partner = 1e18;
                for (int j = k + 1; j <= m + 1; ++j) {
                    // Check partner's C2 constraint (P=n must be at position k < j)
                    // We're inserting n at k so P is before j ✓
                    // Cost of inserting partner at j (in the route with n inserted at k)
                    // Approximate: treat as if route already has n at k
                    int prev_p = (j > k + 1) ? (j - 1 <= m ? route[j - 2] : n) : n;
                    int next_p = (j <= m) ? route[j - 1] : 0;
                    (void)prev_p; (void)next_p;
                    // For simplicity: use original route indices shifted by 1 after k
                    int prev_j = (j == k + 1) ? n : (j - 2 < (int)route.size() ? route[j-2] : n);
                    int next_j = (j - 1 < (int)route.size()) ? route[j-1] : 0;
                    double cp = data.truckDistMatrix[prev_j][partner]
                               + data.truckDistMatrix[partner][next_j]
                               - data.truckDistMatrix[prev_j][next_j];
                    c_partner = min(c_partner, cp);
                }
                c += c_partner;
            }

            if (c < best.cost) {
                best = {v, k, c};
            }
        }
    }
    return best;
}

// Insert n (and partner if C2-P) into partial at their best positions
static void insertCustomer(const PDPData& data, VRPSolution& partial, int n) {
    bool is_p  = isP(data, n);
    bool is_dl = isDL(data, n);
    int partner = c2Partner(data, n);

    if (is_dl) {
        // Partner P must already be in the route (inserted first), skip if not
        // Search for it:
        bool found_p = false;
        for (auto& route : partial) {
            for (int x : route) {
                if (x == partner) { found_p = true; break; }
            }
            if (found_p) break;
        }
        if (!found_p) return; // will be inserted when P is processed
    }

    InsertPos best = findBestInsert(data, partial, n);
    if (best.truck < 0) return;

    int v = best.truck;
    int k = best.pos;
    partial[v].insert(partial[v].begin() + k, n);

    if (is_p && partner > 0) {
        // Insert partner DL after n (best position after k)
        InsertPos bp2 = findBestInsert(data, partial, partner);
        if (bp2.truck >= 0) {
            partial[bp2.truck].insert(partial[bp2.truck].begin() + bp2.pos, partner);
        }
    }
}

// ============================================================
// Repair operators
// ============================================================

VRPSolution repairGreedyInsert(const PDPData& data,
                                const DestroyResult& dr,
                                mt19937& rng) {
    VRPSolution partial = dr.partial;
    vector<int> removed = dr.removed;

    // Process C1 first, then C2-P (which will pull in DL), skip standalone DL
    vector<int> order;
    for (int n : removed) {
        if (!isDL(data, n)) order.push_back(n);
    }
    for (int n : removed) {
        if (isDL(data, n)) {
            // Check if P already in order
            int p = c2Partner(data, n);
            bool p_in_order = false;
            for (int x : order) if (x == p) { p_in_order = true; break; }
            if (!p_in_order) order.push_back(n);
        }
    }

    // Shuffle with slight randomization
    shuffle(order.begin(), order.end(), rng);
    // But re-sort so P comes before DL
    stable_sort(order.begin(), order.end(), [&](int a, int b){
        return isP(data, a) > isP(data, b); // P first
    });

    for (int n : order) {
        insertCustomer(data, partial, n);
    }

    return partial;
}

VRPSolution repairRegret2Insert(const PDPData& data,
                                 const DestroyResult& dr,
                                 mt19937& rng) {
    VRPSolution partial = dr.partial;
    vector<int> uninserted;
    for (int n : dr.removed) {
        if (!isDL(data, n)) uninserted.push_back(n);
    }
    // Add standalone DL (without corresponding P already in uninserted)
    for (int n : dr.removed) {
        if (isDL(data, n)) {
            int p = c2Partner(data, n);
            bool p_in_ui = false;
            for (int x : uninserted) if (x == p) { p_in_ui = true; break; }
            if (!p_in_ui) uninserted.push_back(n);
        }
    }

    while (!uninserted.empty()) {
        // Compute regret-2: difference between best and second-best insertion cost
        int best_i = 0;
        double best_regret = -1e18;

        for (int i = 0; i < static_cast<int>(uninserted.size()); ++i) {
            int n = uninserted[i];
            double c1 = 1e18, c2 = 1e18;

            for (int v = 0; v < static_cast<int>(partial.size()); ++v) {
                for (int k = 0; k <= static_cast<int>(partial[v].size()); ++k) {
                    double c = insertionCost(data, partial, v, k, n);
                    if (c < c1) { c2 = c1; c1 = c; }
                    else if (c < c2) { c2 = c; }
                }
            }

            double regret = c2 - c1;
            if (regret > best_regret) {
                best_regret = regret;
                best_i = i;
            }
        }

        int n = uninserted[best_i];
        uninserted.erase(uninserted.begin() + best_i);
        insertCustomer(data, partial, n);
    }

    (void)rng;
    return partial;
}

// ============================================================
// alnsIteration
// ============================================================

VRPSolution alnsIteration(const PDPData& data,
                           const VRPSolution& vrp,
                           ALNSWeights& /*weights*/,
                           int destroy_op, int repair_op,
                           int n_remove,
                           mt19937& rng) {
    DestroyResult dr;
    switch (destroy_op) {
        case 0: dr = destroyRandomSegment(data, vrp, n_remove, rng); break;
        case 1: dr = destroyWorstRemoval(data, vrp, n_remove, rng); break;
        case 2: dr = destroyRelatedRemoval(data, vrp, n_remove, rng); break;
        default: dr = destroyRandomSegment(data, vrp, n_remove, rng); break;
    }

    VRPSolution repaired;
    switch (repair_op) {
        case 0: repaired = repairGreedyInsert(data, dr, rng); break;
        case 1: repaired = repairRegret2Insert(data, dr, rng); break;
        default: repaired = repairGreedyInsert(data, dr, rng); break;
    }

    return repaired;
}
