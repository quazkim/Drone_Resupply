#include "pdp_alns.h"

#include "pdp_fitness.h"
#include "pdp_ga.h"      // extractCustomerRoutes, repairC2Pairs
#include "pdp_init.h"    // buildEncodingForRouting
#include "pdp_tabu.h"    // tabuSearchPDP
#include "pdp_utils.h"   // computeLowerBound

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <unordered_set>

using namespace std;

namespace {

using Routes = vector<vector<int>>;

// An atomic routing unit: a C1 customer [c], or a C2 pair [pickup, delivery].
struct Item {
    vector<int> nodes;
};

static double truckT(const PDPData& d, int a, int b) {
    if (a < 0 || a >= d.numNodes || b < 0 || b >= d.numNodes) return 1e18;
    return d.truckDistMatrix[a][b];
}

static double droneOrEuclid(const PDPData& d, int a, int b) {
    if (a < 0 || a >= d.numNodes || b < 0 || b >= d.numNodes) return 1e18;
    return d.droneDistMatrix[a][b];
}

// Build the atomic items from the instance (C2 pickup+delivery kept together).
static vector<Item> buildItems(const PDPData& d) {
    vector<Item> items;
    for (int i = 1; i < d.numNodes; ++i) {
        if (i >= (int)d.nodeTypes.size()) break;
        const string& t = d.nodeTypes[i];
        if (t == "DL") continue;  // attached to its pickup
        if (t == "P") {
            int dl = (i < (int)d.deliveryOfPickup.size()) ? d.deliveryOfPickup[i] : -1;
            if (dl > 0) items.push_back(Item{{i, dl}});
            else        items.push_back(Item{{i}});
        } else {
            items.push_back(Item{{i}});  // "D" = C1 direct delivery
        }
    }
    return items;
}

// Remove all nodes of an item from whatever routes contain them.
static void removeItemNodes(Routes& routes, const Item& it) {
    for (int node : it.nodes) {
        for (auto& r : routes) {
            auto p = find(r.begin(), r.end(), node);
            if (p != r.end()) { r.erase(p); break; }
        }
    }
}

// --- Makespan-aware insertion criterion (Pina §4.3.1) ---
// a_i = r_i + min(d_i + δm, t_0i) + s_i  : earliest an order can be "at" customer i.
static double aVal(const PDPData& d, int node) {
    const int depot = d.depotIndex;
    double ri  = (node < (int)d.readyTimes.size()) ? (double)d.readyTimes[node] : 0.0;
    double di  = droneOrEuclid(d, depot, node) + d.resupplyTime;  // d_i + δm
    double t0i = truckT(d, depot, node);
    double si  = d.truckServiceTime;
    return ri + min(di, t0i) + si;
}

// ℓ(χ^v): lower bound on a truck's completion time for an ordered customer list.
static double computeRouteLB(const vector<int>& r, const PDPData& d) {
    if (r.empty()) return 0.0;
    const int depot = d.depotIndex;
    double l = max(aVal(d, r[0]), truckT(d, depot, r[0]));
    for (int i = 1; i < (int)r.size(); ++i)
        l = max(aVal(d, r[i]), l + truckT(d, r[i - 1], r[i]));
    l += truckT(d, r.back(), depot);
    return l;
}

// ℓ of a route after inserting item nodes at gap g.
static double routeLBwithInsert(const vector<int>& r, int gap, const Item& it, const PDPData& d) {
    vector<int> tmp;
    tmp.reserve(r.size() + it.nodes.size());
    tmp.insert(tmp.end(), r.begin(), r.begin() + gap);
    tmp.insert(tmp.end(), it.nodes.begin(), it.nodes.end());
    tmp.insert(tmp.end(), r.begin() + gap, r.end());
    return computeRouteLB(tmp, d);
}

// Insert item into route rIdx at gap g (pair inserted as adjacent block).
static void insertItemAt(Routes& routes, int rIdx, int gap, const Item& it) {
    auto& r = routes[rIdx];
    r.insert(r.begin() + gap, it.nodes.begin(), it.nodes.end());
}

// Scan all (route, gap): cost = resulting global LB = max(ℓ of others, ℓ of modified route).
// Returns best & second-best costs (for regret) and the best placement.
struct InsScan { double d1; double d2; int route; int gap; };
static InsScan scanInsertions(const Routes& routes, const Item& it, const PDPData& data) {
    const int K = (int)routes.size();
    vector<double> Lr(K);
    for (int k = 0; k < K; ++k) Lr[k] = computeRouteLB(routes[k], data);

    InsScan s{1e18, 1e18, 0, 0};
    for (int rk = 0; rk < K; ++rk) {
        double otherMax = 0.0;
        for (int k = 0; k < K; ++k) if (k != rk) otherMax = max(otherMax, Lr[k]);
        const int len = (int)routes[rk].size();
        for (int g = 0; g <= len; ++g) {
            double cand = max(otherMax, routeLBwithInsert(routes[rk], g, it, data));
            if (cand < s.d1) { s.d2 = s.d1; s.d1 = cand; s.route = rk; s.gap = g; }
            else if (cand < s.d2) { s.d2 = cand; }
        }
    }
    return s;
}

// ---------------- DESTROY operators ----------------

static vector<Item> destroyRandom(Routes& routes, const vector<Item>& items, int q, mt19937& g) {
    vector<int> idx(items.size());
    iota(idx.begin(), idx.end(), 0);
    shuffle(idx.begin(), idx.end(), g);
    vector<Item> removed;
    for (int i = 0; i < q && i < (int)idx.size(); ++i) {
        removed.push_back(items[idx[i]]);
        removeItemNodes(routes, items[idx[i]]);
    }
    return removed;
}

static double itemDetour(const Routes& routes, const Item& it, const PDPData& data) {
    double total = 0.0;
    const int depot = data.depotIndex;
    for (int node : it.nodes) {
        for (const auto& r : routes) {
            auto p = find(r.begin(), r.end(), node);
            if (p == r.end()) continue;
            int pos = (int)(p - r.begin());
            int prev = (pos > 0) ? r[pos - 1] : depot;
            int next = (pos + 1 < (int)r.size()) ? r[pos + 1] : depot;
            total += truckT(data, prev, node) + truckT(data, node, next) - truckT(data, prev, next);
            break;
        }
    }
    return total;
}

// Worst removal: remove items with the largest detour (with randomization).
static vector<Item> destroyWorst(Routes& routes, const vector<Item>& items, int q,
                                 const PDPData& data, mt19937& g) {
    vector<pair<double,int>> scored;
    scored.reserve(items.size());
    for (int i = 0; i < (int)items.size(); ++i)
        scored.push_back({itemDetour(routes, items[i], data), i});
    sort(scored.begin(), scored.end(), [](const pair<double,int>& a, const pair<double,int>& b){
        return a.first > b.first;
    });
    uniform_real_distribution<double> U(0.0, 1.0);
    vector<Item> removed;
    vector<char> taken(items.size(), 0);
    int picked = 0;
    while (picked < q && picked < (int)scored.size()) {
        double r = U(g);
        int pos = (int)(r * r * r * scored.size());  // bias toward worst
        if (pos >= (int)scored.size()) pos = (int)scored.size() - 1;
        // advance to next untaken
        while (taken[pos]) pos = (pos + 1) % scored.size();
        taken[pos] = 1;
        const Item& it = items[scored[pos].second];
        removed.push_back(it);
        removeItemNodes(routes, it);
        picked++;
    }
    return removed;
}

// Shaw / related removal: remove items related (distance + release time) to a seed.
static vector<Item> destroyShaw(Routes& routes, const vector<Item>& items, int q,
                                const PDPData& data, mt19937& g) {
    if (items.empty()) return {};
    uniform_int_distribution<int> pick(0, (int)items.size() - 1);
    int seed = pick(g);
    const int sNode = items[seed].nodes[0];
    const double sRel = (sNode < (int)data.readyTimes.size()) ? data.readyTimes[sNode] : 0.0;

    vector<pair<double,int>> rel;
    for (int i = 0; i < (int)items.size(); ++i) {
        if (i == seed) continue;
        int n = items[i].nodes[0];
        double rj = (n < (int)data.readyTimes.size()) ? data.readyTimes[n] : 0.0;
        double relatedness = droneOrEuclid(data, sNode, n) + 0.5 * fabs(sRel - rj);
        rel.push_back({relatedness, i});
    }
    sort(rel.begin(), rel.end());

    vector<Item> removed;
    removed.push_back(items[seed]);
    removeItemNodes(routes, items[seed]);
    for (int i = 0; i < q - 1 && i < (int)rel.size(); ++i) {
        const Item& it = items[rel[i].second];
        removed.push_back(it);
        removeItemNodes(routes, it);
    }
    return removed;
}

// Route removal: empty one non-empty truck, returning its items.
static vector<Item> destroyRoute(Routes& routes, const vector<Item>& items,
                                 const PDPData& data, mt19937& g) {
    vector<int> nonEmpty;
    for (int k = 0; k < (int)routes.size(); ++k)
        if (!routes[k].empty()) nonEmpty.push_back(k);
    if (nonEmpty.empty()) return {};
    uniform_int_distribution<int> pick(0, (int)nonEmpty.size() - 1);
    int rk = nonEmpty[pick(g)];

    unordered_set<int> nodesInRoute(routes[rk].begin(), routes[rk].end());
    vector<Item> removed;
    for (const auto& it : items) {
        bool any = false;
        for (int n : it.nodes) if (nodesInRoute.count(n)) { any = true; break; }
        if (any) { removed.push_back(it); removeItemNodes(routes, it); }
    }
    return removed;
}

// ---------------- REPAIR operators ----------------

static void repairGreedy(Routes& routes, vector<Item>& removed, const PDPData& data, mt19937& g) {
    shuffle(removed.begin(), removed.end(), g);
    for (const auto& it : removed) {
        InsScan b = scanInsertions(routes, it, data);  // minimize resulting makespan LB
        insertItemAt(routes, b.route, b.gap, it);
    }
}

static void repairRegret2(Routes& routes, vector<Item>& removed, const PDPData& data, mt19937& g) {
    (void)g;
    vector<Item> pool = removed;
    while (!pool.empty()) {
        int bestIdx = -1;
        double bestRegret = -1e18;
        InsScan bestPlace{1e18, 1e18, 0, 0};
        for (int i = 0; i < (int)pool.size(); ++i) {
            InsScan s = scanInsertions(routes, pool[i], data);
            double regret = (s.d2 >= 1e17 ? 0.0 : s.d2 - s.d1);
            // Tie-break by smallest best-cost so we still favour cheap insertions.
            if (regret > bestRegret + 1e-9 ||
                (fabs(regret - bestRegret) <= 1e-9 && s.d1 < bestPlace.d1)) {
                bestRegret = regret; bestIdx = i; bestPlace = s;
            }
        }
        if (bestIdx < 0) break;
        insertItemAt(routes, bestPlace.route, bestPlace.gap, pool[bestIdx]);
        pool.erase(pool.begin() + bestIdx);
    }
}

// ---------------- Evaluation ----------------

static double evalRoutes(const Routes& routesIn, const PDPData& data,
                         SolutionEncoding& outEnc, PDPSolution& outSol) {
    Routes routes = routesIn;
    repairC2Pairs(routes, data);  // safety: ensure pickup precedes delivery
    outEnc = buildEncodingForRouting(routes, data);
    outSol = decode_solution(outEnc, data, false);
    return outSol.totalCost + outSol.totalPenalty;
}

// Adaptive operator selection (roulette wheel).
static int rouletteSelect(const vector<double>& w, mt19937& g) {
    double sum = 0.0;
    for (double x : w) sum += x;
    uniform_real_distribution<double> U(0.0, sum);
    double r = U(g);
    double acc = 0.0;
    for (int i = 0; i < (int)w.size(); ++i) {
        acc += w[i];
        if (r <= acc) return i;
    }
    return (int)w.size() - 1;
}

} // namespace

PDPSolution alnsSearchPDP(const PDPData& data,
                          const SolutionEncoding& initial,
                          int maxIterations,
                          double timeLimitSeconds,
                          int runNumber,
                          std::ostream* logStream) {
    const auto t0 = chrono::high_resolution_clock::now();
    const double lowerBound = computeLowerBound(data);

    mt19937 gen((unsigned)(runNumber * 2654435761u + 1u));

    vector<Item> items = buildItems(data);
    const int nItems = (int)items.size();

    // --- Initial solution ---
    Routes current = extractCustomerRoutes(initial);
    if ((int)current.size() < data.numTrucks) current.resize(data.numTrucks);

    SolutionEncoding curEnc, bestEnc;
    PDPSolution curSol, bestSol;
    double curCost = evalRoutes(current, data, curEnc, curSol);

    Routes best = current;
    bestEnc = curEnc; bestSol = curSol;
    double bestCost = curCost;

    // --- Operator pools + adaptive weights ---
    // destroy: 0=random 1=worst 2=shaw 3=route ; repair: 0=greedy 1=regret2
    vector<double> wd(4, 1.0), wr(2, 1.0);
    vector<double> sd(4, 0.0), sr(2, 0.0);
    vector<int> ud(4, 0), ur(2, 0);
    const double rho = 0.2;
    const int segment = 50;     // iterations per weight update
    const double rBest = 10.0, rImpr = 5.0, rAcc = 2.0;

    // --- Simulated annealing acceptance ---
    double T = max(1.0, 0.10 * max(1.0, curCost));
    const double cooling = 0.9985;
    const double Tmin = 1e-2;

    // --- Intensification ---
    const int intensifyEvery = 100;

    cout << "\n[ALNS] items=" << nItems
         << " trucks=" << data.numTrucks
         << " drones=" << data.numDrones
         << " LB=" << fixed << setprecision(2) << lowerBound
         << (timeLimitSeconds > 0 ? ("  TimeLimit=" + to_string((int)timeLimitSeconds) + "s") : "")
         << "\n[ALNS] init Cmax=" << curCost
         << " feasible=" << (curSol.isFeasible ? "YES" : "NO") << "\n";
    cout.flush();

    uniform_real_distribution<double> U01(0.0, 1.0);

    int iter = 0;
    int sinceImprove = 0;
    for (iter = 0; iter < maxIterations; ++iter) {
        // pick destroy size
        int qmax = max(2, (int)round(0.35 * nItems));
        qmax = min(qmax, max(1, nItems));
        uniform_int_distribution<int> qD(1, qmax);
        int q = qD(gen);

        int di = rouletteSelect(wd, gen);
        int ri = rouletteSelect(wr, gen);
        ud[di]++; ur[ri]++;

        Routes cand = current;
        vector<Item> removed;
        switch (di) {
            case 0: removed = destroyRandom(cand, items, q, gen); break;
            case 1: removed = destroyWorst(cand, items, q, data, gen); break;
            case 2: removed = destroyShaw(cand, items, q, data, gen); break;
            default: removed = destroyRoute(cand, items, data, gen); break;
        }
        if (ri == 0) repairGreedy(cand, removed, data, gen);
        else         repairRegret2(cand, removed, data, gen);

        SolutionEncoding candEnc; PDPSolution candSol;
        double candCost = evalRoutes(cand, data, candEnc, candSol);

        double reward = 0.0;
        bool accept = false;
        if (candCost < curCost - 1e-9) {
            accept = true;
            reward = rImpr;
        } else {
            double delta = candCost - curCost;
            if (U01(gen) < exp(-delta / max(Tmin, T))) { accept = true; reward = rAcc; }
        }

        if (accept) {
            current.swap(cand);
            curCost = candCost; curEnc = candEnc; curSol = candSol;
        }

        if (candCost < bestCost - 1e-9) {
            best = (accept ? current : cand);
            bestCost = candCost; bestEnc = candEnc; bestSol = candSol;
            reward = rBest;
            sinceImprove = 0;
        } else {
            sinceImprove++;
        }

        sd[di] += reward; sr[ri] += reward;

        // cooling
        T = max(Tmin, T * cooling);

        // adaptive weight update
        if ((iter + 1) % segment == 0) {
            for (int i = 0; i < (int)wd.size(); ++i) {
                if (ud[i] > 0) wd[i] = (1 - rho) * wd[i] + rho * (sd[i] / ud[i]);
                sd[i] = 0; ud[i] = 0;
            }
            for (int i = 0; i < (int)wr.size(); ++i) {
                if (ur[i] > 0) wr[i] = (1 - rho) * wr[i] + rho * (sr[i] / ur[i]);
                sr[i] = 0; ur[i] = 0;
            }
        }

        // TẦNG 2 - Tabu Search intensification (on incumbent / on new best)
        bool doIntensify = (reward == rBest) || ((iter + 1) % intensifyEvery == 0);
        if (doIntensify) {
            SolutionEncoding refined = tabuSearchPDP(bestEnc, data, 60, bestCost, 7);
            PDPSolution rSol = decode_solution(refined, data, false);
            double rCost = rSol.totalCost + rSol.totalPenalty;
            if (rCost < bestCost - 1e-9) {
                bestCost = rCost; bestEnc = refined; bestSol = rSol;
                best = extractCustomerRoutes(refined);
                // also steer the search there
                current = best; curEnc = refined; curSol = rSol; curCost = rCost;
                sinceImprove = 0;
            }
        }

        // diversification: restart from best after long stagnation
        if (sinceImprove > 0 && sinceImprove % (intensifyEvery * 4) == 0) {
            current = best; curEnc = bestEnc; curSol = bestSol; curCost = bestCost;
            T = max(T, 0.10 * max(1.0, bestCost));  // reheat
        }

        if (logStream && (iter % 50 == 0)) {
            (*logStream) << "[ALNS] iter=" << iter
                         << " bestCmax=" << bestCost
                         << " curCmax=" << curCost
                         << " T=" << T
                         << " feasible=" << (bestSol.isFeasible ? 1 : 0) << "\n";
        }

        // early stop on lower bound
        if (bestSol.isFeasible && bestCost <= lowerBound + 0.5) {
            cout << "[ALNS] *** LB reached *** Cmax=" << bestCost
                 << " LB=" << lowerBound << " iter=" << iter << "\n";
            break;
        }

        // time limit
        if (timeLimitSeconds > 0.0) {
            double el = chrono::duration<double>(
                chrono::high_resolution_clock::now() - t0).count();
            if (el >= timeLimitSeconds) {
                cout << "[ALNS] *** TIME LIMIT *** elapsed=" << fixed << setprecision(1) << el
                     << "s iter=" << iter << " Cmax=" << setprecision(2) << bestCost << "\n";
                break;
            }
        }
    }

    double el = chrono::duration<double>(
        chrono::high_resolution_clock::now() - t0).count();
    cout << "[ALNS] done iters=" << iter
         << " bestCmax=" << fixed << setprecision(2) << bestCost
         << " feasible=" << (bestSol.isFeasible ? "YES" : "NO")
         << " elapsed=" << setprecision(1) << el << "s\n";
    cout.flush();

    bestSol.encoded_routes = bestEnc;
    return bestSol;
}
