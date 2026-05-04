#include "pdp_ga.h"
#include "pdp_tabu.h"
#include "pdp_types.h"
#include "pdp_fitness.h"
#include "pdp_cache.h"
#include "pdp_init.h"
#include <algorithm>
#include <random>
#include <map>
#include <set>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
#include <climits>
#include <numeric>
#include <unordered_set>

using namespace std;

// ============ ADAPTIVE PARAMETERS ============
struct AdaptiveParams {
    // Crossover success rates
    vector<double> crossoverSuccess;
    vector<int> crossoverUsage;
    vector<double> crossoverRates;
    
    // Mutation success rates
    vector<double> mutationSuccess;
    vector<int> mutationUsage;
    vector<double> mutationRates;
    
    // Adaptive mutation rate
    double currentMutationRate;
    double baseMutationRate;
    int noImprovementCount;
    
    AdaptiveParams(double baseRate) : 
        crossoverSuccess(4, 0.0), crossoverUsage(4, 0), crossoverRates(4, 0.25),
        mutationSuccess(5, 0.0), mutationUsage(5, 0), mutationRates(5, 0.2),
        currentMutationRate(baseRate), baseMutationRate(baseRate), noImprovementCount(0) {}
    
    
    void updateCrossoverSuccess(int type, bool improved) {
        crossoverUsage[type]++;
        if (improved) crossoverSuccess[type]++;
    }
 
    void updateMutationSuccess(int type, bool improved) {
        mutationUsage[type]++;
        if (improved) mutationSuccess[type]++;
    }
    
    
    void adaptCrossoverRates() {
        double totalSuccess = 0;
        for (int i = 0; i < 4; ++i) {
            if (crossoverUsage[i] > 0) {
                totalSuccess += crossoverSuccess[i] / crossoverUsage[i];
            }
        }
        
        if (totalSuccess > 0) {
            for (int i = 0; i < 4; ++i) {
                if (crossoverUsage[i] > 0) {
                    double successRate = crossoverSuccess[i] / crossoverUsage[i];
                    crossoverRates[i] = 0.1 + 0.8 * (successRate / totalSuccess);
                } else {
                    crossoverRates[i] = 0.25;
                }
            }
        }
    }
    
    
    void adaptMutationRates() {
        double totalSuccess = 0;
        for (int i = 0; i < 5; ++i) {
            if (mutationUsage[i] > 0) {
                totalSuccess += mutationSuccess[i] / mutationUsage[i];
            }
        }
        
        if (totalSuccess > 0) {
            for (int i = 0; i < 5; ++i) {
                if (mutationUsage[i] > 0) {
                    double successRate = mutationSuccess[i] / mutationUsage[i];
                    mutationRates[i] = 0.05 + 0.9 * (successRate / totalSuccess);
                } else {
                    mutationRates[i] = 0.2;
                }
            }
        }
    }
   
    void adaptMutationRate() {
        if (noImprovementCount > 5) {
            currentMutationRate = min(0.5, currentMutationRate * 1.2);
        } else if (noImprovementCount < 2) {
            currentMutationRate = max(baseMutationRate * 0.5, currentMutationRate * 0.9);
        }
    }
    
    
    int selectCrossoverType(mt19937& gen) {
        discrete_distribution<> dist(crossoverRates.begin(), crossoverRates.end());
        return dist(gen);
    }
    
 
    int selectMutationType(mt19937& gen) {
        discrete_distribution<> dist(mutationRates.begin(), mutationRates.end());
        return dist(gen);
    }
};

struct OnlineSurrogate {
    int n = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;

    void update(double x, double y) {
        n++;
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
    }

    bool ready() const {
        return n >= 30;
    }

    double predict(double x, double fallback) const {
        if (n < 2) return fallback;
        double denom = n * sumXX - sumX * sumX;
        if (fabs(denom) < 1e-9) return fallback;
        double a = (n * sumXY - sumX * sumY) / denom;
        double b = (sumY - a * sumX) / n;
        double y = a * x + b;
        if (!isfinite(y)) return fallback;
        return y;
    }
};

static double edgeNoveltyScore(const vector<int>& seq, const vector<int>& reference) {
    if (seq.size() < 2 || reference.size() < 2 || seq.size() != reference.size()) return 1.0;
    unordered_set<long long> refEdges;
    refEdges.reserve(reference.size() * 2);
    auto edgeKey = [](int a, int b) -> long long {
        return (static_cast<long long>(a) << 32) ^ static_cast<unsigned int>(b);
    };

    for (size_t i = 0; i + 1 < reference.size(); ++i) {
        refEdges.insert(edgeKey(reference[i], reference[i + 1]));
    }

    int overlap = 0;
    for (size_t i = 0; i + 1 < seq.size(); ++i) {
        if (refEdges.count(edgeKey(seq[i], seq[i + 1])) > 0) overlap++;
    }

    int totalEdges = max(1, (int)seq.size() - 1);
    return 1.0 - (double)overlap / totalEdges;
}

static Chromosome makeDefaultChromosome(const vector<int>& seq, const PDPData& data) {
    Chromosome c;
    c.sequence = seq;
    int n = (int)seq.size();
    c.truck_assign.assign(n, 0);
    c.drone_assign.assign(n, 0);
    c.break_bit.assign(n, 1);
    for (int i = 0; i < n; ++i) {
        int t = (int)((1LL * i * data.numTrucks) / max(1, n));
        if (t < 0) t = 0;
        if (t >= data.numTrucks) t = data.numTrucks - 1;
        c.truck_assign[i] = t;
    }
    return c;
}

static bool parentHasFullEncoding(const Chromosome& p) {
    size_t n = p.sequence.size();
    return p.truck_assign.size() == n && p.drone_assign.size() == n && p.break_bit.size() == n;
}

static Chromosome inheritEncodingForChild(
    const vector<int>& childSeq,
    const Chromosome& parent1,
    const Chromosome& parent2,
    const PDPData& data,
    mt19937& rng
) {
    Chromosome child = makeDefaultChromosome(childSeq, data);
    int n = (int)childSeq.size();
    if (n == 0) return child;

    const bool p1ok = parentHasFullEncoding(parent1);
    const bool p2ok = parentHasFullEncoding(parent2);
    if (!p1ok && !p2ok) return child;

    // Cross-array inheritance by INDEX (service slot), not by customer ID.
    // This forces a customer that moves to a new position to adapt to that slot's truck/drone/break pattern.
    bernoulli_distribution pickP1(0.5);
    for (int i = 0; i < n; ++i) {
        bool use1 = pickP1(rng);

        // If both parents have encodings, pick per-position uniformly.
        // If only one parent has encoding, always inherit from it.
        const Chromosome* src = nullptr;
        if (p1ok && p2ok) {
            src = use1 ? &parent1 : &parent2;
        } else if (p1ok) {
            src = &parent1;
        } else {
            src = &parent2;
        }

        // Parents are expected to have same chromosome length (numCustomers) in GA.
        // Guard anyway: if mismatch, fall back to child's defaults for that position.
        if ((int)src->truck_assign.size() == n) child.truck_assign[i] = src->truck_assign[i];
        if ((int)src->drone_assign.size() == n) child.drone_assign[i] = src->drone_assign[i];
        if ((int)src->break_bit.size() == n) child.break_bit[i] = src->break_bit[i];

        // Clamp/sanitize to valid ranges.
        if (child.truck_assign[i] < 0) child.truck_assign[i] = 0;
        if (child.truck_assign[i] >= data.numTrucks) child.truck_assign[i] = max(0, data.numTrucks - 1);
        if (child.drone_assign[i] < 0) child.drone_assign[i] = 0;
        if (child.drone_assign[i] > data.numDrones) child.drone_assign[i] = max(0, data.numDrones);
        child.break_bit[i] = (child.break_bit[i] != 0) ? 1 : 0;
    }
    return child;
}

// ===================== PROPERTY MUTATIONS (assignment-level diversity) =====================

static inline bool isDroneCustomerNode(const PDPData& data, int nodeId) {
    if (!data.isCustomer(nodeId)) return false;
    if (nodeId < 0 || nodeId >= data.numNodes) return false;
    return (data.nodeTypes[nodeId] == "D" && data.readyTimes[nodeId] > 0);
}

// Randomly change truck assignment at a few positions.
static void mutateTruckAssignment(Chromosome& c, mt19937& gen, const PDPData& data) {
    int n = (int)c.sequence.size();
    if (n == 0 || data.numTrucks <= 1) return;
    if ((int)c.truck_assign.size() != n) c.truck_assign.assign(n, 0);

    // Mutate ~5% positions, at least 1.
    int k = max(1, n / 20);
    uniform_int_distribution<> posDist(0, n - 1);
    uniform_int_distribution<> deltaDist(0, data.numTrucks - 2);

    for (int t = 0; t < k; ++t) {
        int i = posDist(gen);
        int cur = c.truck_assign[i];
        int delta = 1 + deltaDist(gen);
        int next = (cur + delta) % data.numTrucks;
        c.truck_assign[i] = next;
    }
}

// Randomly change drone assignment for D-type customers.
static void mutateDroneAssignment(Chromosome& c, mt19937& gen, const PDPData& data) {
    int n = (int)c.sequence.size();
    if (n == 0 || data.numDrones <= 0) return;
    if ((int)c.drone_assign.size() != n) c.drone_assign.assign(n, 0);

    vector<int> dPositions;
    dPositions.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (isDroneCustomerNode(data, c.sequence[i])) dPositions.push_back(i);
        else c.drone_assign[i] = 0; // keep non-D consistent
    }
    if (dPositions.empty()) return;

    // Mutate ~10% of D positions, at least 1.
    int k = max(1, (int)dPositions.size() / 10);
    shuffle(dPositions.begin(), dPositions.end(), gen);
    k = min(k, (int)dPositions.size());

    uniform_int_distribution<> droneDist(0, data.numDrones); // includes 0 = truck
    for (int t = 0; t < k; ++t) {
        int i = dPositions[t];
        int cur = c.drone_assign[i];
        int next = cur;
        // Ensure change (unless only one option).
        for (int tries = 0; tries < 3; ++tries) {
            next = droneDist(gen);
            if (next != cur) break;
        }
        c.drone_assign[i] = next;
    }
}

// Flip break bits to encourage splitting/merging drone batches.
static void mutateBreakBit(Chromosome& c, mt19937& gen) {
    int n = (int)c.sequence.size();
    if (n == 0) return;
    if ((int)c.break_bit.size() != n) c.break_bit.assign(n, 1);

    // Mutate ~10% positions, at least 1.
    int k = max(1, n / 10);
    uniform_int_distribution<> posDist(0, n - 1);
    for (int t = 0; t < k; ++t) {
        int i = posDist(gen);
        c.break_bit[i] = (c.break_bit[i] == 0) ? 1 : 0;
    }
}

// Long-range consolidation: bring a far D-customer next to another D-customer with same drone k
// if they are compatible by ready time and distance.
static void mutateLongRangeConsolidation(Chromosome& c, mt19937& gen, const PDPData& data) {
    int n = (int)c.sequence.size();
    if (n < 2 || data.numDrones <= 0) return;
    if ((int)c.drone_assign.size() != n) c.drone_assign.assign(n, 0);
    if ((int)c.break_bit.size() != n) c.break_bit.assign(n, 1);
    if ((int)c.truck_assign.size() != n) c.truck_assign.assign(n, 0);

    // Collect candidates i that are D-type and currently assigned to some drone k>0.
    vector<int> candidatesI;
    candidatesI.reserve(n);
    for (int i = 0; i < n; ++i) {
        int nodeI = c.sequence[i];
        if (!isDroneCustomerNode(data, nodeI)) continue;
        int kDrone = c.drone_assign[i];
        if (kDrone <= 0 || kDrone > data.numDrones) continue;
        candidatesI.push_back(i);
    }
    if (candidatesI.empty()) return;

    uniform_int_distribution<> pickI(0, (int)candidatesI.size() - 1);
    int i = candidatesI[pickI(gen)];
    int nodeI = c.sequence[i];
    int kDrone = c.drone_assign[i];
    int readyI = data.readyTimes[nodeI];

    // Find a compatible j.
    int bestJ = -1;
    double bestDist = numeric_limits<double>::infinity();
    for (int j = 0; j < n; ++j) {
        if (j == i) continue;
        int nodeJ = c.sequence[j];
        if (!isDroneCustomerNode(data, nodeJ)) continue;
        if (c.drone_assign[j] != kDrone) continue;
        int readyJ = data.readyTimes[nodeJ];
        if (abs(readyJ - readyI) > 30) continue; // minutes
        double d = getDroneDistance(data, nodeI, nodeJ);
        if (d > 10.0) continue; // km threshold (as requested)
        if (d < bestDist) {
            bestDist = d;
            bestJ = j;
        }
    }
    if (bestJ < 0) return;

    // Move gene at bestJ to be adjacent (right after) i, keeping all arrays aligned.
    auto eraseAt = [&](vector<int>& v, int pos) {
        v.erase(v.begin() + pos);
    };
    auto insertAt = [&](vector<int>& v, int pos, int value) {
        v.insert(v.begin() + pos, value);
    };

    int j = bestJ;
    int seqVal = c.sequence[j];
    int truckVal = c.truck_assign[j];
    int droneVal = c.drone_assign[j];
    int breakVal = c.break_bit[j];

    eraseAt(c.sequence, j);
    eraseAt(c.truck_assign, j);
    eraseAt(c.drone_assign, j);
    eraseAt(c.break_bit, j);

    // If we removed an element before i, i shifts left by 1.
    if (j < i) i -= 1;

    int insertPos = min(n - 1, i + 1);
    insertAt(c.sequence, insertPos, seqVal);
    insertAt(c.truck_assign, insertPos, truckVal);
    insertAt(c.drone_assign, insertPos, droneVal);
    // Force merge: set break_bit of the moved (second) customer to 0.
    insertAt(c.break_bit, insertPos, 0);

    (void)breakVal; // old break bit intentionally ignored
}

static void swapMutationChromosome(Chromosome& c, mt19937& gen) {
    if (c.sequence.size() < 2) return;
    uniform_int_distribution<> dist(0, (int)c.sequence.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    swap(c.sequence[i], c.sequence[j]);
    if (c.truck_assign.size() == c.sequence.size()) swap(c.truck_assign[i], c.truck_assign[j]);
    if (c.drone_assign.size() == c.sequence.size()) swap(c.drone_assign[i], c.drone_assign[j]);
    if (c.break_bit.size() == c.sequence.size()) swap(c.break_bit[i], c.break_bit[j]);
}

static void inversionMutationChromosome(Chromosome& c, mt19937& gen) {
    if (c.sequence.size() < 2) return;
    uniform_int_distribution<> dist(0, (int)c.sequence.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    if (i > j) swap(i, j);
    reverse(c.sequence.begin() + i, c.sequence.begin() + j + 1);
    if (c.truck_assign.size() == c.sequence.size()) reverse(c.truck_assign.begin() + i, c.truck_assign.begin() + j + 1);
    if (c.drone_assign.size() == c.sequence.size()) reverse(c.drone_assign.begin() + i, c.drone_assign.begin() + j + 1);
    if (c.break_bit.size() == c.sequence.size()) reverse(c.break_bit.begin() + i, c.break_bit.begin() + j + 1);
}

static void scrambleMutationChromosome(Chromosome& c, mt19937& gen) {
    if (c.sequence.size() < 2) return;
    uniform_int_distribution<> dist(0, (int)c.sequence.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    if (i > j) swap(i, j);
    vector<int> idx;
    idx.reserve(j - i + 1);
    for (int k = i; k <= j; ++k) idx.push_back(k);
    shuffle(idx.begin(), idx.end(), gen);

    auto apply = [&](vector<int>& v) {
        vector<int> tmp;
        tmp.reserve(idx.size());
        for (int k : idx) tmp.push_back(v[k]);
        for (int t = 0; t < (int)idx.size(); ++t) v[i + t] = tmp[t];
    };

    {
        vector<int> tmp;
        tmp.reserve(idx.size());
        for (int k : idx) tmp.push_back(c.sequence[k]);
        for (int t = 0; t < (int)idx.size(); ++t) c.sequence[i + t] = tmp[t];
    }
    if (c.truck_assign.size() == c.sequence.size()) apply(c.truck_assign);
    if (c.drone_assign.size() == c.sequence.size()) apply(c.drone_assign);
    if (c.break_bit.size() == c.sequence.size()) apply(c.break_bit);
}

static void insertionMutationChromosome(Chromosome& c, mt19937& gen) {
    if (c.sequence.size() < 2) return;
    uniform_int_distribution<> dist(0, (int)c.sequence.size() - 1);
    int fromPos = dist(gen);
    int toPos = dist(gen);
    if (fromPos == toPos) return;

    auto moveOne = [&](vector<int>& v) {
        int element = v[fromPos];
        if (fromPos < toPos) {
            for (int i = fromPos; i < toPos; ++i) v[i] = v[i + 1];
        } else {
            for (int i = fromPos; i > toPos; --i) v[i] = v[i - 1];
        }
        v[toPos] = element;
    };

    moveOne(c.sequence);
    if (c.truck_assign.size() == c.sequence.size()) moveOne(c.truck_assign);
    if (c.drone_assign.size() == c.sequence.size()) moveOne(c.drone_assign);
    if (c.break_bit.size() == c.sequence.size()) moveOne(c.break_bit);
}

static void displacementMutationChromosome(Chromosome& c, mt19937& gen) {
    if (c.sequence.size() < 3) return;
    uniform_int_distribution<> dist(0, (int)c.sequence.size() - 1);
    int start = dist(gen);
    int end = dist(gen);
    if (start > end) swap(start, end);
    if (start == end) {
        if (end < (int)c.sequence.size() - 1) end++;
        else if (start > 0) start--;
        else return;
    }

    vector<int> validPositions;
    int segLen = end - start + 1;
    for (int i = 0; i <= (int)c.sequence.size() - segLen; ++i) {
        if (i < start || i > end - (end - start)) validPositions.push_back(i);
    }
    if (validPositions.empty()) return;

    uniform_int_distribution<> posDist(0, (int)validPositions.size() - 1);
    int newPos = validPositions[posDist(gen)];

    auto moveSeg = [&](vector<int>& v) {
        vector<int> segment(v.begin() + start, v.begin() + end + 1);
        v.erase(v.begin() + start, v.begin() + end + 1);
        int insertPos = newPos;
        if (insertPos > start) insertPos -= segLen;
        insertPos = max(0, min(insertPos, (int)v.size()));
        v.insert(v.begin() + insertPos, segment.begin(), segment.end());
    };

    moveSeg(c.sequence);
    if (c.truck_assign.size() == c.sequence.size()) moveSeg(c.truck_assign);
    if (c.drone_assign.size() == c.sequence.size()) moveSeg(c.drone_assign);
    if (c.break_bit.size() == c.sequence.size()) moveSeg(c.break_bit);
}

// ============ PERTURBATION OPERATORS (for Tabu diversification) ============

// Double-bridge perturbation: splits sequence into 4 parts and reassembles
vector<int> doubleBridgePerturbation(const vector<int>& seq, mt19937& gen) {
    int n = seq.size();
    if (n < 8) return seq; // too small
    
    uniform_int_distribution<> dist(1, n - 2);
    vector<int> cuts;
    set<int> cutSet;
    while ((int)cuts.size() < 3) {
        int c = dist(gen);
        if (cutSet.find(c) == cutSet.end()) {
            cuts.push_back(c);
            cutSet.insert(c);
        }
    }
    sort(cuts.begin(), cuts.end());
    
    // Split into 4 segments: [0,c1), [c1,c2), [c2,c3), [c3,n)
    vector<int> result;
    // Reassemble as: segment0, segment2, segment1, segment3
    for (int i = 0; i < cuts[0]; i++) result.push_back(seq[i]);
    for (int i = cuts[1]; i < cuts[2]; i++) result.push_back(seq[i]);
    for (int i = cuts[0]; i < cuts[1]; i++) result.push_back(seq[i]);
    for (int i = cuts[2]; i < n; i++) result.push_back(seq[i]);
    return result;
}

static Chromosome doubleBridgePerturbation(const Chromosome& chromo, mt19937& gen) {
    int n = (int)chromo.sequence.size();
    if (n < 8) return chromo;
    uniform_int_distribution<> dist(1, n - 2);
    vector<int> cuts;
    set<int> cutSet;
    while ((int)cuts.size() < 3) {
        int c = dist(gen);
        if (!cutSet.count(c)) {
            cuts.push_back(c);
            cutSet.insert(c);
        }
    }
    sort(cuts.begin(), cuts.end());

    auto appendRange = [&](Chromosome& out, int a, int b) {
        for (int i = a; i < b; ++i) {
            out.sequence.push_back(chromo.sequence[i]);
            if (chromo.truck_assign.size() == chromo.sequence.size()) out.truck_assign.push_back(chromo.truck_assign[i]);
            if (chromo.drone_assign.size() == chromo.sequence.size()) out.drone_assign.push_back(chromo.drone_assign[i]);
            if (chromo.break_bit.size() == chromo.sequence.size()) out.break_bit.push_back(chromo.break_bit[i]);
        }
    };

    Chromosome out;
    out.sequence.reserve(n);
    out.truck_assign.reserve(chromo.truck_assign.size());
    out.drone_assign.reserve(chromo.drone_assign.size());
    out.break_bit.reserve(chromo.break_bit.size());

    // segment0, segment2, segment1, segment3
    appendRange(out, 0, cuts[0]);
    appendRange(out, cuts[1], cuts[2]);
    appendRange(out, cuts[0], cuts[1]);
    appendRange(out, cuts[2], n);
    return out;
}

// Ruin-recreate perturbation: remove k random customers and reinsert at random positions
vector<int> ruinRecreatePerturbation(const vector<int>& seq, mt19937& gen, double ruinRatio = 0.3) {
    int n = seq.size();
    int k = max(2, (int)(n * ruinRatio));
    
    // Select k random positions to remove
    vector<int> indices(n);
    iota(indices.begin(), indices.end(), 0);
    shuffle(indices.begin(), indices.end(), gen);
    
    set<int> removeSet(indices.begin(), indices.begin() + k);
    
    vector<int> removed;
    vector<int> remaining;
    for (int i = 0; i < n; i++) {
        if (removeSet.count(i)) {
            removed.push_back(seq[i]);
        } else {
            remaining.push_back(seq[i]);
        }
    }
    
    // Reinsert removed customers at random positions
    shuffle(removed.begin(), removed.end(), gen);
    for (int cust : removed) {
        uniform_int_distribution<> posDist(0, (int)remaining.size());
        int pos = posDist(gen);
        remaining.insert(remaining.begin() + pos, cust);
    }
    return remaining;
}

static Chromosome ruinRecreatePerturbation(const Chromosome& chromo, mt19937& gen, double ruinRatio = 0.3) {
    int n = (int)chromo.sequence.size();
    if (n < 4) return chromo;
    if (!parentHasFullEncoding(chromo)) return chromo;

    int k = max(2, (int)(n * ruinRatio));
    k = min(k, n - 1);

    vector<int> indices(n);
    iota(indices.begin(), indices.end(), 0);
    shuffle(indices.begin(), indices.end(), gen);
    set<int> removeSet(indices.begin(), indices.begin() + k);

    struct Gene { int cust, truck, drone, br; };
    vector<Gene> removed;
    removed.reserve(k);

    Chromosome remaining;
    remaining.sequence.reserve(n);
    remaining.truck_assign.reserve(n);
    remaining.drone_assign.reserve(n);
    remaining.break_bit.reserve(n);

    for (int i = 0; i < n; ++i) {
        if (removeSet.count(i)) {
            removed.push_back({chromo.sequence[i], chromo.truck_assign[i], chromo.drone_assign[i], chromo.break_bit[i]});
        } else {
            remaining.sequence.push_back(chromo.sequence[i]);
            remaining.truck_assign.push_back(chromo.truck_assign[i]);
            remaining.drone_assign.push_back(chromo.drone_assign[i]);
            remaining.break_bit.push_back(chromo.break_bit[i]);
        }
    }

    shuffle(removed.begin(), removed.end(), gen);
    for (const auto& g : removed) {
        uniform_int_distribution<> posDist(0, (int)remaining.sequence.size());
        int pos = posDist(gen);
        remaining.sequence.insert(remaining.sequence.begin() + pos, g.cust);
        remaining.truck_assign.insert(remaining.truck_assign.begin() + pos, g.truck);
        remaining.drone_assign.insert(remaining.drone_assign.begin() + pos, g.drone);
        remaining.break_bit.insert(remaining.break_bit.begin() + pos, g.br);
    }

    return remaining;
}

// ============ CROSSOVER OPERATORS ============

// Order Crossover (OX)
vector<int> orderCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child(n, -1);
    
    uniform_int_distribution<> dist(0, n - 1);
    int cut1 = dist(gen);
    int cut2 = dist(gen);
    if (cut1 > cut2) swap(cut1, cut2);
    
    // Copy segment from parent1
    for (int i = cut1; i <= cut2; ++i) {
        child[i] = parent1[i];
    }
    
    // Fill remaining from parent2
    set<int> used;
    for (int i = cut1; i <= cut2; ++i) {
        used.insert(parent1[i]);
    }
    
    int childIdx = (cut2 + 1) % n;
    int parentIdx = (cut2 + 1) % n;
    
    while (childIdx != cut1) {
        if (used.find(parent2[parentIdx]) == used.end()) {
            child[childIdx] = parent2[parentIdx];
            used.insert(parent2[parentIdx]);
            childIdx = (childIdx + 1) % n;
        }
        parentIdx = (parentIdx + 1) % n;
    }
    
    return child;
}

/// 1. Partially Mapped Crossover (PMX) 
vector<int> pmxCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child = parent1;
    
    uniform_int_distribution<> dist(0, n - 1);
    int cut1 = dist(gen);
    int cut2 = dist(gen);
    if (cut1 > cut2) swap(cut1, cut2);
    
    map<int, int> mapping;
    for (int i = cut1; i <= cut2; ++i) {
        child[i] = parent1[i];
        mapping[parent1[i]] = parent2[i];
    }
    
    for (int i = 0; i < n; ++i) {
        if (i >= cut1 && i <= cut2) continue;
        
        int val = parent2[i];
        int failsafe = n * 2; // CHỐT AN TOÀN: Chống lặp vô tận
        while (mapping.find(val) != mapping.end() && failsafe-- > 0) {
            val = mapping[val];
        }
        child[i] = val;
    }
    
    return child;
}

// 2. Cycle Crossover (CX) 
vector<int> cycleCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child(n, -1);
    vector<bool> visited(n, false);
    
    uniform_int_distribution<> coin(0, 1);
    bool useParent1 = coin(gen);
    
    for (int start = 0; start < n; ++start) {
        if (visited[start]) continue;
        
        int idx = start;
        int failsafe = n * 2; // CHỐT AN TOÀN: Chống lặp vô tận
        do {
            if (idx < 0 || idx >= n) break; // Tránh Out-of-bounds
            visited[idx] = true;
            child[idx] = useParent1 ? parent1[idx] : parent2[idx];
            
            int value = useParent1 ? parent2[idx] : parent1[idx];
            auto it = find(parent1.begin(), parent1.end(), value);
            if (it == parent1.end()) break;
            
            idx = distance(parent1.begin(), it);
        } while (idx != start && failsafe-- > 0);
        
        useParent1 = !useParent1;
    }
    
    // Điền lỗ hổng nếu failsafe kích hoạt
    for (int i = 0; i < n; ++i) {
        if (child[i] == -1) child[i] = parent1[i];
    }
    
    return child;
}

// Edge Recombination Crossover (ERX) 
vector<int> edgeCrossover(const vector<int>& parent1, const vector<int>& parent2, mt19937& gen) {
    int n = parent1.size();
    vector<int> child;
    
   
    map<int, set<int>> edges;
    
    // Th├¬m edges tß╗½ parent1
    for (int i = 0; i < n; ++i) {
        int current = parent1[i];
        int prev = parent1[(i - 1 + n) % n];
        int next = parent1[(i + 1) % n];
        edges[current].insert(prev);
        edges[current].insert(next);
    }
    
    // Th├¬m edges tß╗½ parent2
    for (int i = 0; i < n; ++i) {
        int current = parent2[i];
        int prev = parent2[(i - 1 + n) % n];
        int next = parent2[(i + 1) % n];
        edges[current].insert(prev);
        edges[current].insert(next);
    }
    
    
    uniform_int_distribution<> dist(0, n - 1);
    int current = parent1[dist(gen)];
    child.push_back(current);
    
    set<int> used;
    used.insert(current);
    
    while (child.size() < n) {
        
        for (auto& pair : edges) {
            pair.second.erase(current);
        }
        
        int next = -1;
        
        int minConnections = INT_MAX;
        for (int neighbor : edges[current]) {
            if (used.find(neighbor) == used.end()) {
                int connections = edges[neighbor].size();
                if (connections < minConnections) {
                    minConnections = connections;
                    next = neighbor;
                }
            }
        }
        
       
        if (next == -1) {
            vector<int> unused;
            for (int i = 0; i < n; ++i) {
                int node = parent1[i];
                if (used.find(node) == used.end()) {
                    unused.push_back(node);
                }
            }
            if (!unused.empty()) {
                uniform_int_distribution<> unusedDist(0, unused.size() - 1);
                next = unused[unusedDist(gen)];
            }
        }
        
        if (next != -1) {
            child.push_back(next);
            used.insert(next);
            current = next;
        } else {
            break; 
        }
    }
    
    if (child.size() < n) {
        for (int i = 0; i < n; ++i) {
            int node = parent1[i];
            if (used.find(node) == used.end()) {
                child.push_back(node);
                used.insert(node);
            }
        }
    }
    
    return child;
}

// ============ MUTATION OPERATORS ============

void swapMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    swap(seq[i], seq[j]);
}

void inversionMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    if (i > j) swap(i, j);
    
    reverse(seq.begin() + i, seq.begin() + j + 1);
}

void scrambleMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    if (i > j) swap(i, j);
    
    shuffle(seq.begin() + i, seq.begin() + j + 1, gen);
}

void insertionMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 2) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int fromPos = dist(gen);
    int toPos = dist(gen);
    
    if (fromPos == toPos) return;
    
    int element = seq[fromPos];
    if (fromPos < toPos) {
        for (int i = fromPos; i < toPos; ++i) {
            seq[i] = seq[i + 1];
        }
    } else {
        for (int i = fromPos; i > toPos; --i) {
            seq[i] = seq[i - 1];
        }
    }
    seq[toPos] = element;
}


void displacementMutation(vector<int>& seq, mt19937& gen) {
    if (seq.size() < 3) return;
    
    uniform_int_distribution<> dist(0, seq.size() - 1);
    int start = dist(gen);
    int end = dist(gen);
    if (start > end) swap(start, end);
    
    // ─Éß║úm bß║úo c├│ ├¡t nhß║Ñt 1 phß║ºn tß╗¡ ─æß╗â di chuyß╗ân
    if (start == end) {
        if (end < (int)seq.size() - 1) end++;
        else if (start > 0) start--;
        else return;
    }
    
    
    vector<int> validPositions;
    for (int i = 0; i <= (int)seq.size() - (end - start + 1); ++i) {
        if (i < start || i > end - (end - start)) {
            validPositions.push_back(i);
        }
    }
    
    if (validPositions.empty()) return;
    
    uniform_int_distribution<> posDist(0, validPositions.size() - 1);
    int newPos = validPositions[posDist(gen)];
    

    vector<int> segment(seq.begin() + start, seq.begin() + end + 1);
    

    seq.erase(seq.begin() + start, seq.begin() + end + 1);
    
   
    if (newPos > start) {
        newPos -= (end - start + 1);
    }
    newPos = max(0, min(newPos, (int)seq.size()));
    
    // Ch├¿n v├áo vß╗ï tr├¡ mß╗¢i
    seq.insert(seq.begin() + newPos, segment.begin(), segment.end());
}

// ============ REPAIR OPERATOR ============

void repairSequence(vector<int>& seq, const PDPData& data, mt19937& gen) {
    // ─Éß║┐m sß╗æ lß║ºn xuß║Ñt hiß╗çn cß╗ºa mß╗ùi customer
    map<int, int> count;
    for (int id : seq) {
        if (data.isCustomer(id)) {
            count[id]++;
        }
    }
    
    // T├¼m customer bß╗ï thiß║┐u
    vector<int> missing;
    for (int i = 0; i < data.numNodes; ++i) {
        if (data.isCustomer(i) && count[i] == 0) {
            missing.push_back(i);
        }
    }
    
    // Loß║íi bß╗Å duplicate v├á thay bß║▒ng missing
    shuffle(missing.begin(), missing.end(), gen);
    int missingIdx = 0;
    
    for (int& id : seq) {
        if (data.isCustomer(id) && count[id] > 1) {
            if (missingIdx < (int)missing.size()) {
                int oldId = id;
                id = missing[missingIdx++];
                count[oldId]--;
                count[id]++;
            }
        }
    }
    
    // Th├¬m missing c├▓n lß║íi
    for (int i = missingIdx; i < (int)missing.size(); ++i) {
        seq.push_back(missing[i]);
    }
}

// ============ SELECTION ============

int tournamentSelection(const vector<double>& fitness,
                        int tournamentSize,
                        mt19937& gen) {
    uniform_int_distribution<> dist(0, (int)fitness.size() - 1);

    int bestIdx = dist(gen);
    double bestFitness = fitness[bestIdx];

    for (int i = 1; i < tournamentSize; ++i) {
        int idx = dist(gen);
        if (fitness[idx] < bestFitness) {
            bestIdx = idx;
            bestFitness = fitness[idx];
        }
    }

    return bestIdx;
}

// ============ ADAPTIVE EVALUATION ============

// Lightweight structural proxy to avoid extra decode calls during operator scoring.
// Returns true when offspring is sufficiently different from both parents.
bool evaluateImprovement(const vector<int>& offspring, const vector<int>& parent1, 
                        const vector<int>& parent2, const PDPData& data) {
    (void)data;
    int diff1 = 0;
    int diff2 = 0;
    int n = (int)offspring.size();
    for (int i = 0; i < n; ++i) {
        if (offspring[i] != parent1[i]) diff1++;
        if (offspring[i] != parent2[i]) diff2++;
    }
    // Mark as promising if child differs by at least 20% from both parents.
    int minDiff = max(1, n / 5);
    return (diff1 >= minDiff && diff2 >= minDiff);
}

// Lightweight proxy: count mutation as useful if sequence actually changed.
bool evaluateMutationImprovement(const vector<int>& mutated, const vector<int>& original, 
                               const PDPData& data) {
    (void)data;
    return mutated != original;
}

// ============ MAIN GA ALGORITHM ============

PDPSolution geneticAlgorithmPDP(const PDPData& data, int populationSize, 
                               int maxGenerations, double mutationRate, int runNumber) {
    random_device rd;
    mt19937 rng(rd() + runNumber * 12345);
    
    cout << "\n=========================================" << endl;
    cout << "  GENETIC ALGORITHM + TABU SEARCH (PDP)" << endl;
    cout << "=========================================" << endl;
    cout << "Population size: " << populationSize << endl;
    cout << "Max generations: " << maxGenerations << endl;
    cout << "Base mutation rate: " << mutationRate << " (adaptive)" << endl;
    cout << "Tabu threshold: decoded-evaluation based" << endl;
    cout << "Adaptive operators: ENABLED" << endl;
    
    // Initialize adaptive parameters
    AdaptiveParams adaptiveParams(mutationRate);
    OnlineSurrogate surrogate;
    
    // STEP 0: Initialize Solution Cache
    // Cache persists across all generations to leverage solution reuse
    SolutionCache solutionCache;
    cout << "\n[0] Solution Cache initialized (MAX_SIZE: 150000 entries ≈ 1.5GB RAM)" << endl;
    
    // STEP 1: Initialize population
    cout << "\n[1] Initializing population..." << endl;
    vector<Chromosome> population;
    population.resize(populationSize);
    {
        vector<Chromosome> initChromos = initStructuredPopulationChromosome(populationSize, data, runNumber);
        for (int i = 0; i < populationSize; ++i) {
            population[i] = (i < (int)initChromos.size()) ? initChromos[i] : Chromosome();
        }
    }
    
    // Evaluate initial population
    vector<double> fitness(populationSize);
    PDPSolution bestSolution;
    bestSolution.totalCost = numeric_limits<double>::infinity();
    vector<int> bestSequence;
    Chromosome bestChromosome;
    
    for (int i = 0; i < populationSize; ++i) {
        PDPSolution sol = evaluateWithCache(population[i], data, solutionCache);
        fitness[i] = sol.totalCost + sol.totalPenalty;
        
        if (fitness[i] < bestSolution.totalCost + bestSolution.totalPenalty) {
            bestSolution = sol;
            bestSequence = sol.sequence;
            bestChromosome = static_cast<const Chromosome&>(sol);
        }
    }
    
    cout << "Initial best cost: " << fixed << setprecision(2) 
         << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
    
    int noImprovementCounter = 0;
    int noImprovementEvalCounter = 0;
    int earlyStopThreshold = (int)(0.2 * maxGenerations);  // 20% of generations without improvement
    int tabuThreshold = data.numCustomers <= 20 ? 
                        max(100, populationSize * 5) :      // Small instances: trigger early Tabu
                        max(200, populationSize * 20);      // Large instances: original threshold
    int tabuRounds = 0;
    bool tabuApplied = false;
    int adaptationInterval = max(5, maxGenerations / 20);
    
    // STEP 2: GA Loop
    for (int generation = 0; generation < maxGenerations; ++generation) {
        // 2.1: Create offspring using adaptive crossover
        vector<Chromosome> offspring;
        vector<int> crossoverTypes;
        vector<pair<double, double>> parentFitnesses;  // Track parent fitness for threshold
        
        int numOffspring = populationSize;
        for (int i = 0; i < numOffspring; ++i) {
            int p1Idx = tournamentSelection(fitness, 3, rng);
            int p2Idx = tournamentSelection(fitness, 3, rng);
            const vector<int>& parent1 = population[p1Idx].sequence;
            const vector<int>& parent2 = population[p2Idx].sequence;
            parentFitnesses.push_back({fitness[p1Idx], fitness[p2Idx]});
            
            int crossoverType = adaptiveParams.selectCrossoverType(rng);
            crossoverTypes.push_back(crossoverType);
            
            vector<int> child;
            // Use only first 3 crossovers (safer, well-tested)
            int safeType = crossoverType % 3;
            if (safeType == 0) {
                child = orderCrossover(parent1, parent2, rng);
            } else if (safeType == 1) {
                child = pmxCrossover(parent1, parent2, rng);
            } else {
                child = cycleCrossover(parent1, parent2, rng);
            }
            repairSequence(child, data, rng);

            Chromosome childChromo = inheritEncodingForChild(child, population[p1Idx], population[p2Idx], data, rng);
            offspring.push_back(childChromo);
            
        
            bool improved = evaluateImprovement(child, parent1, parent2, data);
            adaptiveParams.updateCrossoverSuccess(crossoverType, improved);
        }
        
        // 2.2: Adaptive Mutation
        int mutationCount = (int)(offspring.size() * adaptiveParams.currentMutationRate);
        uniform_int_distribution<> offspringDist(0, offspring.size() - 1);
        
        for (int i = 0; i < mutationCount; ++i) {
            int idx = offspringDist(rng);
            Chromosome original = offspring[idx];
            
            // Mix structural mutations (sequence-level) AND property mutations (assignment-level).
            // Structural mutations keep the "bốc nguyên cột dọc" behavior; property mutations break that stickiness.
            int mutationType = adaptiveParams.selectMutationType(rng);
            if (mutationType == 0) {
                swapMutationChromosome(offspring[idx], rng);
            } else if (mutationType == 1) {
                inversionMutationChromosome(offspring[idx], rng);
            } else if (mutationType == 2) {
                scrambleMutationChromosome(offspring[idx], rng);
            } else if (mutationType == 3) {
                insertionMutationChromosome(offspring[idx], rng);
            } else {
                displacementMutationChromosome(offspring[idx], rng);
            }

            // Property mutation stage (probabilistic): encourages assignment exploration.
            // Tune these if you see either too much randomness (penalties explode) or too much convergence.
            bernoulli_distribution doProp(0.55);
            if (doProp(rng)) {
                // Weighted choice among 4 property mutations.
                // (truck, drone, break, long-range)
                discrete_distribution<> propPick({25, 30, 30, 15});
                int p = propPick(rng);
                if (p == 0) {
                    mutateTruckAssignment(offspring[idx], rng, data);
                } else if (p == 1) {
                    mutateDroneAssignment(offspring[idx], rng, data);
                } else if (p == 2) {
                    mutateBreakBit(offspring[idx], rng);
                } else {
                    mutateLongRangeConsolidation(offspring[idx], rng, data);
                }
            }

            // Lightweight proxy: mutation is useful if anything changed.
            bool improved = !(offspring[idx] == original);
            adaptiveParams.updateMutationSuccess(mutationType, improved);
        }
        
        // 2.3: Evaluate offspring with quota-based pre-screening (elite + exploration)
        // Compute adaptive φ = η_current / η_max (η_max = 100)
        double phi = min(1.0, (double)noImprovementCounter / 100.0);
        
        // Get current best cost and compute threshold
        double currentBestCost = bestSolution.totalCost + bestSolution.totalPenalty;
        
        // Threshold = (1 - φ) * z(χ*) + φ * LB(χ*)
        // Where z(χ*) = best solution cost, LB(χ*) = lower bound estimate
        vector<double> offspringFitness(offspring.size(), numeric_limits<double>::infinity());
        int decodedCount = 0;
        int skippedCount = 0;
        vector<double> proxyScore(offspring.size(), currentBestCost);
        vector<int> proxyOrder(offspring.size(), 0);
        iota(proxyOrder.begin(), proxyOrder.end(), 0);

        // Build cheap proxy score (lower is better).
        for (size_t i = 0; i < offspring.size(); ++i) {
            double lowerBoundEstimate = currentBestCost;
            if (i < parentFitnesses.size()) {
                lowerBoundEstimate = min(parentFitnesses[i].first, parentFitnesses[i].second) * 0.98;
            }
            // Adaptive threshold still used as a smooth proxy anchor.
            double threshold = (1.0 - phi) * currentBestCost + phi * lowerBoundEstimate;
            proxyScore[i] = min(lowerBoundEstimate, threshold);
        }

        sort(proxyOrder.begin(), proxyOrder.end(),
             [&proxyScore](int a, int b) { return proxyScore[a] < proxyScore[b]; });

        int eliteDecodeCount = max(1, (int)(offspring.size() * 0.50));
        int exploreDecodeCount = max(1, (int)(offspring.size() * 0.10));
        vector<char> shouldDecode(offspring.size(), 0);

        for (int k = 0; k < eliteDecodeCount && k < (int)proxyOrder.size(); ++k) {
            shouldDecode[proxyOrder[k]] = 1;
        }

        vector<int> tailIdx;
        for (int k = eliteDecodeCount; k < (int)proxyOrder.size(); ++k) {
            tailIdx.push_back(proxyOrder[k]);
        }

        // Diversity-aware exploration: decode candidates that are most novel vs current best.
        vector<pair<double, int>> noveltyCandidates;
        noveltyCandidates.reserve(tailIdx.size());
        for (int idx : tailIdx) {
            double novelty = edgeNoveltyScore(offspring[idx].sequence, bestSequence);
            noveltyCandidates.push_back({novelty, idx});
        }
        sort(noveltyCandidates.begin(), noveltyCandidates.end(),
             [](const pair<double, int>& a, const pair<double, int>& b) {
                 return a.first > b.first;
             });
        for (int k = 0; k < exploreDecodeCount && k < (int)noveltyCandidates.size(); ++k) {
            shouldDecode[noveltyCandidates[k].second] = 1;
        }
        
        for (size_t i = 0; i < offspring.size(); ++i) {
            if (shouldDecode[i]) {
                PDPSolution sol = evaluateWithCache(offspring[i], data, solutionCache);
                offspringFitness[i] = sol.totalCost + sol.totalPenalty;
                surrogate.update(proxyScore[i], offspringFitness[i]);
                decodedCount++;
            } else {
                // Surrogate-assisted estimate for non-decoded offspring.
                double predicted = surrogate.predict(proxyScore[i], proxyScore[i]);
                double conservative = max(proxyScore[i], predicted);
                offspringFitness[i] = conservative * 1.02;
                skippedCount++;
            }
        }
        
        if (generation > 0 && generation % 20 == 0) {
            cout << "[THRESHOLD] Gen " << generation << ": φ=" << fixed << setprecision(3) << phi 
                 << ", Best=" << setprecision(2) << currentBestCost
                 << ", Decoded=" << decodedCount << "/" << offspring.size() 
                 << " (" << (int)(decodedCount*100.0/offspring.size()) << "%)"
                  << ", Skipped=" << skippedCount
                  << ", SurrogateSamples=" << surrogate.n << endl;
        }
        
        // 2.4: Selection - 50% best offspring + 20% random offspring + 30% best parents
        int numBestOffspring = (int)(populationSize * 0.5);
        int numRandomOffspring = (int)(populationSize * 0.2);
        int numBestParents = populationSize - numBestOffspring - numRandomOffspring;
        
        // Sort offspring
        vector<int> offspringIndices(offspring.size());
        iota(offspringIndices.begin(), offspringIndices.end(), 0);
        sort(offspringIndices.begin(), offspringIndices.end(),
             [&](int a, int b) { return offspringFitness[a] < offspringFitness[b]; });
        
        // Sort parents
        vector<int> parentIndices(population.size());
        iota(parentIndices.begin(), parentIndices.end(), 0);
        sort(parentIndices.begin(), parentIndices.end(),
             [&](int a, int b) { return fitness[a] < fitness[b]; });
        
        // Create new population
        vector<Chromosome> newPopulation;
        vector<double> newFitness;
        
        // 50% best offspring
        for (int i = 0; i < numBestOffspring; ++i) {
            newPopulation.push_back(offspring[offspringIndices[i]]);
            newFitness.push_back(offspringFitness[offspringIndices[i]]);
        }
        
        // 20% random offspring (from remaining, not already selected)
        {
            vector<int> remainingOffIdx;
            for (int i = numBestOffspring; i < (int)offspringIndices.size(); ++i) {
                remainingOffIdx.push_back(offspringIndices[i]);
            }
            shuffle(remainingOffIdx.begin(), remainingOffIdx.end(), rng);
            int toAdd = min(numRandomOffspring, (int)remainingOffIdx.size());
            for (int i = 0; i < toAdd; ++i) {
                newPopulation.push_back(offspring[remainingOffIdx[i]]);
                newFitness.push_back(offspringFitness[remainingOffIdx[i]]);
            }
        }
        
        // 30% best parents (elitism)
        for (int i = 0; i < numBestParents; ++i) {
            newPopulation.push_back(population[parentIndices[i]]);
            newFitness.push_back(fitness[parentIndices[i]]);
        }
        
        population = newPopulation;
        fitness = newFitness;
        
        // Update best solution
        double currentBestCostAfterSelection = bestSolution.totalCost + bestSolution.totalPenalty;
        int bestIdx = 0;
        for (int i = 1; i < (int)fitness.size(); ++i) {
            if (fitness[i] < fitness[bestIdx]) bestIdx = i;
        }
        if (fitness[bestIdx] < currentBestCostAfterSelection) {
            PDPSolution sol = evaluateWithCache(population[bestIdx], data, solutionCache);
            bestSolution = sol;
            bestSequence = sol.sequence;
            bestChromosome = static_cast<const Chromosome&>(sol);
            noImprovementCounter = 0;
            noImprovementEvalCounter = 0;
            adaptiveParams.noImprovementCount = 0;

            cout << "Gen " << generation << ": New best = " << fixed << setprecision(2)
                 << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty
                 << ", mut_rate: " << setprecision(3) << adaptiveParams.currentMutationRate << ")" << endl;
        } else {
            noImprovementCounter++;
            noImprovementEvalCounter += decodedCount;
            adaptiveParams.noImprovementCount++;
        }
        
        // 2.6: Adaptive parameter updates
        if (generation > 0 && generation % adaptationInterval == 0) {
            adaptiveParams.adaptCrossoverRates();
            adaptiveParams.adaptMutationRates();
            adaptiveParams.adaptMutationRate();
            
            // In thß╗æng k├¬ adaptive (mß╗ùi 10 generations)
            if (generation % (adaptationInterval * 2) == 0) {
                cout << "[ADAPT] Gen " << generation << " - Crossover rates: ";
                for (int i = 0; i < 4; ++i) {
                    cout << fixed << setprecision(2) << adaptiveParams.crossoverRates[i] << " ";
                }
                cout << ", Mutation rate: " << setprecision(3) << adaptiveParams.currentMutationRate << endl;
            }
        }
        
        // 2.5: Apply Tabu Search to top 5% after stagnation
        if (noImprovementEvalCounter >= tabuThreshold) {
            int topK = max(1, populationSize / 10); // top 10%
            cout << "\n[TABU] No improvement for " << noImprovementEvalCounter
                 << " decoded evaluations. Applying Tabu Search to top " << topK << " individuals..." << endl;
            
            // Sort population indices by fitness (ascending = best first)
            vector<int> sortedIdx(populationSize);
            iota(sortedIdx.begin(), sortedIdx.end(), 0);
            sort(sortedIdx.begin(), sortedIdx.end(), 
                 [&fitness](int a, int b) { return fitness[a] < fitness[b]; });
            
            double bestBeforeTabu = bestSolution.totalCost + bestSolution.totalPenalty;
            
            // Also add random individuals (with perturbation) for diversity
            int numRandom = topK; // same number of random individuals
            vector<int> randomIdx;
            {
                vector<int> nonTopIdx;
                for (int i = topK; i < populationSize; i++) nonTopIdx.push_back(sortedIdx[i]);
                shuffle(nonTopIdx.begin(), nonTopIdx.end(), rng);
                int toTake = min(numRandom, (int)nonTopIdx.size());
                for (int i = 0; i < toTake; i++) randomIdx.push_back(nonTopIdx[i]);
            }
            
            // Process top individuals + random individuals
            int totalTabu = topK + (int)randomIdx.size();
            for (int k = 0; k < totalTabu; ++k) {
                int idx;
                bool isRandom = (k >= topK);
                if (!isRandom) {
                    idx = sortedIdx[k];
                } else {
                    idx = randomIdx[k - topK];
                }
                
                if ((int)population[idx].sequence.size() != data.numCustomers) continue;
                
                try {
                    // Apply perturbation before Tabu for diversity
                    Chromosome startChromo = population[idx];
                    if (isRandom || k > 0) {
                        // Alternate between double-bridge and ruin-recreate
                        if (k % 2 == 0) {
                            startChromo = doubleBridgePerturbation(startChromo, rng);
                        } else {
                            startChromo = ruinRecreatePerturbation(startChromo, rng, 0.3);
                        }
                    }

                    Chromosome tabuResult = tabuSearchPDP(startChromo, data, 50, solutionCache);
                    if ((int)tabuResult.sequence.size() != data.numCustomers) continue;

                    // Multi-start Assignment LS: try 3 random starts, pick best
                    PDPSolution bestTabuSol = evaluateWithCache(tabuResult, data, solutionCache);
                    double bestTabuFit = bestTabuSol.totalCost + bestTabuSol.totalPenalty;
                    
                    for (int ms = 0; ms < 3; ms++) {
                        // Create random assignment encoding
                        AssignmentEncoding randEnc;
                        int seqLen = (int)tabuResult.sequence.size();
                        randEnc.truck_assign.resize(seqLen);
                        randEnc.drone_assign.resize(seqLen);
                        randEnc.break_bit.resize(seqLen);
                        
                        uniform_int_distribution<> truckDist(0, data.numTrucks - 1);
                        uniform_int_distribution<> droneDist(0, data.numDrones);
                        uniform_int_distribution<> bitDist(0, 1);
                        
                        for (int j = 0; j < seqLen; j++) {
                            randEnc.truck_assign[j] = truckDist(rng);
                            int c = tabuResult.sequence[j];
                            if (data.isCustomer(c) && data.nodeTypes[c] == "D" && data.readyTimes[c] > 0) {
                                randEnc.drone_assign[j] = droneDist(rng);
                            } else {
                                randEnc.drone_assign[j] = 0;
                            }
                            randEnc.break_bit[j] = bitDist(rng);
                        }
                        
                        PDPSolution msSol = runAssignmentLS(tabuResult.sequence, randEnc, data, 50);
                        double msFit = msSol.totalCost + msSol.totalPenalty;
                        if (msFit < bestTabuFit - 0.01) {
                            bestTabuSol = msSol;
                            bestTabuFit = msFit;
                        }
                    }
                    
                    // Update population if improved
                    if (bestTabuFit < fitness[idx]) {
                        population[idx] = static_cast<const Chromosome&>(bestTabuSol);
                        fitness[idx] = bestTabuFit;
                        
                        cout << "[TABU] Individual " << k << " (rank " << idx 
                             << "): " << fixed << setprecision(2) << bestTabuFit << endl;
                        
                        // Update global best
                        if (bestTabuFit < bestBeforeTabu) {
                            bestSolution = bestTabuSol;
                            bestSequence = bestTabuSol.sequence;
                            bestChromosome = static_cast<const Chromosome&>(bestTabuSol);
                            bestBeforeTabu = bestTabuFit;
                        }
                    }
                } catch (const exception& e) {
                    cerr << "ERROR in Tabu Search for individual " << k << ": " << e.what() << endl;
                }
            }
            
            cout << "[TABU] Best after Tabu+AssignLS: " << fixed << setprecision(2)
                 << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
            
            tabuApplied = true;
            tabuRounds++;
            noImprovementCounter = 0;
            noImprovementEvalCounter = 0;
            
            // Diversity restart: if Tabu didn't improve after 3 rounds, regenerate 80%
            if (tabuRounds >= 3 && bestSolution.totalCost + bestSolution.totalPenalty >= bestBeforeTabu - 0.01) {
                cout << "[DIVERSITY] Restarting 80% of population (keeping top 20%)..." << endl;
                
                // Re-sort after Tabu modifications
                iota(sortedIdx.begin(), sortedIdx.end(), 0);
                sort(sortedIdx.begin(), sortedIdx.end(), 
                     [&fitness](int a, int b) { return fitness[a] < fitness[b]; });
                
                // Keep top 20%, regenerate 80%
                int keepCount = max(2, populationSize / 5);
                
                // Half from structured init, half from perturbation of best
                int regenCount = populationSize - keepCount;
                int fromInit = regenCount / 2;
                int fromPerturb = regenCount - fromInit;
                
                auto newInds = initStructuredPopulationChromosome(fromInit, data, runNumber + generation);
                
                int regenIdx = 0;
                for (int k = keepCount; k < populationSize && regenIdx < regenCount; ++k, ++regenIdx) {
                    int idx = sortedIdx[k];
                    if (regenIdx < fromInit && regenIdx < (int)newInds.size()) {
                        // Structured init with full encoding
                        population[idx] = newInds[regenIdx];
                    } else {
                        // Perturbation of best chromosome
                        Chromosome perturbed = bestChromosome;
                        if (regenIdx % 2 == 0) {
                            perturbed = doubleBridgePerturbation(perturbed, rng);
                        } else {
                            perturbed = ruinRecreatePerturbation(perturbed, rng, 0.4);
                        }
                        population[idx] = perturbed;
                    }
                    PDPSolution sol = evaluateWithCache(population[idx], data, solutionCache);
                    fitness[idx] = sol.totalCost + sol.totalPenalty;
                }
                tabuRounds = 0;
                cout << "[DIVERSITY] Done. Continuing GA..." << endl;
            }
        }
    }
    
    cout << "\n=========================================" << endl;
    cout << "  GA + TABU COMPLETED" << endl;
    cout << "=========================================" << endl;
    
    // Final multi-start Assignment LS on the best solution
    if (!bestSequence.empty()) {
        cout << "Running final multi-start Assignment LS (5 starts) on best solution..." << endl;
        PDPSolution finalSol = evaluateWithCache(bestChromosome, data, solutionCache);
        double bestFit = bestSolution.totalCost + bestSolution.totalPenalty;
        double finalFit = finalSol.totalCost + finalSol.totalPenalty;
        if (finalFit < bestFit - 0.01) {
            bestSolution = finalSol;
            bestFit = finalFit;
            bestChromosome = static_cast<const Chromosome&>(finalSol);
        }
        
        // Multi-start: try 5 random assignment starts
        for (int ms = 0; ms < 5; ms++) {
            AssignmentEncoding randEnc;
            int seqLen = (int)bestSequence.size();
            randEnc.truck_assign.resize(seqLen);
            randEnc.drone_assign.resize(seqLen);
            randEnc.break_bit.resize(seqLen);
            
            uniform_int_distribution<> truckDist(0, data.numTrucks - 1);
            uniform_int_distribution<> droneDist(0, data.numDrones);
            uniform_int_distribution<> bitDist(0, 1);
            
            for (int j = 0; j < seqLen; j++) {
                randEnc.truck_assign[j] = truckDist(rng);
                int c = bestSequence[j];
                if (data.isCustomer(c) && data.nodeTypes[c] == "D" && data.readyTimes[c] > 0) {
                    randEnc.drone_assign[j] = droneDist(rng);
                } else {
                    randEnc.drone_assign[j] = 0;
                }
                randEnc.break_bit[j] = bitDist(rng);
            }
            
            PDPSolution msSol = runAssignmentLS(bestSequence, randEnc, data, 50);
            double msFit = msSol.totalCost + msSol.totalPenalty;
            if (msFit < bestFit - 0.01) {
                bestSolution = msSol;
                bestFit = msFit;
                bestSequence = msSol.sequence;
                bestChromosome = static_cast<const Chromosome&>(msSol);
                cout << "Multi-start " << ms << " improved: " << fixed << setprecision(2) << msFit << endl;
            }
        }
    }
    
    cout << "Final best cost: " << fixed << setprecision(2)
         << bestSolution.totalCost << " (penalty: " << bestSolution.totalPenalty << ")" << endl;
    
    // In thß╗æng k├¬ adaptive cuß╗æi c├╣ng
    cout << "\n[ADAPTIVE STATS]" << endl;
    cout << "Final mutation rate: " << fixed << setprecision(3) << adaptiveParams.currentMutationRate << endl;
    cout << "Crossover success rates: ";
    for (int i = 0; i < 4; ++i) {
        if (adaptiveParams.crossoverUsage[i] > 0) {
            double rate = adaptiveParams.crossoverSuccess[i] / adaptiveParams.crossoverUsage[i];
            cout << "[" << i << "]:" << setprecision(2) << rate << " ";
        }
    }
    cout << endl;
    cout << "Mutation success rates: ";
    for (int i = 0; i < 5; ++i) {
        if (adaptiveParams.mutationUsage[i] > 0) {
            double rate = adaptiveParams.mutationSuccess[i] / adaptiveParams.mutationUsage[i];
            cout << "[" << i << "]:" << setprecision(2) << rate << " ";
        }
    }
    cout << endl;
    
    return bestSolution;
}
