#include "pdp_init.h"

#include <algorithm>
#include <numeric>
#include <unordered_set>

using namespace std;

static double truckTravelMinutes(const PDPData& data, int a, int b) {
    if (a < 0 || a >= data.numNodes || b < 0 || b >= data.numNodes) {
        return 1e18;
    }
    return data.truckDistMatrix[a][b];
}

static double pkgRelease(const PDPData& data, int p) {
    if (p <= 0 || p >= (int)data.readyTimes.size()) return 0.0;
    return (double)data.readyTimes[p];
}

static double pkgWeight(const PDPData& data, int p) {
    if (p <= 0 || p >= (int)data.demands.size()) return 0.0;
    return (double)data.demands[p];
}

static vector<int> listCustomers(const PDPData& data) {
    vector<int> c;
    for (int i = 1; i < data.numNodes; ++i) {
        if (data.isCustomer(i)) c.push_back(i);
    }
    return c;
}

static void ensureDepotStops(Route& route) {
    if (route.empty() || route.front().node != 0) {
        route.insert(route.begin(), RouteStop(0));
    }
    if (route.back().node != 0) {
        route.push_back(RouteStop(0));
    }
    // Route must end with 0 (no packages)
    route.back().packages.clear();
}

static void buildDepotLoadsForRoute(Route& route, const PDPData& data) {
    ensureDepotStops(route);

    unordered_set<int> onboard;
    double onboardLoad = 0.0;

    double t = 0.0;
    int pos = data.depotIndex;

    auto loadAtDepot = [&](size_t depotIdx) {
        // We're assumed to be at depot at time t.
        vector<int> P;
        double remaining = (double)data.truckCapacity - onboardLoad;

        // If this depot stop was inserted as 0[{i}], treat those as required packages.
        // Wait at depot until required packages are released.
        double requiredReleaseTime = t;
        for (int req : route[depotIdx].packages) {
            requiredReleaseTime = max(requiredReleaseTime, pkgRelease(data, req));
        }
        if (requiredReleaseTime > t + 1e-9) {
            t = requiredReleaseTime; // wait
        }

        // Load required packages first.
        for (int req : route[depotIdx].packages) {
            if (req <= 0) continue;
            if (onboard.count(req)) continue;
            const double w = pkgWeight(data, req);
            if (w <= remaining + 1e-9) {
                P.push_back(req);
                remaining -= w;
                onboard.insert(req);
                onboardLoad += w;
            }
        }

        // Then greedily load upcoming packages that are released by time t.
        for (size_t j = depotIdx + 1; j < route.size(); ++j) {
            int node = route[j].node;
            if (node <= 0) continue;
            int pkg = node;
            if (onboard.count(pkg)) continue;
            if (pkgRelease(data, pkg) > t + 1e-9) continue;

            const double w = pkgWeight(data, pkg);
            if (w <= remaining + 1e-9) {
                P.push_back(pkg);
                remaining -= w;
                onboard.insert(pkg);
                onboardLoad += w;
            }
        }

        route[depotIdx].packages = std::move(P);

        if (!route[depotIdx].packages.empty()) {
            // Depot loading time
            t += data.depotReceiveTime;
        }
    };

    // Pass 1: simulate left-to-right; when serving a customer without package, insert depot return 0[i]
    for (size_t idx = 0; idx < route.size(); ++idx) {
        RouteStop& st = route[idx];

        if (st.node == 0) {
            // Move to depot if needed
            if (pos != data.depotIndex) {
                t += truckTravelMinutes(data, pos, data.depotIndex);
                pos = data.depotIndex;
            }

            // Wait until all packages that will be loaded here are released (we decide what to load at time t).
            loadAtDepot(idx);
            continue;
        }

        const int i = st.node;
        // Travel to customer
        t += truckTravelMinutes(data, pos, i);
        pos = i;

        // If package i not onboard, insert a depot return right before serving i
        if (!onboard.count(i)) {
            // Insert depot stop 0[{i}] at position idx
            RouteStop depot(0, vector<int>{i});
            route.insert(route.begin() + (long)idx, depot);

            // Simulate: go from current pos (i) back to depot, wait release, load, then go to i again.
            // To keep simulation consistent, we rewind: we are currently at i at time t, but
            // the inserted depot stop should happen BEFORE traveling to i.
            // Simplest fix: restart the whole simulation after insertion.
            idx = (size_t)-1;

            onboard.clear();
            onboardLoad = 0.0;
            t = 0.0;
            pos = data.depotIndex;
            continue;
        }

        // Serve i
        t += data.truckServiceTime;
        onboard.erase(i);
        onboardLoad -= pkgWeight(data, i);
    }

    // Ensure last depot is end depot 0 (no packages)
    if (!route.empty() && route.back().node == 0) {
        route.back().packages.clear();
    }
}

static SolutionEncoding buildIndividual(const PDPData& data, mt19937& gen) {
    vector<int> customers = listCustomers(data);
    shuffle(customers.begin(), customers.end(), gen);

    const int nTrucks = max(1, data.numTrucks);
    SolutionEncoding sol;
    sol.assign(nTrucks, Route{});

    const int N = (int)customers.size();
    const int base = N / nTrucks;
    const int rem = N % nTrucks;

    int offset = 0;
    for (int k = 0; k < nTrucks; ++k) {
        const int len = base + (k < rem ? 1 : 0);
        Route route;
        route.reserve((size_t)len + 2);
        route.push_back(RouteStop(0));
        for (int j = 0; j < len; ++j) {
            route.push_back(RouteStop(customers[offset + j]));
        }
        route.push_back(RouteStop(0));
        offset += len;

        buildDepotLoadsForRoute(route, data);
        sol[k] = std::move(route);
    }

    return sol;
}

std::vector<SolutionEncoding> initStructuredPopulationPDP(int populationSize,
                                                         const PDPData& data,
                                                         int runNumber) {
    std::vector<SolutionEncoding> pop;
    pop.reserve((size_t)max(0, populationSize));

    std::seed_seq seed{runNumber, data.numNodes, data.numTrucks, data.numDrones};
    std::mt19937 gen(seed);

    for (int i = 0; i < populationSize; ++i) {
        pop.push_back(buildIndividual(data, gen));
    }

    return pop;
}
