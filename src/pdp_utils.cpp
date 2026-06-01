#include "pdp_utils.h"

#include "pdp_fitness.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

double computeLowerBound(const PDPData& data) {
    double lb = 0.0;
    const int depot = data.depotIndex;
    for (int i = 1; i < data.numNodes; ++i) {
        if (!data.isCustomer(i)) continue;
        if (i < (int)data.nodeTypes.size() && data.nodeTypes[i] != "D") continue;

        const double ri       = (double)data.readyTimes[i];
        const double droneTo  = data.droneDistMatrix[depot][i];
        const double truckTo  = data.truckDistMatrix[depot][i];
        const double truckBack= data.truckDistMatrix[i][depot];
        const double svc      = data.truckServiceTime;

        // Option A: drone resupplies truck at node i
        double droneOption = max(truckTo, ri + droneTo) + data.resupplyTime + svc + truckBack;
        // Option B: truck carries package from depot (waits until r_i)
        double truckCarry  = ri + truckTo + svc + truckBack;

        lb = max(lb, min(droneOption, truckCarry));
    }
    return lb;
}

double euclideanDistance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

bool validateEncodingConstraints(const SolutionEncoding& encoded, const PDPData& data) {
    unordered_map<int, int> servedCount;

    for (const auto& route : encoded) {
        vector<int> customers;
        customers.reserve(route.size());
        for (const auto& st : route) {
            if (st.node > 0) customers.push_back(st.node);
        }
        unordered_map<int, int> pos;
        for (int i = 0; i < (int)customers.size(); ++i) pos[customers[i]] = i;

        for (const auto& st : route) {
            if (st.node > 0) servedCount[st.node]++;
        }

        // Resupply ordering within the same route
        for (const auto& st : route) {
            if (st.node <= 0 || st.packages.empty()) continue;
            auto itH = pos.find(st.node);
            if (itH == pos.end()) return false;
            const int posH = itH->second;

            for (int p : st.packages) {
                if (!data.isCustomer(p)) return false;
                auto itP = pos.find(p);
                if (itP == pos.end()) return false;
                if (itP->second < posH) return false;
            }
        }
    }

    // Each customer must be served exactly once.
    for (int i = 1; i < data.numNodes; ++i) {
        if (!data.isCustomer(i)) continue;
        if (servedCount[i] != 1) return false;
    }

    return true;
}

static void printPackages(const vector<int>& pkgs) {
    cout << "[";
    for (size_t i = 0; i < pkgs.size(); ++i) {
        cout << pkgs[i];
        if (i + 1 < pkgs.size()) cout << ",";
    }
    cout << "]";
}

void printEncodedSolution(const SolutionEncoding& encoded) {
    cout << "\n[ENCODED SOLUTION]\n";
    for (size_t k = 0; k < encoded.size(); ++k) {
        cout << "Route_" << (k + 1) << " = ";
        const auto& r = encoded[k];
        cout << "[";
        for (size_t i = 0; i < r.size(); ++i) {
            const auto& st = r[i];
            if (st.node == 0) {
                cout << 0;
                if (!st.packages.empty()) {
                    cout << "";
                    printPackages(st.packages);
                }
            } else {
                cout << st.node;
                if (!st.packages.empty()) {
                    cout << "";
                    printPackages(st.packages);
                }
            }
            if (i + 1 < r.size()) cout << ", ";
        }
        cout << "]\n";
    }
}

void printSolution(const PDPSolution& solution, const PDPData& data) {
    (void)data;

    cout << "\n+========================================================+\n"
         << "|                 FINAL SOLUTION                         |\n"
         << "+========================================================+\n";

    cout << "C_max            : " << fixed << setprecision(2) << solution.totalCost << " min\n"
         << "Penalty          : " << solution.totalPenalty << "\n"
         << "Feasible         : " << (solution.isFeasible ? "YES" : "NO") << "\n";

    printEncodedSolution(solution.encoded_routes);
    print_decoded_routes(solution, data);
}
