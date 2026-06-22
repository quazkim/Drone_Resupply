/**
 * @file main.cpp
 * @brief Entry point for TS-ALNS-LSP solver (3-layer architecture).
 *
 * Layer 0: GA init population → best SolutionEncoding → extractVRPSolution
 * Layer 1: Tabu Search on VRPSolution (routing only), ALNS escape
 * Layer 2: CP-SAT LSP evaluates each candidate routing (loading)
 *
 * Usage: ./pdp_solver <instance_file> [options]
 *   --depot 0|1|2   depot mode (0=center default)
 *   --iter N        TS max iterations (default 1000)
 *   --lsp T         CP-SAT time limit per call in seconds (default 3.0)
 *   --run K         run number / random seed
 *   --time T        total time limit in seconds
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "pdp_ga.h"         // extractCustomerRoutes
#include "pdp_init.h"       // initStructuredPopulationPDP
#include "pdp_reader.h"
#include "pdp_tabu_main.h"  // tabuSearch, TSConfig, TSResult
#include "pdp_types.h"
#include "pdp_utils.h"
#include "pdp_validation.h"
#include "pdp_fitness.h"    // decode_solution, print_decoded_routes

using namespace std;

int main(int argc, char* argv[]) {
    auto t_start = chrono::high_resolution_clock::now();

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance_file> [--depot 0|1|2]"
             << " [--iter N] [--lsp T] [--run K] [--time T]\n";
        return 1;
    }

    const string instanceFile = argv[1];
    int depotMode    = 0;
    int maxIter      = 1000;
    double lspCutoff = 3.0;
    int runNumber    = 1;
    double timeLimit = -1.0;

    for (int i = 2; i < argc; ++i) {
        string arg = argv[i];
        if ((arg == "--depot" || arg == "--iter" || arg == "--lsp" ||
             arg == "--run"   || arg == "--time") && i + 1 < argc) {
            istringstream ss(argv[++i]);
            if      (arg == "--depot") ss >> depotMode;
            else if (arg == "--iter")  ss >> maxIter;
            else if (arg == "--lsp")   ss >> lspCutoff;
            else if (arg == "--run")   ss >> runNumber;
            else if (arg == "--time")  ss >> timeLimit;
        } else {
            cerr << "Unknown argument: " << arg << "\n";
            return 1;
        }
    }

    // ---- Load instance ----
    PDPData data;
    data.depotMode = depotMode;
    if (!readPDPFile(instanceFile, data)) {
        cerr << "Error: cannot read " << instanceFile << "\n";
        return 1;
    }
    showPDPInfo(data);

    bool isSmall = (data.numCustomers + 1 <= 20);
    if (timeLimit < 0) timeLimit = isSmall ? 150.0 : 3600.0;

    const string depotNames[] = {"center", "border", "outside"};
    cout << "\n+========================================================+\n"
         << "|        PDP SOLVER -- TS-ALNS-LSP (CP-SAT)             |\n"
         << "+========================================================+\n"
         << "Instance : " << instanceFile << "\n"
         << "Depot    : " << depotNames[depotMode] << "\n"
         << "TS iters : " << maxIter << "\n"
         << "LSP cut  : " << lspCutoff << " s\n"
         << "Time lim : " << timeLimit << " s\n\n";

    // ============================================================
    // Layer 0: Init population → extract best VRPSolution
    // ============================================================
    const int POP_SIZE = isSmall ? 40 : 60;  // [Vidal HGS 2022] μ=25, μ+λ=65
    cout << "[L0] Building init population (size=" << POP_SIZE << ")...\n";
    vector<SolutionEncoding> population =
        initStructuredPopulationPDP(POP_SIZE, data, runNumber);

    // Pick best individual from population using greedy decode
    SolutionEncoding best_init_enc;
    double best_init_cost = numeric_limits<double>::infinity();
    for (const auto& enc : population) {
        PDPSolution sol = decode_solution(enc, data, /*throw=*/false);
        if (sol.isFeasible && sol.totalCost < best_init_cost) {
            best_init_cost = sol.totalCost;
            best_init_enc  = enc;
        }
    }
    if (best_init_enc.empty()) {
        // Fallback: take first individual
        best_init_enc = population[0];
        best_init_cost = decode_solution(best_init_enc, data, false).totalCost;
    }

    VRPSolution init_vrp = extractVRPSolution(best_init_enc);
    cout << "[L0] Best init C_max=" << fixed << setprecision(2)
         << best_init_cost << "\n\n";

    // ============================================================
    // Layer 1+2: Tabu Search + CP-SAT LSP
    // ============================================================
    TSConfig cfg;
    cfg.lsp_cutoff = lspCutoff;
    cfg.seed       = runNumber * 137;
    cfg.verbose    = true;

    // [V3] Adaptive parameters based on problem size
    cfg.eta_max = isSmall ? 150 : 200;
    if (data.numCustomers >= 18 && cfg.lsp_cutoff < 5.0)
        cfg.lsp_cutoff = 5.0;

    // FIX 3: Time-based termination — use remaining budget after GA init [paper §5.2]
    {
        auto now = chrono::high_resolution_clock::now();
        double used = chrono::duration<double>(now - t_start).count();
        cfg.time_limit_sec = max(10.0, timeLimit - used - 2.0);
    }
    // When time limit governs, use a large iteration cap (not the --iter default of 1000)
    cfg.max_iter = (cfg.time_limit_sec > 0) ? 10000000 : maxIter;

    cout << "[L1] Starting Tabu Search...\n";
    TSResult ts = tabuSearch(data, init_vrp, cfg);

    // ============================================================
    // Results
    // ============================================================
    cout << "\n+========================================================+\n"
         << "|               TS-ALNS-LSP RESULT                      |\n"
         << "+========================================================+\n"
         << "C_max (TS+LSP) : " << fixed << setprecision(2) << ts.best_obj << " min\n"
         << "TS iterations  : " << ts.iterations_done << "\n";

    if (ts.best_lsp.feasible && !ts.best_lsp.encoding.empty()) {
        PDPSolution final_sol = decode_solution(ts.best_lsp.encoding, data, false);
        cout << "Feasible       : " << (final_sol.isFeasible ? "YES" : "NO") << "\n";
        cout << "Penalty        : " << final_sol.totalPenalty << "\n";

        cout << "\n[ENCODING]\n";
        printEncodedSolution(ts.best_lsp.encoding);

        cout << "\n[SOLUTION DETAIL]\n";
        print_decoded_routes(final_sol, data);
        printSolution(final_sol, data);
        validateSolution(final_sol, data, /*verbose=*/true);
        printSolutionSummary(final_sol, best_init_cost, ts.best_obj);
    } else {
        cout << "No feasible solution found.\n";
    }

    double elapsed = chrono::duration<double>(
        chrono::high_resolution_clock::now() - t_start).count();
    cout << "\n[RUNTIME] " << fixed << setprecision(2) << elapsed << " s\n";

    return 0;
}
