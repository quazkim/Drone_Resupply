#include "config.h"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include "vrp_parser.h"
#include "solution.h"
#include "tabu.h"
#include "insertion.h"
#include "untils.h"
#include "initial_solution.h"

using namespace std;
const int max_iterations = 10;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        cerr << "Usage: " << argv[0] << " path_to_vrp_folder instanceName(CMT1..CMT14)" << endl;
        return 1;
    }

    string base_path = argv[1];
    string instance_name = argv[2];
    if (base_path.back() != '/' && base_path.back() != '\\')
        base_path += "/";

    string filename = base_path + instance_name + ".vrp";

    // Chạy cả 2 mode cho 1 instance
    for (bool penalty_mode : {true, false})
    {
        USE_PENALTY = penalty_mode;
        cout << "\n=============================\n";
        cout << "Instance: " << filename
             << " | Mode: " << (USE_PENALTY ? "Penalty" : "Hard") << "\n";
        cout << "=============================\n";

        for (int run = 1; run <= 10; ++run)
        {
            VRPInstance vrp;
            if (!read_vrp_file(filename, vrp))
            {
                cerr << "Failed to read instance " << filename << "\n";
                continue;
            }

            if (!read_vehicle_count(instance_name, vrp))
            {
                cerr << "Failed to read vehicle count for " << instance_name << "\n";
                continue;
            }

            Solution sol;
            initial_solution(vrp, sol, vrp.vehicle_number);
            evaluate_solution(sol, vrp, alpha, beta);
            print_solution(sol,vrp)       ;


            auto start = chrono::high_resolution_clock::now();
            TabuSearch tabu(vrp, max_iterations);
            Solution best = tabu.run(sol);
            auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> elapsed = end - start;

            if (best.total_cost == std::numeric_limits<double>::infinity())
            {
                cout << "Run " << run << " | No feasible solution\n";
            }
            else
            {
                cout << "Run " << run
                     << " | Cost = " << best.total_cost
                     << " | Time = " << elapsed.count() << "s\n";
            }

            string outname = instance_name +
                             (USE_PENALTY ? "_penalty_" : "_hard_") +
                             "run" + to_string(run) + ".sol";
            int stop_iter = tabu.get_last_iter();
            int tenure_used = tabu.get_tabu_tenure();

            write_solution_to_file(best, outname, filename, elapsed.count(),
                                   vrp.vehicle_number, USE_PENALTY, alpha, beta,
                                   stop_iter, tenure_used, vrp);
        }
    }
    return 0;
}
