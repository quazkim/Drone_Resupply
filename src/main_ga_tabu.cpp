/**
 * @file main_ga_tabu.cpp
 * @brief Entry point for the GA solver (Truck--Drone Resupply encoding).
 *        Encoding follows 0[P] and i[P] as in the MD specs.
 *
 * Usage: ./main_ga_tabu <instance_file> [--depot 0|1|2]
 *   --depot 0  = center (default)
 *   --depot 1  = border
 *   --depot 2  = outside
 */

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "pdp_ga.h" // geneticAlgorithmPDP (MD encoding)
#include "pdp_init.h" // initStructuredPopulationPDP
#include "pdp_reader.h"
#include "pdp_types.h"
#include "pdp_utils.h"      // printSolution, printEncodedSolution
#include "pdp_validation.h" // validateSolution, printSolutionSummary

using namespace std;

int main(int argc, char *argv[]) {
  auto startTotal = chrono::high_resolution_clock::now();

  // ---- Solver parameters (edit here) ----
  const int POPULATION_SIZE = 500;
  const int MAX_GENERATIONS = 3000;
  const double MUTATION_RATE = 0.15;
  const int RUN_NUMBER = 1;

  // ---- Usage check ----
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <instance_file> [--depot MODE]\n"
         << "Depot modes: 0=center (default)  1=border  2=outside\n"
         << "Examples:\n"
         << "  " << argv[0] << " Instance/U_10_0.5_Num_1.txt\n"
         << "  " << argv[0] << " Instance/U_30_0.5_Num_1.txt --depot 1\n"
         << "  " << argv[0] << " Instance/U_50_1.0_Num_1.txt --depot 2\n"
         << "\nCurrent parameters:\n"
         << "  Population: " << POPULATION_SIZE << "\n"
         << "  Generations: " << MAX_GENERATIONS << "\n"
         << "  Mutation rate: " << MUTATION_RATE << "\n"
         << "  Run number: " << RUN_NUMBER << "\n";
    return 1;
  }

  const string instanceFile = argv[1];
  int populationSize = POPULATION_SIZE;
  int maxGenerations = MAX_GENERATIONS;
  double mutationRate = MUTATION_RATE;
  int runNumber = RUN_NUMBER;
    bool populationSizeSet = false;
    bool maxGenerationsSet = false;
    string logPath;
    bool logEnabled = false;
    ofstream logFile;

  // Parse --depot MODE
  int depotMode = 0;
  for (int i = 2; i < argc; ++i) {
    if (string(argv[i]) == "--depot" && i + 1 < argc) {
      istringstream ss(argv[++i]);
      if (!(ss >> depotMode) || depotMode < 0 || depotMode > 2) {
        cerr << "Error: --depot MODE must be 0, 1, or 2\n";
        return 1;
      }
      } else if (string(argv[i]) == "--pop" && i + 1 < argc) {
        istringstream ss(argv[++i]);
        if (!(ss >> populationSize) || populationSize <= 0) {
          cerr << "Error: --pop must be a positive integer\n";
          return 1;
        }
        populationSizeSet = true;
      } else if (string(argv[i]) == "--gen" && i + 1 < argc) {
        istringstream ss(argv[++i]);
        if (!(ss >> maxGenerations) || maxGenerations < 0) {
          cerr << "Error: --gen must be an integer >= 0\n";
          return 1;
        }
        maxGenerationsSet = true;
      } else if (string(argv[i]) == "--mut" && i + 1 < argc) {
        istringstream ss(argv[++i]);
        if (!(ss >> mutationRate) || mutationRate < 0.0 || mutationRate > 1.0) {
          cerr << "Error: --mut must be in [0,1]\n";
          return 1;
        }
      } else if (string(argv[i]) == "--run" && i + 1 < argc) {
        istringstream ss(argv[++i]);
        if (!(ss >> runNumber) || runNumber <= 0) {
          cerr << "Error: --run must be a positive integer\n";
          return 1;
        }
      } else if (string(argv[i]) == "--log" && i + 1 < argc) {
        logPath = argv[++i];
        logEnabled = true;
      } else {
        cerr << "Error: Unknown or incomplete argument: " << argv[i] << "\n"
             << "Supported options:\n"
             << "  --depot 0|1|2\n"
             << "  --pop N\n"
             << "  --gen G\n"
             << "  --mut R\n"
             << "  --run K\n";
        return 1;
    }
  }

  const string depotNames[] = {"center", "border", "outside"};

  cout << "\n+========================================================+\n"
      << "|     PDP SOLVER - GENETIC ALGORITHM (MD encoding)      |\n"
       << "+========================================================+\n"
       << "Instance : " << instanceFile << "\n"
       << "Depot    : " << depotNames[depotMode] << " (mode " << depotMode
       << ")\n";

  if (logEnabled) {
    logFile.open(logPath);
    if (!logFile.is_open()) {
      cerr << "Error: cannot open log file: " << logPath << "\n";
      return 1;
    }
    cout << "Log file : " << logPath << "\n";
  }

  // ---- Load instance ----
  PDPData data;
  data.depotMode = depotMode;
  if (!readPDPFile(instanceFile, data)) {
    cerr << "Error: Failed to read instance file!\n";
    return 1;
  }
  showPDPInfo(data);

  cout << "Instance details:\n"
       << "  Customers : " << data.numCustomers << "\n"
       << "  Trucks    : " << data.numTrucks << "\n"
       << "  Drones    : " << data.numDrones << "\n"
       << "  Endurance : " << data.droneEndurance << " min\n"
       << "  Depot pos : (" << data.coordinates[data.depotIndex].first << ", "
       << data.coordinates[data.depotIndex].second << ")\n";

  // ---- Scale detection (auto-tune unless user overrides) ----
  bool isSmallScale = (data.numCustomers + 1 <= 20);
  if (!populationSizeSet && !maxGenerationsSet) {
    if (isSmallScale) {
      populationSize = 400;
      maxGenerations = 3000;
      cout << "\n[SCALE] Small scale (customers <= 19): "
           << "pop=" << populationSize << " gen=" << maxGenerations << "\n";
    } else {
      cout << "\n[SCALE] Large scale (customers > 19): "
           << "pop=" << populationSize << " gen=" << maxGenerations << "\n";
    }
  } else {
    cout << "\n[SCALE] Manual: pop=" << populationSize
         << " gen=" << maxGenerations
         << " mut=" << mutationRate
         << " run=" << runNumber << "\n";
  }

  // ---- Run GA ----
    vector<SolutionEncoding> initPopulation =
      initStructuredPopulationPDP(populationSize, data, runNumber);
    PDPSolution solution =
      geneticAlgorithmPDP(data, initPopulation,
          populationSize, maxGenerations, mutationRate,
          runNumber, isSmallScale,
          logEnabled ? (std::ostream*)&logFile : nullptr);

  double costGA = solution.totalCost;

  // ---- Print result ----
  cout << "\n+========================================================+\n"
       << "|               GA + TABU RESULT                        |\n"
       << "+========================================================+\n"
       << "C_max (GA+Tabu) : " << fixed << setprecision(2) << costGA << " min\n"
       << "Penalty         : " << solution.totalPenalty << "\n"
       << "Feasible        : " << (solution.isFeasible ? "YES" : "NO") << "\n";

  // ---- Print chromosome encoding (Gene-based, no raw vector<int> loop) ----
  cout << "\n ENCODING (solution.encoded_routes):\n";
  printEncodedSolution(solution.encoded_routes);

  // ---- Full solution detail ----
  cout << "\n+========================================================+\n"
       << "|               FINAL SOLUTION                          |\n"
       << "+========================================================+\n";
  printSolution(solution, data);

  // ---- Validation ----
  validateSolution(solution, data, /*verbose=*/true);

  // ---- Summary ----
  printSolutionSummary(solution, costGA, solution.totalCost);

  // ---- Total runtime ----
  double totalSec = chrono::duration<double>(
                        chrono::high_resolution_clock::now() - startTotal)
                        .count();
  cout << "\n[TOTAL RUNTIME] " << fixed << setprecision(2) << totalSec
       << " seconds\n";

  return 0;
}
