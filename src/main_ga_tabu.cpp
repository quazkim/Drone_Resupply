/**
 * @file main_ga_tabu.cpp
 * @brief Entry point for the GA + Tabu Search solver.
 *        Sử dụng kiến trúc mã hóa vector<Gene> (Chromosome).
 *
 * Usage: ./main_ga_tabu <instance_file> [--depot 0|1|2]
 *   --depot 0  = center (default)
 *   --depot 1  = border
 *   --depot 2  = outside
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "pdp_ga.h" // geneticAlgorithmPDP  (returns PDPSolution, uses Chromosome internally)
#include "pdp_reader.h"
#include "pdp_types.h"
#include "pdp_utils.h"      // printSolution, printChromosome
#include "pdp_validation.h" // validateSolution, printSolutionSummary

using namespace std;

int main(int argc, char *argv[]) {
  auto startTotal = chrono::high_resolution_clock::now();

  // ---- Solver parameters (edit here) ----
  const int POPULATION_SIZE = 500;
  const int MAX_GENERATIONS = 500;
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

  // Parse --depot MODE
  int depotMode = 0;
  for (int i = 2; i < argc; ++i) {
    if (string(argv[i]) == "--depot" && i + 1 < argc) {
      istringstream ss(argv[++i]);
      if (!(ss >> depotMode) || depotMode < 0 || depotMode > 2) {
        cerr << "Error: --depot MODE must be 0, 1, or 2\n";
        return 1;
      }
    }
  }

  const string depotNames[] = {"center", "border", "outside"};

  cout << "\n+========================================================+\n"
       << "|     PDP SOLVER - GA + TABU SEARCH (Gene-based)        |\n"
       << "+========================================================+\n"
       << "Instance : " << instanceFile << "\n"
       << "Depot    : " << depotNames[depotMode] << " (mode " << depotMode
       << ")\n";

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

  // ---- Scale detection ----
  bool isSmallScale = (data.numCustomers + 1 <= 20);
  if (isSmallScale) {
    populationSize = 400;
    maxGenerations = 300;
    cout << "\n[SCALE] Small scale (customers <= 19): "
         << "pop=" << populationSize << " gen=" << maxGenerations << "\n";
  } else {
    cout << "\n[SCALE] Large scale (customers > 19): "
         << "pop=" << populationSize << " gen=" << maxGenerations << "\n";
  }

  // ---- Run GA + Tabu ----
  // geneticAlgorithmPDP manages Chromosome (vector<Gene>) internally.
  // It returns a PDPSolution whose original_sequence is a Chromosome.
  PDPSolution solution =
      geneticAlgorithmPDP(data, populationSize, maxGenerations, mutationRate,
                          runNumber, isSmallScale);

  double costGA = solution.totalCost;

  // ---- Print result ----
  cout << "\n+========================================================+\n"
       << "|               GA + TABU RESULT                        |\n"
       << "+========================================================+\n"
       << "C_max (GA+Tabu) : " << fixed << setprecision(2) << costGA << " min\n"
       << "Penalty         : " << solution.totalPenalty << "\n"
       << "Feasible        : " << (solution.isFeasible ? "YES" : "NO") << "\n";

  // ---- Print chromosome encoding (Gene-based, no raw vector<int> loop) ----
  cout << "\n CHROMOSOME (solution.original_sequence):\n";
  printChromosome(solution.original_sequence);

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
