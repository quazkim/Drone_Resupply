#ifndef PDP_READER_H
#define PDP_READER_H

#include "pdp_types.h"
#include <string>

/**
 * @file pdp_reader.h
 * @brief I/O utilities for parsing PDP instance files.
 *
 * NODE/INDEX CONVENTIONS:
 *   Physical array (0-based): index 0 = depot, 1..N-1 = customers
 *
 * SOLUTION ENCODING (per MD specs):
 *   0[P]  -> depot load P
 *   i     -> serve customer i
 *   i[P]  -> drone resupply P at customer i (before serving i)
 *   0     -> return to depot and end route
 */

// ---- Distance helpers ----

/**
 * @brief Manhattan distance |x1-x2| + |y1-y2| (truck routing model).
 */
double manhattanDistance(double x1, double y1, double x2, double y2);

// Note: euclideanDistance is declared in pdp_utils.h and defined in pdp_reader.cpp

/**
 * @brief Construct truck (Manhattan→time) and drone (Euclidean→time) distance
 *        matrices for all nodes [0 .. numNodes-1].
 *
 * Both matrices use 0-based indices: row/col 0 = physical depot.
 * Results are stored in data.truckDistMatrix and data.droneDistMatrix.
 *
 * @param[in,out] data  PDPData with coordinates already populated
 */
void buildAllDistanceMatrices(PDPData& data);

// ---- File I/O ----

/**
 * @brief Parse a PDP instance file and populate PDPData.
 *
 * File format (one node per line after a header line):
 *   <id>  <x>  <y>  <type>  <readyTime>  <pairId>
 * where type ∈ {P, DL, D}.  '#' lines are ignored.
 *
 * After parsing:
 *   - Depot is inserted at array index 0 (depotIndex = 0).
 *   - Customers occupy indices 1 .. N-1.
 *   - Both distance matrices are built automatically.
 *
 * @param filename  Path to instance file
 * @param[out] data PDPData to be populated
 * @return true on success, false if file cannot be opened or is malformed
 */
bool readPDPFile(const std::string& filename, PDPData& data);

/**
 * @brief Print a concise summary of the loaded instance to stdout.
 * Includes the encoding mapping block for the MD encoding.
 *
 * @param data  Populated PDPData
 */
void showPDPInfo(const PDPData& data);

#endif // PDP_READER_H
