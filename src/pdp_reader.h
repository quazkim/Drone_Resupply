#ifndef PDP_READER_H
#define PDP_READER_H

#include "pdp_types.h"
#include <string>

/**
 * @file pdp_reader.h
 * @brief I/O utilities for parsing PDP instance files.
 *
 * NODE ID CONVENTIONS (gene encoding):
 *   Gene(node_id  > 0)  →  Customer node  (0-based array index: 1 .. numNodes-1)
 *   Gene(node_id == 0)  →  Separator       (vách ngăn chia Truck 1 / Truck 2)
 *   Gene(node_id == -1) →  Depot Return    (lệnh xe quay về depot ảo giữa hành trình)
 *
 * PHYSICAL ARRAY LAYOUT (0-based):
 *   Index 0             →  Depot vật lý (depotIndex = 0)
 *   Index 1 .. N-1      →  Customer nodes (P, DL, D)
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
 * Includes the encoding mapping block:
 *   [ENCODING MAPPING] Gene(0)->Separator | Gene(-1)->Depot Return | Gene(>0)->Customers
 *
 * @param data  Populated PDPData
 */
void showPDPInfo(const PDPData& data);

#endif // PDP_READER_H
