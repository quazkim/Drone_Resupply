#ifndef PDP_READER_H
#define PDP_READER_H

#include "pdp_types.h"
#include <string>

/**
 * @brief Compute Manhattan distance between two 2D points.
 * Used for truck routing (urban/grid-based model).
 * @param x1, y1 Coordinates of first point
 * @param x2, y2 Coordinates of second point
 * @return Manhattan distance |x1-x2| + |y1-y2|
 */
double manhattanDistance(double x1, double y1, double x2, double y2);

/**
 * @brief Compute Euclidean distance between two 2D points.
 * Used for drone routing (straight-line distance).
 * @param x1, y1 Coordinates of first point
 * @param x2, y2 Coordinates of second point
 * @return Euclidean distance sqrt((x1-x2)^2 + (y1-y2)^2)
 */
double euclideanDistance(double x1, double y1, double x2, double y2);

/**
 * @brief Parse a PDP instance file and populate PDPData structure.
 * 
 * File format expected:
 * - Node definitions with coordinates and types (P, DL, D, DEPOT)
 * - Fleet specifications (trucks, drones, capacities, speeds)
 * - Constraints (ready times, endurance limits)
 * 
 * @param filename Path to input PDP instance file
 * @param[out] data PDPData structure to be filled
 * @return true if parsing successful, false if file not found or malformed
 */
bool readPDPFile(const string& filename, PDPData& data);

/**
 * @brief Display comprehensive instance information to console.
 * @param data The PDP instance to display
 */
void showPDPInfo(const PDPData& data);

/**
 * @brief Construct both distance matrices (truck and drone) from node coordinates.
 * - Truck distances: Manhattan (urban grid model)
 * - Drone distances: Euclidean (direct line-of-sight)
 * @param[out] data PDPData with coordinates populated; matrices will be initialized
 */
void buildAllDistanceMatrices(PDPData& data); 

#endif
