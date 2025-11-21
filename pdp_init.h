#ifndef PDP_INIT_H
#define PDP_INIT_H

#include "pdp_types.h"
#include <vector>
#include <random>

using namespace std;

// --- CÁC HÀM TIỆN ÍCH (Public để các module khác có thể dùng) ---
double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id);
double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id);
vector<vector<int>> decodeSeq(const vector<int>& seq, const PDPData& data);

// --- HÀM SỬA CHỮA (REPAIR) ---
void quickRepairPDP(vector<int>& seq, const PDPData& data, mt19937& gen);

// --- CÁC CHIẾN LƯỢC KHỞI TẠO ---
vector<vector<int>> initStructuredPopulationPDP(int populationSize, const PDPData& data, int runNumber = 1);
vector<vector<int>> initRandomPDP(int populationSize, const PDPData& data);
vector<vector<int>> initSweepPDP(int populationSize, const PDPData& data);
vector<vector<int>> initGreedyTimePDP(int populationSize, const PDPData& data);
vector<vector<int>> initNearestNeighborPDP(int populationSize, const PDPData& data);

#endif