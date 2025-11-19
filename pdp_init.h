#ifndef PDP_INIT_H
#define PDP_INIT_H

#include "pdp_types.h"
#include <vector>
#include <random>

using namespace std;

// --- CÁC HÀM KHỞI TẠO TỪ FILE CỦA BẠN ---
void quickRepairPDP(vector<int>& seq, const PDPData& data, mt19937& gen);
vector<vector<int>> initStructuredPopulationPDP(int populationSize, const PDPData& data, int runNumber = 1);
vector<vector<int>> initRandomPDP(int populationSize, const PDPData& data);
vector<vector<int>> initSweepPDP(int populationSize, const PDPData& data);
vector<vector<int>> initGreedyTimePDP(int populationSize, const PDPData& data);
vector<vector<int>> initNearestNeighborPDP(int populationSize, const PDPData& data);

// --- HÀM ĐÁNH GIÁ MỚI (DECODER) ---
// (Sẽ được định nghĩa trong pdp_init.cpp)
PDPSolution decodeAndEvaluate(const vector<int>& seq, const PDPData& data);

// --- CÁC HÀM TIỆN ÍCH NỘI BỘ (cần cho init và decode) ---
// (Khai báo ở đây để `main` hoặc `pdp_ga` có thể gọi `decodeAndEvaluate`)
double getTruckDistance(const PDPData& data, int nodeA_id, int nodeB_id);
double getDroneDistance(const PDPData& data, int nodeA_id, int nodeB_id);
vector<vector<int>> decodeSeq(const vector<int>& seq, const PDPData& data);
double polarAngle(const pair<double, double>& depot, const pair<double, double>& customer, bool normalize);
pair<double,double> simulateRouteMetrics(const vector<int>& customers,
                                        int candidateNode,
                                        const PDPData& data,
                                        const vector<vector<double>>& dist); // Hàm gốc của bạn


#endif