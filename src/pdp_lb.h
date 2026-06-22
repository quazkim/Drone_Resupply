#ifndef PDP_LB_H
#define PDP_LB_H

#include "pdp_types.h"
#include <vector>

// Lower bound computation — paper Proposition 2
// a_i = r_i + min(d_i + delta_m, t_0i) + s_i   (earliest node-ready time)
// l_1 = max(a_{chi1}, t_{0,chi1})
// l_k = max(a_{chik}, l_{k-1} + t_{chi_{k-1}, chik})
// l(chi^v) = l_{m_v} + t_{chi_{m_v}, depot}
// LB(chi) = max_v l(chi^v)

// Earliest time node i can be departed from (lower bound on its service completion)
double computeNodeLB(const PDPData& data, int customer);

// LB info for one truck route (sequence of customer ids, 1-indexed, no depot endpoints)
LBInfo computeRouteLB(const PDPData& data, const std::vector<int>& route);

// LB on C_max for the full VRP solution (max over trucks)
double computeVRPLB(const PDPData& data, const VRPSolution& vrp);

// Adaptive threshold: Threshold = (1-phi)*z_star + phi*lb_star
double computeThreshold(double z_star, double lb_star, double phi);

// Returns index of the "last truck" — truck with highest LB completion time.
// Used by TS to pick which truck's tail to relocate from.
int findLastTruckByLB(const PDPData& data, const VRPSolution& vrp);

#endif
