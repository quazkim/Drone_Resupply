#ifndef PDP_LSP_H
#define PDP_LSP_H

#include "pdp_types.h"
#include <vector>

// ============================================================
// Layer 2: Loading Subproblem (LSP)
// Given fixed truck routes (VRPSolution), decide:
//   - Which C1 packages go in initial depot load
//   - Which go via drone resupply (at which customer node)
// Objective: minimize C_max (makespan)
// ============================================================

// Greedy warm start: load C1 packages at depot while capacity allows;
// overflow packages go to drone resupply at their own delivery node.
// Always returns a feasible SolutionEncoding.
SolutionEncoding greedyLoading(const PDPData& data, const VRPSolution& vrp);

// CP-SAT exact solver for the LSP.
//   vrp        : fixed customer routes (no loading info)
//   warm_start : greedy solution used as initial hint (AddHint)
//   cutoff_sec : CP-SAT time limit in seconds (default 5.0)
//   ub_cutoff  : if known best C_max is ub_cutoff, only accept strictly better
// Returns LSPResult with solved=true if CP-SAT finds/proves optimum within cutoff.
LSPResult solveLSP(const PDPData& data,
                   const VRPSolution& vrp,
                   const SolutionEncoding& warm_start,
                   double cutoff_sec = 5.0,
                   double ub_cutoff = 1e18);

// Convenience: run greedy + CP-SAT in one call.
LSPResult evaluateRoutes(const PDPData& data,
                         const VRPSolution& vrp,
                         double cutoff_sec = 5.0,
                         double ub_cutoff = 1e18);

#endif
