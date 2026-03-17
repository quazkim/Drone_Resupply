# Benchmark System: GA+Tabu vs MILP/Matheuristic

## Overview

This benchmarking system automatically compares the GA+Tabu solver against industry-standard MILP optimal solutions and Matheuristic results from the paper.

## Files

- **scripts/run_benchmarks.py**: Comprehensive Python script to run all instances and generate CSV comparisons
- **.github/workflows/benchmark.yml**: GitHub Actions workflow for automated benchmarking on push/PR
- **.github/workflows/quick-benchmark.yml**: Fast CI check (optional)

## Local Usage

### Quick Test (10-15 customers, ~5 min)
```bash
python3 scripts/run_benchmarks.py --max-customers 15 --sample-size 30
```

### Full Benchmark (All instances, ~1-2 hours)
```bash
python3 scripts/run_benchmarks.py
```

### Custom Run
```bash
python3 scripts/run_benchmarks.py \
  --max-customers 20 \
  --sample-size 50 \
  --output-dir results
```

## Output Files

The script generates CSV files in the `results/` directory:

### 1. `our_results_*.csv`
Our solver's results:
- **Filename**: Instance filename
- **Customers**: Problem size (10, 15, 20, 30, 40, 50, 75, 100)
- **Beta**: Drone capacity multiplier (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
- **Instance**: Instance number (1-10)
- **Depot Location**: Center/Border/Outside (inferred from coordinates)
- **Objective Value**: Final cost (makespan in minutes)
- **Runtime (s)**: Total execution time
- **Feasible**: Success indicator
- **Error**: Error message if applicable

### 2. `comparison_*.csv`
Full comparison with author's MILP optimal:
- All fields from `our_results_*.csv`
- **MILP Objective**: Optimal value from paper
- **Gap to MILP (%)**: Our gap percentage vs optimal

### 3. Performance Analysis
Automatic grouping by:
- **Problem Size**: Performance trends across customer counts
- **Depot Location**: Sensitivity to depot position
- **Beta Value**: Drone capacity impact

## GitHub Actions Integration

### Automatic Triggers
- **On Push**: Quick test on default branch (10-15 customers)
- **On PR**: Quick test with PR comments
- **Manual**: Workflow dispatch for full benchmark

### Running Full Benchmark on GitHub
```bash
# Via GitHub UI:
# Actions → Benchmark → Run workflow → Select "full-benchmark"

# Via GitHub CLI:
gh workflow run benchmark.yml -f test_mode=full-benchmark
```

### Results Retrieval
Results automatically uploaded as artifacts:
```bash
# Download latest artifacts
gh run download [RUN_ID] -n benchmark-results

# Or view in GitHub UI:
# Actions → [Latest run] → Artifacts → benchmark-results
```

## CSV Format

### For 10-customer instances
Matches author format but with your solver name included:
```csv
Filename,Customers,Beta,Instance,Depot Location,Objective Value,Runtime (s),Feasible,Error,MILP Objective,Gap to MILP (%)
U_10_0.5_Num_1.txt,10,0.5,1,Center,138.00,0.34,True,,131,5.34
U_10_0.5_Num_2.txt,10,0.5,2,Center,160.00,0.38,True,,160,0.00
```

## Comparison with Reference Data

### MILP Results (Optimal)
- Source: `reference/VRPRD-DR/Results/Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv`
- 10 customers: 45 instances (100% optimal)
- 15-20 customers: Partial optimal (time-limited MILP)
- 30+ customers: Matheuristic only (MILP infeasible)

### Matheuristic Results
- 10-20 customers: Heuristic solutions with timing
- 30-50 customers: Large-scale benchmark
- 75-100 customers: Extended instances

## Depot Location Inference

The system automatically classifies depot location:
- **Center**: Depot coordinates near centroid of customers
- **Border**: Depot near region boundary
- **Outside**: Depot outside customer region bounds

This matches author's classification without needing manual metadata.

## Performance Metrics

### Key Statistics Computed
- **Avg Gap to MILP**: Average gap percentage across instances
- **Avg Runtime**: Mean execution time by category
- **Success Rate**: Percentage of feasible solutions
- **Scalability**: Performance across problem sizes

### Grouping Options
1. By **Problem Size** (10, 15, 20, 30, 40, 50, 75, 100)
2. By **Depot Location** (Center, Border, Outside)
3. By **Beta** (Drone capacity: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
4. By **Combination** (e.g., 10-customer instances with Center depot)

## Tips for Benchmarking

### Optimize Parameter Tuning
Modify these in `src/pdp_ga.cpp` before running:
```cpp
#define POP_SIZE           200  // Population size
#define MAX_GENERATIONS    500  // GA iterations
#define TABU_ITERATIONS    50   // Tabu search iterations
```

For small instances (≤15 nodes), the solver already enables intensive search:
- POP_SIZE: 40 → 200
- MAX_GENERATIONS: 30 → 500  
- Tabu iterations: 10000

### Reproducible Results
The solver uses:
- Deterministic seeding (controlled randomness)
- Solution caching (consistent evaluations)
- Fixed parameter adaptation rules

To ensure reproducibility across runs, don't modify VND operators or fitness calculations.

### Timeout Strategy
- Quick test: 1-hour total timeout
- Full benchmark: 1-hour per instance
- GitHub Actions: 6-hour job timeout

### Memory Requirements
- ~1.5GB for solution cache (configurable)
- Solution cache size: 150,000 entries
- Increase for larger instances if needed

## Integration with CI/CD

### Build Prerequisites
```bash
# Must compile successfully
make clean && make

# Verify executable exists
[ -f main_ga_tabu ] && echo "✓ Ready"
```

### Test Coverage Matrix

| Feature | Quick Test | Full Benchmark |
|---------|-----------|-----------------|
| 10-customer instances | ✓ | ✓ |
| 15-customer instances | ✓ | ✓ |
| 20-customer instances | ✓ | ✓ |
| 30+ customers | ✗ | ✓ |
| Depot inference | ✓ | ✓ |
| MILP comparison | ✓ | ✓ |

## Troubleshooting

### Build Fails
```bash
make clean
make -j4  # Parallel build
```

### Script Errors
```bash
# Check Python version
python3 --version  # Should be 3.7+

# Install pandas if missing
pip3 install pandas

# Run with verbose output
python3 -u scripts/run_benchmarks.py
```

### Missing Reference Data
Ensure these files exist:
- `reference/VRPRD-DR/Results/Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv`
- `Instances 2/U_*.txt` (all instance files)

### No Results Generated
Check:
1. Executable runs: `./main_ga_tabu Instances\ 2/U_10_0.5_Num_1.txt`
2. Output format: Look for `Final cost:` and `[TOTAL RUNTIME]` in output
3. Results directory exists: `mkdir -p results`

## Next Steps

1. **Run local quick test** to verify setup
2. **Commit workflow file** to enable GitHub Actions
3. **Merge to main** to trigger automated benchmarking
4. **Monitor results** via GitHub Actions → Artifacts
5. **Analyze trends** to identify optimization opportunities

## Contact & References

For questions about the solver implementation, refer to:
- Energy-efficient benchmarking: `docs/benchmark-guide.md`
- Algorithm documentation: `docs/algorithms.md`
- Instance format: `Instances 2/READ ME.txt`

---

**Last Updated**: March 2025 | **Version**: 1.0
