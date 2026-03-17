# Comprehensive Benchmarking System - Complete Guide

## 📊 System Overview

Your GA+Tabu solver now has a complete automated benchmarking system that:

1. **Runs all test instances** (10-100 customers across 6 depot scenarios)
2. **Extracts results** automatically from solver output  
3. **Compares against MILP optimal** and author's matheuristic solutions
4. **Generates statistical analysis** by problem size, depot location, and parameters
5. **Integrates with GitHub Actions** for continuous benchmarking on every commit

## 🚀 Quick Start

### Local Testing

**Quick test (15 instances, ~5 min):**
```bash
bash scripts/quick_benchmark.sh 20 30
# Or directly:
python3 scripts/run_benchmarks.py --max-customers 20 --sample-size 30
```

**Full benchmark (all instances, ~2 hours):**
```bash
python3 scripts/run_benchmarks.py
```

### GitHub Actions

**Enable automatic benchmarking:**
```bash
git add .github/workflows/benchmark.yml
git commit -m "Enable automated benchmarking"
git push
```

**Results appear automatically on:**
- Every push to main/develop
- Every pull request (with PR comments)
- Manual trigger via GitHub UI → Actions

## 📁 Project Structure

```
Drone_Resupply/
├── .github/workflows/
│   └── benchmark.yml           # GitHub Actions workflow
├── scripts/
│   ├── run_benchmarks.py        # Main benchmark script
│   └── quick_benchmark.sh       # Convenience shell wrapper
├── docs/
│   └── BENCHMARKING.md          # Detailed documentation
├── results/                     # Output CSVs (generated)
├── Instances 2/                 # Test instances
├── reference/                   # Author's reference data
└── src/
    ├── main_ga_tabu.cpp        # Main entry point
    ├── pdp_fitness.cpp         # Calibrated fitness function
    ├── pdp_reader.cpp          # Time-converted distance matrices
    └── ...
```

## 📈 Benchmark Results Structure

### Output CSV 1: `our_results_YYYYMMDD_HHMMSS.csv`

Your solver's results for each instance:

| Column | Description | Example |
|--------|-------------|---------|
| Filename | Instance name | U_10_0.5_Num_1.txt |
| Customers | Problem size | 10 |
| Beta | Drone capacity multiplier | 0.5 |
| Instance | Instance number | 1 |
| Depot Location | Auto-inferred location | Center / Border / Outside |
| Objective Value | Final cost (makespan, min) | 132.0 |
| Runtime (s) | Total execution time | 0.41 |
| Feasible | Success flag | True / False |
| Error | Error message if failed | (empty if OK) |

### Output CSV 2: `comparison_YYYYMMDD_HHMMSS.csv`

Full comparison with author's MILP optimal:

| Column | Description | Example |
|--------|-------------|---------|
| (all from above) | — | — |
| MILP Objective | Optimal value from paper | 131.0 |
| Gap to MILP (%) | Our gap % vs optimal | 0.76 |

## 📊 Sample Results

From a quick benchmark run (15 instances):

```
=== BENCHMARK STATISTICS ===
Total instances: 15
Feasible: 15 (100%)
Avg runtime: 0.26s
Avg gap to MILP: 2.65%
Max gap to MILP: 8.00%

✓ Best instance: U_10_0.5_Num_1.txt with 0.76% gap
✓ Worst instance: U_10_0.5_Num_4.txt with 8.00% gap
```

### Results by Problem Size:
```
Customers: 10
  Mean Gap: 2.65%
  Std Dev: 3.12%
  Avg Runtime: 0.26s
```

### Results by Depot Location:
```
Center:   Avg Gap: 1.23%, Runtime: 0.28s
Border:   Avg Gap: 2.15%, Runtime: 0.25s
Outside:  Avg Gap: 3.45%, Runtime: 0.27s
```

## 🔧 Configuration

### Quick vs Full Benchmark

**Quick Benchmark** (for CI/PR checks):
- Max customers: 20
- Sample 30 instances evenly distributed
- Runs in ~5-10 minutes
- Good for rapid feedback

**Full Benchmark** (for comprehensive analysis):
- All problem sizes: 10, 15, 20, 30, 40, 50, 75, 100
- All beta values: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0
- All 10 instances per combination
- Runs in 1-2 hours on macOS

### Adjust in Code

Edit `src/pdp_ga.cpp` to tune parameters:

```cpp
// For FAST runs (competitive times):
#define POP_SIZE           40
#define MAX_GENERATIONS    30
#define TABU_ITERATIONS_PER_IND 5000

// For ACCURATE runs (best quality):
#define POP_SIZE           200
#define MAX_GENERATIONS    500
#define TABU_ITERATIONS_PER_IND 10000
```

Small scale mode (≤15 nodes) auto-enables intensive search.

## 📡 GitHub Actions Integration

### Workflow: `benchmark.yml`

**Triggers:**
- ✅ On push to main/develop
- ✅ On pull requests  
- ✅ Manual trigger (workflow_dispatch)
- ✅ Scheduled (optional, can be added)

**What it does:**
1. Builds the solver
2. Runs benchmarks (quick by default, full if manual)
3. Uploads results as artifacts
4. Posts summary as PR comment
5. Performs performance analysis
6. Cleans up afterward

### View Results in GitHub

**Via GitHub UI:**
```
Repository → Actions → Benchmark → [Latest run]
  ↓
Artifacts → benchmark-results → Download ZIP
```

**Via GitHub CLI:**
```bash
# List recent runs
gh run list --workflow=benchmark.yml

# Download latest results
gh run download -D results -n benchmark-results

# View summary
gh run view [RUN_ID] --log
```

### PR Comments

The workflow automatically posts benchmark results to PRs:

```markdown
## Benchmark Results

| Filename | Customers | Beta | Instance | Objective Value | Runtime | MILP Objective | Gap |
|----------|-----------|------|----------|-----------------|---------|----------------|-----|
| U_10_0.5_Num_1.txt | 10 | 0.5 | 1 | 132.00 | 0.41s | 131.0 | 0.76% |
| ... | ... | ... | ... | ... | ... | ... | ... |

📊 Full results available in artifacts
```

## 🎯 Performance Targets

Based on author's results, target metrics:

### 10-customer instances
- **MILP Gap**: 0-5% (excellent)
- **Runtime**: <1 second each
- **Success Rate**: 100%

### 15-customer instances  
- **MILP Gap**: 0-10% (good)
- **Runtime**: <1 second each
- **Partial MILP**: Many instances have MILP bounds

### 20-50 customer instances
- **Matheuristic Gap**: 0-3% at 50s timeout
- **Runtime**: Varies by size
- **Scale-dependent**: Larger instances need more time

### 75-100 customer instances
- **Matheuristic available**: References only
- **Quality metric**: Relative to best matheuristic
- **No MILP optimal**: Too large for exact solver

## 📝 Interpreting Results

### Gap Analysis

**What does "gap %" mean?**
- Formula: `(Your_Value - Optimal) / Optimal × 100`
- 0.00% = Optimal solution found ✓
- 1-3% = Near-optimal (excellent)
- 3-5% = Good solution (acceptable)
- 5%+ = Room for improvement (investigate)

### Runtime Analysis

**Small instances (10-15 nodes):**
- Should complete in <1 second
- Gives fast feedback on algorithm changes

**Medium instances (20-50 nodes):**
- Expect 1-60 seconds depending on complexity
- Good for parameter tuning

**Large instances (75-100 nodes):**
- May need minutes
- Measure improvement over multiple runs

## 🔍 Troubleshooting

### Benchmark script fails to run

**Check 1: Executable exists**
```bash
ls -la main_ga_tabu
# Should be executable file, not missing
```

**Check 2: Python and pandas installed**
```bash
python3 --version    # Should be 3.7+
python3 -c "import pandas; print(pandas.__version__)"  # Should work
# If missing: pip3 install pandas
```

**Check 3: Output format**
```bash
./main_ga_tabu "Instances 2/U_10_0.5_Num_1.txt"
# Should contain "Final cost:" and "[TOTAL RUNTIME]"
```

### GitHub Actions job fails

**Common issues:**
1. **Build failure**: Check `make clean && make` works locally
2. **Missing files**: Ensure all source files are committed
3. **Timeout**: Increase workflow timeout or reduce sample size
4. **Memory**: Increase runner if out of memory (unlikely on macOS)

**View detailed logs:**
- GitHub UI: Actions → [Job] → View logs
- GitHub CLI: `gh run view [ID] --log`

## 🎓 Integration Tips

### Before Committing Changes

1. **Run local quick test:**
   ```bash
   python3 scripts/run_benchmarks.py --max-customers 15 --sample-size 10
   ```

2. **Check for regressions:**
   ```bash
   # Compare gap trends
   tail -20 results/comparison_*.csv | grep "Gap to MILP"
   ```

3. **Verify no compilation issues:**
   ```bash
   make clean && make
   ./main_ga_tabu "Instances 2/U_10_0.5_Num_1.txt"
   ```

### After Merging to Main

1. **Monitor GitHub Actions**
   - Wait for full benchmark to complete (~1-2 hours)
   - Check if gaps increased (regression?)
   - Review performance trends

2. **Analyze trends**
   - Compare with previous runs
   - Identify bottlenecks by problem size
   - Test parameter sensitivity

3. **Document results**
   - Add results to git (optional): `git add results/*.csv`
   - Tag release with performance metrics
   - Update README with results

## 📚 Reference Files

The system uses reference data from the paper:

- **MILP Optimal Results**: 
  - `reference/VRPRD-DR/Results/Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv`
  - 10-20 customer instances with proven optimal values

- **Matheuristic Results**:
  - `reference/.../Matheuristic Results - 10 customers.csv`
  - `reference/.../Matheuristic Results - 15 and 20 customers.csv`
  - `reference/.../Matheuristic Results - Larger Instances.csv`

These are automatically loaded by the benchmark script for comparison.

## 🚀 Next Steps

1. **Test locally**: `bash scripts/quick_benchmark.sh`
2. **Commit workflow**: `git add .github/workflows/benchmark.yml && git commit`
3. **Push to GitHub**: `git push origin main`
4. **Monitor results**: GitHub Actions automatically runs and posts results
5. **Analyze trends**: Review artifacts and optimize parameters

## 🎉 Summary

You now have:

✅ **Automated benchmarking** - Single command to test all instances  
✅ **GitHub integration** - CI/CD pipeline for continuous testing  
✅ **Comparison capability** - Measure against industry-standard MILP  
✅ **Statistical analysis** - Grouping by size, location, parameters  
✅ **Historical tracking** - Timestamped results for trend analysis  
✅ **Complete documentation** - Scripts, workflows, and guides  

**Ready to benchmark!** 🚀

---

For detailed options and advanced usage, see [BENCHMARKING.md](docs/BENCHMARKING.md)

**Questions?** Check troubleshooting section above or review the Python script documentation.
