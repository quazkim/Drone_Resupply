#!/usr/bin/env python3
"""
Benchmark script: Run GA+Tabu instances and compare with MILP optimal
Generates CSV report with gap analysis
"""

import os
import subprocess
import csv
import sys
from pathlib import Path
import time

# Configuration
WORK_DIR = Path("/Users/kimquan/LabDrone/Drone_Resupply")
INSTANCE_DIR = WORK_DIR / "Instances 2"
MILP_RESULTS = WORK_DIR / "reference/VRPRD-DR/Results/Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv"
OUTPUT_CSV = WORK_DIR / "results/benchmark_ga_vs_milp.csv"
SOLVER = WORK_DIR / "main_ga_tabu"

os.chdir(WORK_DIR)
OUTPUT_CSV.parent.mkdir(exist_ok=True)

# Load MILP results
milp_db = {}
if MILP_RESULTS.exists():
    with open(MILP_RESULTS, encoding='utf-8', errors='ignore') as f:
        content = f.read()
        # Handle potential line break issues
        lines = content.split('\n')
        if lines:
            # Parse header
            header = [h.strip() for h in lines[0].split(',')]
            for line in lines[1:]:
                if not line.strip():
                    continue
                parts = [p.strip() for p in line.split(',')]
                if len(parts) < 9:
                    continue
                try:
                    customers = int(parts[0])
                    depot = parts[1]
                    instance = int(parts[2])
                    beta = float(parts[3])
                    objective = float(parts[8])
                    
                    key = (customers, depot, instance, beta)
                    milp_db[key] = objective
                except (ValueError, IndexError):
                    continue
    print(f"✓ Loaded {len(milp_db)} MILP optimal values")
else:
    print(f"⚠ MILP results not found at {MILP_RESULTS}")

# Test instances
test_configs = [
    # (size, betas, depot_names, max_instances)
    (10, [0.5, 1.0, 1.5], ["Center", "Border", "Outside"], 5),
    (15, [0.5, 1.0, 1.5], ["Center", "Border", "Outside"], 3),  # Only 3 instances
    (20, [0.5, 1.0, 1.5], ["Center", "Border", "Outside"], 2),  # Only 2 instances
]

# Depot mode mapping
depot_map = {"Center": 0, "Border": 1, "Outside": 2}

# Results tracking
results = []

print("\n" + "="*80)
print("BENCHMARK: GA+Tabu vs MILP Optimal")
print("="*80 + "\n")

for size, betas, depots, max_inst in test_configs:
    print(f">>> Testing {size} customers <<<\n")
    sys.stdout.flush()
    
    for beta in betas:
        for depot_name in depots:
            depot_mode = depot_map[depot_name]
            
            for inst_num in range(1, max_inst + 1):
                instance_file = INSTANCE_DIR / f"U_{size}_{beta}_Num_{inst_num}.txt"
                
                if not instance_file.exists():
                    continue
                
                # Run GA+Tabu
                try:
                    progress = f"  ↻ U_{size}_{beta}_Num_{inst_num} ({depot_name:8s}): Running..."
                    print(progress, end='', flush=True)
                    sys.stdout.flush()
                    
                    start = time.time()
                    result = subprocess.run(
                        [str(SOLVER), str(instance_file), "--depot", str(depot_mode)],
                        capture_output=True,
                        text=True,
                        timeout=600
                    )
                    elapsed = time.time() - start
                    
                    # Extract final cost
                    ga_cost = None
                    for line in result.stdout.split('\n'):
                        if line.startswith('Final cost:'):
                            try:
                                ga_cost = float(line.split(':')[1].split('min')[0].strip())
                            except:
                                pass
                    
                    if ga_cost is None:
                        print(f"\r  ❌ U_{size}_{beta}_Num_{inst_num} ({depot_name}): Failed to extract cost                    ")
                        sys.stdout.flush()
                        continue
                    
                    # Look up MILP optimal
                    milp_key = (size, depot_name, inst_num, beta)
                    milp_opt = milp_db.get(milp_key)
                    
                    if milp_opt:
                        gap = ((ga_cost - milp_opt) / milp_opt) * 100
                        status = "✓" if gap <= 5 else "⚠" if gap <= 10 else "❌"
                        msg = (f"\r  {status} U_{size}_{beta}_Num_{inst_num} ({depot_name:8s}): " +
                              f"GA={ga_cost:7.2f}, MILP={milp_opt:7.2f}, Gap={gap:6.2f}%, Time={elapsed:6.1f}s         ")
                        print(msg)
                    else:
                        msg = (f"\r  ℹ U_{size}_{beta}_Num_{inst_num} ({depot_name:8s}): " +
                              f"GA={ga_cost:7.2f}, MILP=N/A, Time={elapsed:6.1f}s                 ")
                        print(msg)
                    
                    sys.stdout.flush()
                    
                    results.append({
                        'Customers': size,
                        'Depot': depot_name,
                        'Instance': inst_num,
                        'Beta': beta,
                        'GA_Cost': ga_cost,
                        'MILP_Optimal': milp_opt if milp_opt else 'N/A',
                        'Gap_%': gap if milp_opt else 'N/A',
                        'Runtime_s': elapsed
                    })
                    
                except subprocess.TimeoutExpired:
                    print(f"\r  ⏱ U_{size}_{beta}_Num_{inst_num} ({depot_name}): Timeout (>600s)                    ")
                    sys.stdout.flush()
                except Exception as e:
                    print(f"\r  ❌ U_{size}_{beta}_Num_{inst_num} ({depot_name}): {e}                    ")
                    sys.stdout.flush()
            
            print()
            sys.stdout.flush()

# Write CSV
with open(OUTPUT_CSV, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['Customers', 'Depot', 'Instance', 'Beta', 'GA_Cost', 'MILP_Optimal', 'Gap_%', 'Runtime_s'])
    writer.writeheader()
    writer.writerows(results)

print("\n" + "="*80)
if results:
    avg_gap = sum(r['Gap_%'] for r in results if isinstance(r['Gap_%'], float)) / len([r for r in results if isinstance(r['Gap_%'], float)])
    print(f"✓ Benchmark complete!")
    print(f"  Total runs: {len(results)}")
    print(f"  Average gap: {avg_gap:.2f}%")
    print(f"  Results saved to: {OUTPUT_CSV}")
else:
    print("❌ No results generated")
print("="*80 + "\n")
