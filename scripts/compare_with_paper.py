#!/usr/bin/env python3
"""
Run specified instances and compare results with MILP paper values
"""

import subprocess
import os
import re
from pathlib import Path

# Paper MILP results
PAPER_RESULTS = {
    (10, 0.5, 1): 131,
    (10, 0.5, 2): 160,
    (10, 0.5, 3): 134,
    (10, 0.5, 4): 150,
    (10, 0.5, 5): 124,
    (10, 1.0, 1): 173,
    (10, 1.0, 2): 214,
    (10, 1.0, 3): 193,
    (10, 1.0, 4): 234,
    (10, 1.0, 5): 175,
    (10, 1.5, 1): 246,
    (10, 1.5, 2): 199,
    (10, 1.5, 3): 248,
    (10, 1.5, 4): 350,
    (10, 1.5, 5): 271,
}

def run_instance(instance_file, num_customers, beta, instance_num):
    """Run a single instance and extract C_max"""
    try:
        cmd = f"./main_ga_tabu \"{instance_file}\" 50 100 0.1 1"
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=300,
            cwd="/Users/kimquan/LabDrone/Drone_Resupply"
        )
        
        output = result.stdout
        
        # Extract C_max from "Cost after Local Search: XXX.XX minutes"
        match = re.search(r"Cost after Local Search:\s+([\d.]+)", output)
        if match:
            cmax = float(match.group(1))
            return cmax
        
        # Also try "SUMMARY" section
        match = re.search(r"Cost before Local Search:\s+([\d.]+)", output)
        if match:
            cmax = float(match.group(1))
            return cmax
            
        print(f"  ✗ Could not extract C_max from output")
        return None
        
    except subprocess.TimeoutExpired:
        print(f"  ✗ Timeout (>300s)")
        return None
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return None

def main():
    base_path = "/Users/kimquan/LabDrone/Drone_Resupply/Instances 2"
    
    # Test instances to run
    test_cases = [
        (10, 0.5, 1), (10, 0.5, 2), (10, 0.5, 3), (10, 0.5, 4), (10, 0.5, 5),
        (10, 1.0, 1), (10, 1.0, 2), (10, 1.0, 3), (10, 1.0, 4), (10, 1.0, 5),
        (10, 1.5, 1), (10, 1.5, 2), (10, 1.5, 3), (10, 1.5, 4), (10, 1.5, 5),
    ]
    
    print("\n" + "="*100)
    print("RUNNING GA+TABU INSTANCES AND COMPARING WITH MILP PAPER RESULTS")
    print("="*100 + "\n")
    
    results = []
    
    for num_customers, beta, instance_num in test_cases:
        # Convert beta to correct format (0.5 -> 0.5, 1.0 -> 1.0, etc)
        beta_str = f"{beta:.1f}"
        instance_file = f"{base_path}/U_{num_customers}_{beta_str}_Num_{instance_num}.txt"
        
        # Check if file exists
        if not os.path.exists(instance_file):
            print(f"✗ Instance file not found: {instance_file}")
            continue
        
        print(f"Running U_{num_customers}_{beta}_Num_{instance_num}...", end=" ", flush=True)
        
        cmax = run_instance(instance_file, num_customers, beta, instance_num)
        
        if cmax is not None:
            paper_value = PAPER_RESULTS[(num_customers, beta, instance_num)]
            diff_percent = ((cmax - paper_value) / paper_value) * 100
            
            status = "✓" if diff_percent <= 0 else "!"
            print(f"{status} GA+Tabu: {cmax:7.2f} | MILP: {paper_value:3d} | Diff: {diff_percent:+6.2f}%")
            
            results.append({
                'customers': num_customers,
                'beta': beta,
                'instance': instance_num,
                'ga_tabu': cmax,
                'milp': paper_value,
                'diff_percent': diff_percent
            })
        else:
            print("FAILED")
    
    # Print summary table
    print("\n" + "="*100)
    print("SUMMARY TABLE")
    print("="*100)
    print(f"{'Customers':<12} {'Beta':<8} {'Instance':<10} {'GA+Tabu':<12} {'MILP':<8} {'Diff %':<12} {'Status':<8}")
    print("-"*100)
    
    better_count = 0
    equal_count = 0
    worse_count = 0
    
    for r in results:
        status = "BETTER" if r['diff_percent'] < -0.5 else ("EQUAL" if abs(r['diff_percent']) < 0.5 else "WORSE")
        if r['diff_percent'] < -0.5:
            better_count += 1
        elif abs(r['diff_percent']) < 0.5:
            equal_count += 1
        else:
            worse_count += 1
        
        print(f"{r['customers']:<12} {r['beta']:<8.1f} {r['instance']:<10} {r['ga_tabu']:<12.2f} {r['milp']:<8} {r['diff_percent']:<+11.2f}% {status:<8}")
    
    print("-"*100)
    print(f"\nStatistics:")
    print(f"  Better than MILP: {better_count} instances")
    print(f"  Equal (±0.5%):   {equal_count} instances")
    print(f"  Worse than MILP:  {worse_count} instances")
    print(f"  Total:            {len(results)} instances")
    
    if results:
        avg_diff = sum(r['diff_percent'] for r in results) / len(results)
        print(f"\nAverage difference: {avg_diff:+.2f}%")
    
    print("="*100 + "\n")

if __name__ == "__main__":
    main()
