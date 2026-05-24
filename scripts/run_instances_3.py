#!/usr/bin/env python3
import subprocess
import re
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

def run_instance(instance_path):
    """Run single instance and return (name, c_max, cpu_time)"""
    try:
        result = subprocess.run(
            ['./main_ga_tabu', str(instance_path), '50', '100', '0.1', '1'],
            capture_output=True, text=True, timeout=120
        )
        
        # Extract C_max and times from output
        match_cmax = re.search(r'Cost after Local Search:\s*([\d.]+)', result.stdout)
        match_ga_time = re.search(r'GA time:\s*([\d.]+)', result.stdout)
        match_ls_time = re.search(r'Local search time:\s*([\d.]+)', result.stdout)
        
        if match_cmax:
            c_max = match_cmax.group(1)
            ga_time = match_ga_time.group(1) if match_ga_time else '0'
            ls_time = match_ls_time.group(1) if match_ls_time else '0'
            total_time = str(round(float(ga_time) + float(ls_time), 2))
            
            instance_name = instance_path.name.replace('_all_d.txt', '')
            return (instance_name, c_max, total_time)
    
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT: {instance_path.name}")
    except Exception as e:
        print(f"  ERROR {instance_path.name}: {e}")
    
    return None

# Get all instance files from Instances 3 folder
instances_dir = Path("Instances 3")
instance_files = sorted(instances_dir.glob("U_*_all_d.txt"))

print(f"Running {len(instance_files)} instances from Instances 3 with 10 workers...\n")

# Run instances in parallel
results = {}
completed = 0

with ThreadPoolExecutor(max_workers=10) as executor:
    futures = {executor.submit(run_instance, inst): inst for inst in instance_files}
    
    for future in as_completed(futures):
        inst = futures[future]
        result = future.result()
        completed += 1
        
        if result:
            name, c_max, cpu_time = result
            results[name] = (c_max, cpu_time)
            print(f"[{completed:2d}/{len(instance_files)}] {inst.name:35s} ✓ C_max={c_max:7s} CPU={cpu_time}s")
        else:
            print(f"[{completed:2d}/{len(instance_files)}] {inst.name:35s} ✗")

print(f"\n✓ Completed {len(results)}/{len(instance_files)} instances\n")

# Save results to file
output_file = "results/Instances_3_Results.txt"
with open(output_file, 'w') as f:
    f.write("Instance\tC_max\tCPU_time(s)\n")
    for name in sorted(results.keys()):
        c_max, cpu_time = results[name]
        f.write(f"{name}\t{c_max}\t{cpu_time}\n")

print(f"✓ Results saved to {output_file}")
print(f"\nSummary by size:")
for size in ['10', '15', '20']:
    size_results = [v[0] for k, v in results.items() if f'U_{size}_' in k]
    if size_results:
        avg = sum(float(x) for x in size_results) / len(size_results)
        print(f"  U_{size}: {len(size_results)} instances, avg C_max = {avg:.2f}")
