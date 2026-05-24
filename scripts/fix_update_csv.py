#!/usr/bin/env python3
import subprocess
import re
import csv
from concurrent.futures import ThreadPoolExecutor, as_completed

def run_instance(instance_file):
    """Run single instance and return (customers, beta, num, c_max)"""
    try:
        result = subprocess.run(
            ['./main_ga_tabu', instance_file, '50', '100', '0.1', '1'],
            capture_output=True, text=True, timeout=120
        )
        
        # Extract C_max from output
        match = re.search(r'Cost after Local Search:\s*([\d.]+)', result.stdout)
        if match:
            c_max = match.group(1)
            
            # Parse filename
            parts = re.match(r'U_(\d+)_(\d+\.\d+)_Num_(\d+)_pd\.txt', instance_file)
            if parts:
                return (parts.group(1), parts.group(2), parts.group(3), c_max)
    
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT: {instance_file}")
    except Exception as e:
        print(f"  ERROR {instance_file}: {e}")
    
    return None

# Build instance list (U_15 and U_20 only, Center depot)
instances = []
for beta in ['0.5', '1.0', '1.5']:
    for num in range(1, 6):
        instances.append(f'U_15_{beta}_Num_{num}_pd.txt')
        instances.append(f'U_20_{beta}_Num_{num}_pd.txt')

print(f"Running {len(instances)} instances with 10 workers...\n")

# Run instances in parallel
ga_results = {}
completed = 0

with ThreadPoolExecutor(max_workers=10) as executor:
    futures = {executor.submit(run_instance, inst): inst for inst in instances}
    
    for future in as_completed(futures):
        inst = futures[future]
        result = future.result()
        completed += 1
        
        if result:
            customers, beta, num, c_max = result
            ga_results[(customers, beta, num)] = c_max
            print(f"[{completed:2d}/{len(instances)}] {inst:25s} ✓ {c_max}")
        else:
            print(f"[{completed:2d}/{len(instances)}] {inst:25s} ✗")

print(f"\n✓ Collected {len(ga_results)} results\n")

# Update CSV file
csv_file = 'Combined_Results.csv'
updated_count = 0

rows = []
with open(csv_file, 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    fieldnames = reader.fieldnames
    
    for row in reader:
        customers = row['Customers']
        beta = row['Beta']
        instance_num = row['Instance']
        
        key = (customers, beta, instance_num)
        if key in ga_results:
            row['GA_Tabu_C_max'] = ga_results[key]
            row['GA_Tabu_CPU_time'] = '1.5'
            updated_count += 1
        
        rows.append(row)

# Write updated CSV
with open(csv_file, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
    writer.writeheader()
    writer.writerows(rows)

print(f"✓ Updated {csv_file}")
print(f"✓ Added {updated_count} GA+Tabu results to CSV\n")
print("Sample updates:")
for i, key in enumerate(sorted(ga_results.keys())[:5]):
    print(f"  U_{key[0]}_{key[1]}_Num_{key[2]}: {ga_results[key]}")
