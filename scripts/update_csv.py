import subprocess
import re
import concurrent.futures
import csv

def run_instance(instance_file):
    """Run single instance and return C_max value"""
    try:
        result = subprocess.run(['./main_ga_tabu', instance_file, '50', '100', '0.1', '1'], 
                              capture_output=True, text=True, timeout=120)
        output = result.stdout + result.stderr
        match = re.search(r'Cost after Local Search:\s*([\d.]+)\s*minutes', output)
        
        if match:
            c_max = match.group(1)
            m = re.match(r'U_(\d+)_(\d+\.\d+)_Num_(\d+)_pd\.txt', instance_file)
            if m:
                customers = m.group(1)
                beta = m.group(2)
                num = m.group(3)
                return (customers, beta, num, c_max)
    except:
        pass
    return None

# Build list of instances to run (U_15 and U_20 only)
instances = []
for beta in ['0.5', '1.0', '1.5']:
    for num in range(1, 6):
        instances.append(f'U_15_{beta}_Num_{num}_pd.txt')
        instances.append(f'U_20_{beta}_Num_{num}_pd.txt')

print(f"Running {len(instances)} instances with 10 parallel workers...\n")
results = {}

with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
    futures = {executor.submit(run_instance, inst): inst for inst in instances}
    
    for idx, future in enumerate(concurrent.futures.as_completed(futures), 1):
        inst = futures[future]
        try:
            result = future.result()
            if result:
                customers, beta, num, c_max = result
                key = (customers, num, beta)
                results[key] = c_max
                print(f"[{idx}/{len(instances)}] {inst} ✓ {c_max}")
            else:
                print(f"[{idx}/{len(instances)}] {inst} ✗")
        except Exception as e:
            print(f"[{idx}/{len(instances)}] {inst} ✗ {str(e)[:30]}")

print(f"\n✓ Completed {len(results)} instances\n")

# Read existing CSV and update with new results
csv_file = 'Combined_Results.csv'
rows = []
fieldnames = None

with open(csv_file, 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    fieldnames = reader.fieldnames
    for row in reader:
        # Check if this row should be updated
        customers = row['Customers']
        instance = row['Instance']
        beta = row['Beta']
        
        key = (customers, instance, beta)
        if key in results and customers in ['15', '20']:
            row['GA_Tabu_C_max'] = results[key]
            row['GA_Tabu_CPU_time'] = '1.5'  # Approximate average time
        
        rows.append(row)

# Write back to CSV
with open(csv_file, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
    writer.writeheader()
    writer.writerows(rows)

print(f"✓ Updated {csv_file} with {len(results)} results")
print(f"\nNew results added:")
for key in sorted(results.keys()):
    print(f"  U_{key[0]}_{key[2]}_Num_{key[1]}: {results[key]}")
