import subprocess
import re

results = {}
instances_to_run = []

# U_10 - 0.5, 1.0, 1.5 - 1 to 5
for beta in ['0.5', '1.0', '1.5']:
    for num in range(1, 6):
        instances_to_run.append(f'U_10_{beta}_Num_{num}_pd.txt')

# U_15 - 0.5, 1.0, 1.5 - 1 to 5  
for beta in ['0.5', '1.0', '1.5']:
    for num in range(1, 6):
        instances_to_run.append(f'U_15_{beta}_Num_{num}_pd.txt')

# U_20 - 0.5, 1.0, 1.5 - 1 to 5
for beta in ['0.5', '1.0', '1.5']:
    for num in range(1, 6):
        instances_to_run.append(f'U_20_{beta}_Num_{num}_pd.txt')

total = len(instances_to_run)
for idx, instance_file in enumerate(instances_to_run, 1):
    print(f"[{idx}/{total}] {instance_file}...", end=" ", flush=True)
    
    try:
        result = subprocess.run(['./main_ga_tabu', instance_file, '50', '100', '0.1', '1'], 
                              capture_output=True, text=True, timeout=120)
        
        output = result.stdout + result.stderr
        
        # Look for "Cost after Local Search: X minutes"
        match = re.search(r'Cost after Local Search:\s*([\d.]+)\s*minutes', output)
        
        if match:
            c_max = match.group(1)
            
            # Parse filename directly
            import re as re_mod
            m = re_mod.match(r'U_(\d+)_(\d+\.\d+)_Num_(\d+)_pd\.txt', instance_file)
            if m:
                customers = m.group(1)
                beta = m.group(2)
                num = m.group(3)
                
                key = f"{customers}_{beta}_{num}"
                results[key] = c_max
                print(f"✓ {c_max}")
        else:
            print(f"✗ Parse failed")
            
    except subprocess.TimeoutExpired:
        print(f"✗ Timeout")
    except Exception as e:
        print(f"✗ {str(e)[:30]}")

print(f"\n\nCompleted {len(results)} instances\n")

# Print results in format needed for CSV
for key in sorted(results.keys()):
    customers, beta, num = key.split('_')
    c_max = results[key]
    print(f"{customers};Center;{num};{beta};{c_max}")

