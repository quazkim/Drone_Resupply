import subprocess
import re
import concurrent.futures
import time

def run_instance(instance_file):
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
                return (f"{customers};Center;{num};{beta}", c_max, instance_file)
    except:
        pass
    return None

# Run U_10 Center, U_15 Center, U_20 Center (15 instances = 45 total, but run 10 parallel)
instances = []
for beta in ['0.5', '1.0', '1.5']:
    for num in range(1, 6):
        instances.append(f'U_15_{beta}_Num_{num}_pd.txt')
        instances.append(f'U_20_{beta}_Num_{num}_pd.txt')

print(f"Running {len(instances)} instances with 10 parallel workers...\n")
results = {}
completed = 0

with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
    futures = {executor.submit(run_instance, inst): inst for inst in instances}
    
    for future in concurrent.futures.as_completed(futures):
        inst = futures[future]
        try:
            result = future.result()
            if result:
                key, c_max, _ = result
                results[key] = c_max
                completed += 1
                print(f"[{completed}/{len(instances)}] {inst} ✓ {c_max}")
            else:
                print(f"[{completed+1}/{len(instances)}] {inst} ✗")
        except Exception as e:
            print(f"[{completed+1}/{len(instances)}] {inst} ✗ {str(e)[:30]}")

print(f"\n✓ Completed {completed} instances\n")
print("Results:")
for key in sorted(results.keys()):
    print(f"{key};{results[key]}")

