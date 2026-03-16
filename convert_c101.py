#!/usr/bin/env python3
"""Convert C101_1.dat format to our PDP format"""

import sys

input_file = "reference/C101_1.dat"
output_file = "Instances 2/C101_1.txt"

lines = open(input_file).readlines()

# Parse parameters
params = {}
data_start = 0
for i, line in enumerate(lines):
    if '\t' in line and not line.startswith('XCOORD'):
        parts = line.strip().split('\t')
        if len(parts) == 2:
            try:
                params[parts[0]] = float(parts[1])
            except:
                params[parts[0]] = parts[1]
    if 'XCOORD' in line:
        data_start = i + 1
        break

print(f"Parameters: {params}")
print(f"Data starts at line {data_start}")

# Parse coordinates
with open(output_file, 'w') as out:
    customer_id = 0
    for i in range(data_start, len(lines)):
        parts = lines[i].strip().split()
        if len(parts) < 4:
            continue
        
        x = float(parts[0])
        y = float(parts[1])
        demand = int(float(parts[2]))
        ready_time = int(float(parts[3])) if len(parts) > 3 else 0
        
        # ID X Y TYPE READY_TIME PAIR_ID
        node_type = "D"  # All as delivery
        pair_id = 0
        
        out.write(f"{customer_id}\t{x}\t{y}\t{node_type}\t{ready_time}\t{pair_id}\n")
        customer_id += 1

print(f"✓ Converted to: {output_file}")
print(f"Total nodes: {customer_id}")
