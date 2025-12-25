#!/usr/bin/env python3
"""
Convert all Instance 2 files from old format (ri X Y) to new format (id X Y type ready_time pair_id)
All customers are type D with pair_id = 0
"""

import os
import sys

def convert_file(input_path, output_path):
    """Convert a single file from old format to new format"""
    try:
        with open(input_path, 'r') as f:
            lines = f.readlines()
        
        # Skip header line if it exists
        start_idx = 0
        if lines and lines[0].startswith('ri'):
            start_idx = 1
        
        # Convert data
        converted_lines = ['0\t0\t0\tD\t0\t0\n']  # Depot
        
        node_id = 1
        for line in lines[start_idx:]:
            line = line.strip()
            if not line:
                continue
            
            parts = line.split()
            if len(parts) >= 3:
                try:
                    ready_time = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    
                    # Format: id  X  Y  type  ready_time  pair_id
                    converted_line = f"{node_id}\t{x:.2f}\t{y:.2f}\tD\t{int(ready_time)}\t0\n"
                    converted_lines.append(converted_line)
                    node_id += 1
                except ValueError:
                    print(f"  Warning: Skipped invalid line: {line}")
                    continue
        
        # Write output
        with open(output_path, 'w') as f:
            f.writelines(converted_lines)
        
        num_customers = node_id - 1
        print(f"✓ {os.path.basename(input_path):40s} → {num_customers:3d} customers")
        return True
    
    except Exception as e:
        print(f"✗ {os.path.basename(input_path):40s} ERROR: {e}")
        return False

def main():
    folder = "/Users/kimquan/LabDrone/Drone_Resupply/Instances 2"
    
    if not os.path.isdir(folder):
        print(f"Error: Folder not found: {folder}")
        sys.exit(1)
    
    # Get all .txt files
    files = [f for f in os.listdir(folder) if f.endswith('.txt') and not f.startswith('.')]
    files.sort()
    
    # Filter to only instance files (exclude already converted and README)
    instance_files = [f for f in files if f.startswith('U_') and not f.endswith('_converted.txt')]
    
    print(f"Converting {len(instance_files)} files from Instances 2...")
    print("=" * 80)
    
    success_count = 0
    for filename in instance_files:
        input_path = os.path.join(folder, filename)
        output_path = input_path  # Overwrite original
        
        if convert_file(input_path, output_path):
            success_count += 1
    
    print("=" * 80)
    print(f"Conversion complete: {success_count}/{len(instance_files)} files converted")

if __name__ == "__main__":
    main()
