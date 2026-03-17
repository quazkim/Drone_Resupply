#!/usr/bin/env python3
"""
Comprehensive Benchmarking Suite for GA+Tabu vs Authors' MILP/Matheuristic
Runs all instances and generates comparison CSV
"""

import os
import sys
import subprocess
import csv
import re
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import pandas as pd

@dataclass
class Result:
    """Store result from a single run"""
    filename: str
    customers: int
    beta: float
    instance_num: int
    depot_location: str
    objective_value: Optional[float] = None
    runtime: Optional[float] = None
    feasible: bool = False
    error: Optional[str] = None
    
    def to_dict(self):
        return {
            'Filename': self.filename,
            'Customers': self.customers,
            'Beta': self.beta,
            'Instance': self.instance_num,
            'Depot Location': self.depot_location,
            'Objective Value': self.objective_value,
            'Runtime (s)': self.runtime,
            'Feasible': self.feasible,
            'Error': self.error
        }

class BenchmarkRunner:
    def __init__(self, workspace_dir: str, executable: str = "main_ga_tabu"):
        self.workspace_dir = Path(workspace_dir)
        self.executable_path = self.workspace_dir / executable
        self.instances_dir = self.workspace_dir / "Instances 2"
        self.results = []
        
        # Mapping from beta to approximate depot location based on author's data
        # This is a heuristic - we'll also try to infer from coordinates
        self.depot_location_map = {}
        
    def parse_filename(self, filename: str) -> Tuple[int, float, int]:
        """Parse U_10_0.5_Num_1.txt -> (10, 0.5, 1)"""
        match = re.match(r'U_(\d+)_([0-9.]+)_Num_(\d+)\.txt', filename)
        if match:
            return int(match.group(1)), float(match.group(2)), int(match.group(3))
        return None
    
    def infer_depot_location(self, instance_file: str) -> str:
        """
        Infer depot location from file coordinates
        Center: depot ~ center of all customers
        Border: depot ~ edge/near border
        Outside: depot ~ clearly outside the region
        """
        try:
            with open(instance_file, 'r') as f:
                lines = f.readlines()[1:]  # Skip header
                coordinates = []
                for line in lines[:11]:  # Read first 11 nodes (depot + up to 10 customers)
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        x, y = float(parts[1]), float(parts[2])
                        coordinates.append((x, y))
                
                if not coordinates:
                    return "Unknown"
                
                # Depot is first node (index 1 - after reading skip header)
                depot = coordinates[0]
                customers = coordinates[1:]
                
                if not customers:
                    return "Unknown"
                
                # Calculate centroid of customers
                center_x = sum(c[0] for c in customers) / len(customers)
                center_y = sum(c[1] for c in customers) / len(customers)
                
                # Calculate bounds
                min_x = min(c[0] for c in customers)
                max_x = max(c[0] for c in customers)
                min_y = min(c[1] for c in customers)
                max_y = max(c[1] for c in customers)
                
                # Depot distance from centroid (normalized)
                dist_from_center = ((depot[0] - center_x)**2 + (depot[1] - center_y)**2)**0.5
                region_size = ((max_x - min_x)**2 + (max_y - min_y)**2)**0.5 / 2
                
                # If depot is outside the bounding box of customers
                if (depot[0] < min_x or depot[0] > max_x or 
                    depot[1] < min_y or depot[1] > max_y):
                    return "Outside"
                
                # If depot is close to center
                if dist_from_center < region_size * 0.3:
                    return "Center"
                
                # Otherwise it's at border
                return "Border"
                
        except Exception as e:
            print(f"Error inferring depot location: {e}")
            return "Unknown"
    
    def parse_output(self, output: str) -> Tuple[Optional[float], Optional[float]]:
        """Extract objective value and runtime from program output"""
        objective = None
        runtime = None
        
        # Extract objective value (Final cost)
        obj_match = re.search(r'Final cost:\s*([\d.]+)\s*min', output)
        if obj_match:
            objective = float(obj_match.group(1))
        
        # Extract runtime
        runtime_match = re.search(r'\[TOTAL RUNTIME\]\s*([\d.]+)\s*seconds', output)
        if runtime_match:
            runtime = float(runtime_match.group(1))
        
        return objective, runtime
    
    def run_instance(self, instance_file: str) -> Result:
        """Run a single instance and return result"""
        full_path = self.instances_dir / instance_file
        
        result = Result(
            filename=instance_file,
            customers=0,
            beta=0.0,
            instance_num=0,
            depot_location="Unknown"
        )
        
        # Parse filename
        parsed = self.parse_filename(instance_file)
        if parsed:
            result.customers, result.beta, result.instance_num = parsed
        else:
            result.error = "Failed to parse filename"
            return result
        
        # Infer depot location
        result.depot_location = self.infer_depot_location(str(full_path))
        
        # Run executable
        try:
            start_time = time.time()
            cmd = [str(self.executable_path), str(full_path)]
            output = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=3600  # 1 hour timeout
            )
            elapsed = time.time() - start_time
            
            # Parse output
            objective, runtime = self.parse_output(output.stdout + output.stderr)
            
            result.objective_value = objective
            result.runtime = runtime if runtime else elapsed
            result.feasible = objective is not None
            
            if not result.feasible:
                result.error = f"Failed to extract objective. Return code: {output.returncode}"
                
        except subprocess.TimeoutExpired:
            result.error = "Timeout (>1 hour)"
        except FileNotFoundError:
            result.error = f"Executable not found: {self.executable_path}"
        except Exception as e:
            result.error = str(e)
        
        return result
    
    def run_all(self, max_customers: Optional[int] = None, 
                sample_size: Optional[int] = None) -> List[Result]:
        """Run all instances matching criteria"""
        instances = sorted(self.instances_dir.glob("U_*.txt"))
        
        # Filter by customer count if specified
        if max_customers:
            instances = [f for f in instances 
                        if self.parse_filename(f.name) and 
                        self.parse_filename(f.name)[0] <= max_customers]
        
        # Sample if specified
        if sample_size and len(instances) > sample_size:
            step = len(instances) // sample_size
            instances = instances[::step]
        
        print(f"Running {len(instances)} instances...")
        
        for idx, instance_file in enumerate(instances, 1):
            print(f"[{idx}/{len(instances)}] {instance_file.name}...", end=' ', flush=True)
            
            result = self.run_instance(instance_file.name)
            self.results.append(result)
            
            if result.feasible:
                print(f"✓ {result.objective_value:.2f} min ({result.runtime:.2f}s)")
            else:
                print(f"✗ {result.error}")
        
        return self.results
    
    def save_csv(self, output_file: str):
        """Save results to CSV"""
        with open(output_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.results[0].to_dict().keys())
            writer.writeheader()
            for result in self.results:
                writer.writerow(result.to_dict())
        
        print(f"\nResults saved to {output_file}")
    
    def generate_comparison(self, output_file: str):
        """Generate comparison with author's MILP/Matheuristic results"""
        # Load author's results
        milp_df = self._load_milp_results()
        
        # Convert our results to DataFrame
        our_df = pd.DataFrame([r.to_dict() for r in self.results])
        
        # Merge and compute gaps
        comparison = []
        
        for _, our_row in our_df.iterrows():
            # Find matching MILP result
            milp_row = milp_df[
                (milp_df['Customers'] == our_row['Customers']) &
                (milp_df['Instance'] == our_row['Instance']) &
                (milp_df['Beta'] == our_row['Beta'])
            ]
            
            comp_row = our_row.to_dict()
            
            if not milp_row.empty:
                milp_obj = milp_row.iloc[0]['Objective value']
                comp_row['MILP Objective'] = milp_obj
                
                if our_row['Objective Value']:
                    gap = ((our_row['Objective Value'] - milp_obj) / milp_obj) * 100
                    comp_row['Gap to MILP (%)'] = round(gap, 2)
                else:
                    comp_row['Gap to MILP (%)'] = None
            else:
                comp_row['MILP Objective'] = None
                comp_row['Gap to MILP (%)'] = None
            
            comparison.append(comp_row)
        
        # Save comparison
        comp_df = pd.DataFrame(comparison)
        comp_df.to_csv(output_file, index=False)
        print(f"Comparison saved to {output_file}")
        
        # Print statistics
        print("\n=== BENCHMARK STATISTICS ===")
        print(f"Total instances: {len(our_df)}")
        print(f"Feasible: {our_df['Feasible'].sum()}")
        print(f"Avg runtime: {our_df['Runtime (s)'].mean():.2f}s")
        
        comp_df_clean = comp_df.dropna(subset=['Gap to MILP (%)'])
        if len(comp_df_clean) > 0:
            print(f"Avg gap to MILP: {comp_df_clean['Gap to MILP (%)'].mean():.2f}%")
            print(f"Max gap to MILP: {comp_df_clean['Gap to MILP (%)'].max():.2f}%")
    
    def _load_milp_results(self) -> pd.DataFrame:
        """Load MILP results from reference CSV"""
        milp_file = self.workspace_dir / "reference/VRPRD-DR/Results/Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv"
        
        if milp_file.exists():
            df = pd.read_csv(milp_file)
            # Rename columns to match our output
            df = df.rename(columns={
                'Customers': 'Customers',
                'Instance': 'Instance',
                'Beta': 'Beta',
                'Objective value': 'Objective value'
            })
            return df
        
        return pd.DataFrame()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Benchmark GA+Tabu solver')
    parser.add_argument('--max-customers', type=int, default=20, 
                       help='Max customer count to test (default: 20)')
    parser.add_argument('--sample-size', type=int, default=None,
                       help='Sample N instances evenly (default: all)')
    parser.add_argument('--output-dir', type=str, default='results',
                       help='Output directory for CSV files')
    
    args = parser.parse_args()
    
    workspace_dir = Path(__file__).parent.parent
    runner = BenchmarkRunner(str(workspace_dir))
    
    # Run benchmarks
    print(f"Starting benchmark runs (max_customers={args.max_customers}, sample_size={args.sample_size})...")
    runner.run_all(max_customers=args.max_customers, sample_size=args.sample_size)
    
    # Create output directory
    output_dir = workspace_dir / args.output_dir
    output_dir.mkdir(exist_ok=True)
    
    # Save results
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_file = output_dir / f"our_results_{timestamp}.csv"
    runner.save_csv(str(csv_file))
    
    # Generate comparison
    comp_file = output_dir / f"comparison_{timestamp}.csv"
    runner.generate_comparison(str(comp_file))
    
    print(f"\n✓ Results saved to {output_dir}/")

if __name__ == "__main__":
    main()
