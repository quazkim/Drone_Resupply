#!/usr/bin/env python3
"""
Merge MILP results with GA+Tabu results
"""

import csv
import pandas as pd

# Read original MILP results
milp_df = pd.read_csv('Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv', delimiter=';')

# Read GA+Tabu results (we'll add more as we run them)
ga_results = """Customers,Depot,Instance,Beta,C_max
10,Center,1,0.5,96.45
10,Center,2,0.5,109.96
10,Center,3,0.5,100.60
10,Center,4,0.5,141.55
10,Center,5,0.5,84.12
10,Center,1,1.0,137.74
10,Center,2,1.0,201.50
10,Center,3,1.0,144.82
10,Center,4,1.0,224.15
10,Center,5,1.0,176.14
10,Center,1,1.5,208.53
10,Center,2,1.5,140.55
10,Center,3,1.5,249.51
10,Center,4,1.5,322.71
10,Center,5,1.5,272.85"""

from io import StringIO
ga_df = pd.read_csv(StringIO(ga_results))

# Create output combining both
output_rows = []

# Process each row from MILP
for idx, milp_row in milp_df.iterrows():
    customers = milp_row['Customers']
    depot = milp_row['Depot location']
    instance = milp_row['Instance']
    beta = milp_row['Beta']
    
    # Find matching GA result
    ga_match = ga_df[
        (ga_df['Customers'] == customers) & 
        (ga_df['Depot'] == depot) & 
        (ga_df['Instance'] == instance) & 
        (ga_df['Beta'] == beta)
    ]
    
    row_dict = {
        'Customers': customers,
        'Depot location': depot,
        'Instance': instance,
        'Beta': beta,
        'MILP_CPU_time': milp_row['CPU time (s)'],
        'MILP_Objective': milp_row['Objective value'],
        'MILP_Gap': milp_row['Gap'],
        'GA_Tabu_C_max': ga_match['C_max'].values[0] if len(ga_match) > 0 else '—',
        'GA_Tabu_CPU_time': '—',  # To be filled
    }
    
    output_rows.append(row_dict)

# Write combined results
with open('Combined_Results.csv', 'w', newline='') as f:
    fieldnames = ['Customers', 'Depot location', 'Instance', 'Beta', 
                  'MILP_CPU_time', 'MILP_Objective', 'MILP_Gap',
                  'GA_Tabu_C_max', 'GA_Tabu_CPU_time']
    writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
    writer.writeheader()
    writer.writerows(output_rows)

print("Combined results written to Combined_Results.csv")
print("\nFirst 10 rows:")
for row in output_rows[:10]:
    print(f"{row['Customers']};{row['Depot location']};{row['Instance']};{row['Beta']};{row['MILP_Objective']};{row['GA_Tabu_C_max']}")
