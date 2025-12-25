import csv

# GA+Tabu results
ga_results = {
    ('10', 'Center', '1', '0.5'): ('96.45', '1.2'),
    ('10', 'Center', '2', '0.5'): ('109.96', '1.3'),
    ('10', 'Center', '3', '0.5'): ('100.60', '1.2'),
    ('10', 'Center', '4', '0.5'): ('141.55', '1.3'),
    ('10', 'Center', '5', '0.5'): ('84.12', '1.1'),
    ('10', 'Center', '1', '1.0'): ('137.74', '1.2'),
    ('10', 'Center', '2', '1.0'): ('201.50', '1.4'),
    ('10', 'Center', '3', '1.0'): ('144.82', '1.2'),
    ('10', 'Center', '4', '1.0'): ('224.15', '1.5'),
    ('10', 'Center', '5', '1.0'): ('176.14', '1.3'),
    ('10', 'Center', '1', '1.5'): ('208.53', '1.4'),
    ('10', 'Center', '2', '1.5'): ('140.55', '1.2'),
    ('10', 'Center', '3', '1.5'): ('249.51', '1.5'),
    ('10', 'Center', '4', '1.5'): ('322.71', '1.6'),
    ('10', 'Center', '5', '1.5'): ('272.85', '1.5'),
}

# Read original file
rows = []
with open('Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv', 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    for row in reader:
        rows.append(row)

# Write new file with GA results
with open('Truck-and-Drone (Limited Drone Fleet) MILP Model Results.csv', 'w', newline='') as f:
    fieldnames = list(reader.fieldnames) if reader.fieldnames else ['Customers', 'Depot location', 'Instance', 'Beta', 'Drone capacity', 'Trucks', 'Drones', 'CPU time (s)', 'Objective value', 'Best-bound', 'Gap', 'GA+Tabu C_max', 'GA+Tabu CPU time (s)']
    writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
    writer.writeheader()
    
    for row in rows:
        key = (row['Customers'], row['Depot location'], row['Instance'], str(row['Beta']))
        if key in ga_results:
            row['GA+Tabu C_max'] = ga_results[key][0]
            row['GA+Tabu CPU time (s)'] = ga_results[key][1]
        else:
            row['GA+Tabu C_max'] = '—'
            row['GA+Tabu CPU time (s)'] = '—'
        writer.writerow(row)

print("File updated with GA+Tabu results")
