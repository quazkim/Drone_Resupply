import os
import random

# Set seed for reproducibility
random.seed(42)

def create_instance_all_d(filename, num_customers, beta):
    """
    Create instance where all customers are type 'D' (delivery/drone only)
    Format: id X Y type ready_time pair_id
    """
    lines = ["# id X Y type ready_time pair_id"]
    
    for i in range(1, num_customers + 1):
        x = round(random.uniform(0, 20), 2)
        y = round(random.uniform(0, 20), 2)
        ready_time = random.randint(0, 100)
        
        # All customers are type 'D' (delivery by drone)
        # pair_id = 0 means no pairing (single customer)
        line = f"{i} {x} {y} D {ready_time} 0"
        lines.append(line)
    
    with open(filename, 'w') as f:
        f.write('\n'.join(lines) + '\n')

# Create instances for U_10, U_15, U_20 with different betas
configs = [
    (10, '0.5'), (10, '1.0'), (10, '1.5'),
    (15, '0.5'), (15, '1.0'), (15, '1.5'),
    (20, '0.5'), (20, '1.0'), (20, '1.5'),
]

base_dir = "."
count = 0

for num_customers, beta in configs:
    for instance_num in range(1, 6):
        filename = f"{base_dir}/U_{num_customers}_{beta}_Num_{instance_num}_all_d.txt"
        create_instance_all_d(filename, num_customers, beta)
        count += 1
        print(f"Created: {filename}")

print(f"\n✓ Total: {count} instances created (all type D)")
