### Explanation:
- **Distance Matrix**: The function starts by building the distance matrix using the existing `buildDistanceMatrix` function.
- **Greedy Selection**: For each unvisited node, it calculates the travel time and the ready time, then selects the node with the minimum greedy weight.
- **Capacity Check**: It checks if adding the next node exceeds the truck's capacity. If it does, it moves to the next truck.
- **Route Formatting**: Finally, it formats the routes into a sequence and applies a quick repair to ensure feasibility before adding it to the population.

### Integration:
You can integrate this function into your `pdp_init.cpp` file and call it in your main program or wherever you need to initialize the population for the PDP problem.