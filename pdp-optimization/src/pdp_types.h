// 5. Greedy Time-based Initialization for PDP
vector<vector<int>> initGreedyTimeBasedPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    
    // Build the distance matrix
    vector<vector<double>> distMatrix = buildDistanceMatrix(data);
    
    for (int p = 0; p < populationSize; p++) {
        vector<vector<int>> routes(data.numTrucks);
        vector<bool> visited(data.numNodes + 1, false);
        vector<int> routeLoad(data.numTrucks, 0);
        vector<int> routePosition(data.numTrucks, data.depotIndex - 1); // Current position (0-indexed)
        
        // Initialize the depot
        for (int t = 0; t < data.numTrucks; t++) {
            routes[t].push_back(data.depotIndex); // Start from depot
        }
        
        // Greedy assignment based on time weight
        while (true) {
            double bestWeight = numeric_limits<double>::max();
            int bestTruck = -1;
            int bestCustomer = -1;
            int bestPosition = -1;

            for (int t = 0; t < data.numTrucks; t++) {
                for (int customer = 1; customer <= data.numNodes; customer++) {
                    if (!visited[customer] && (data.nodeTypes[customer - 1] == "P" || data.nodeTypes[customer - 1] == "DL")) {
                        // Calculate travel time to the customer
                        double travelTime = distMatrix[routePosition[t]][customer - 1] / data.truckSpeed * 60; // Convert to minutes
                        double readyTime = data.readyTimes[customer - 1];
                        double weight = travelTime + readyTime;

                        // Check if this is the best option
                        if (weight < bestWeight) {
                            bestWeight = weight;
                            bestTruck = t;
                            bestCustomer = customer;
                            bestPosition = routes[t].size(); // Insert at the end of the current route
                        }
                    }
                }
            }

            // If no valid customer found, break
            if (bestTruck == -1) break;

            // Assign the best customer to the best truck
            routes[bestTruck].push_back(bestCustomer);
            visited[bestCustomer] = true;
            routePosition[bestTruck] = bestCustomer - 1; // Update the current position for the truck
        }

        // Return to depot for each route
        for (int t = 0; t < data.numTrucks; t++) {
            if (!routes[t].empty()) {
                routes[t].push_back(data.depotIndex); // End route at depot
            }
        }

        // Convert routes to sequence format
        vector<int> seq;
        for (size_t i = 0; i < routes.size(); i++) {
            for (int nodeId : routes[i]) {
                seq.push_back(nodeId);
            }
            if (i < routes.size() - 1) {
                seq.push_back(0); // Route separator
            }
        }

        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }

    return population;
}