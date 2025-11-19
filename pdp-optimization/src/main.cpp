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
        int currentDepot = data.depotIndex - 1; // 0-indexed
        
        // Initialize the current load for each truck
        vector<int> currentLoad(data.numTrucks, 0);
        
        // Create a list of customers (P and DL)
        vector<int> customers;
        for (int i = 0; i < data.nodeTypes.size(); i++) {
            if (data.nodeTypes[i] == "P" || data.nodeTypes[i] == "DL") {
                customers.push_back(i + 1); // 1-indexed
            }
        }
        
        // Start the greedy assignment
        while (!customers.empty()) {
            double bestWeight = numeric_limits<double>::max();
            int bestCustomer = -1;
            int bestTruck = -1;
            int bestPosition = -1;

            // Evaluate each truck for each customer
            for (int truck = 0; truck < data.numTrucks; truck++) {
                for (int i = 0; i < customers.size(); i++) {
                    int customer = customers[i];
                    if (currentLoad[truck] < data.truckCapacity) {
                        // Calculate travel time from current position to the customer
                        double travelTime = distMatrix[currentDepot][customer - 1] / data.truckSpeed * 60; // Convert to minutes
                        double readyTime = data.readyTimes[customer - 1];
                        double weight = travelTime + readyTime;

                        // Check if this is the best option
                        if (weight < bestWeight) {
                            bestWeight = weight;
                            bestCustomer = customer;
                            bestTruck = truck;
                            bestPosition = i;
                        }
                    }
                }
            }

            // If a best customer was found, assign it to the best truck
            if (bestCustomer != -1) {
                routes[bestTruck].push_back(bestCustomer);
                currentLoad[bestTruck] += 1;
                visited[bestCustomer] = true;
                customers.erase(customers.begin() + bestPosition); // Remove from available customers
            } else {
                break; // No more customers can be assigned
            }
        }

        // Convert routes to sequence format
        vector<int> seq;
        for (size_t i = 0; i < routes.size(); i++) {
            for (int nodeId : routes[i]) {
                seq.push_back(nodeId);
            }
            if (i < routes.size() - 1) {
                seq.push_back(0); // Add depot separator
            }
        }

        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }

    return population;
}