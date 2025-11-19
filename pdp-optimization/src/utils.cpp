// 5. Greedy Time-based Initialization for PDP
vector<vector<int>> initGreedyTimeBasedPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    
    // Build distance matrix
    vector<vector<double>> distMatrix = buildDistanceMatrix(data);
    
    for (int p = 0; p < populationSize; p++) {
        vector<vector<int>> routes(data.numTrucks);
        vector<bool> visited(data.numNodes + 1, false);
        int currentTruck = 0;
        int currentLoad = 0;
        int currentPos = data.depotIndex - 1; // 0-indexed
        
        // Start with the depot
        visited[data.depotIndex] = true;
        
        while (true) {
            // Find the next customer to visit based on the greedy weight
            double bestWeight = numeric_limits<double>::max();
            int bestCustomer = -1;
            
            for (int i = 1; i <= data.numNodes; i++) {
                if (!visited[i] && (data.nodeTypes[i - 1] == "P" || data.nodeTypes[i - 1] == "DL")) {
                    double travelTime = distMatrix[currentPos][i - 1] / data.truckSpeed * 60; // Convert to minutes
                    double readyTime = data.readyTimes[i - 1];
                    double weight = travelTime + readyTime;
                    
                    if (weight < bestWeight) {
                        bestWeight = weight;
                        bestCustomer = i;
                    }
                }
            }
            
            // If no more customers can be visited, break
            if (bestCustomer == -1) {
                break;
            }
            
            // Add the best customer to the current route
            routes[currentTruck].push_back(bestCustomer);
            visited[bestCustomer] = true;
            currentLoad += 1; // Assuming each customer has a load of 1
            
            // Update current position
            currentPos = bestCustomer - 1; // Update to 0-indexed
            
            // Check if the current truck has reached its capacity
            if (currentLoad >= data.truckCapacity) {
                // Move to the next truck
                currentTruck++;
                currentLoad = 0;
                currentPos = data.depotIndex - 1; // Reset to depot
                if (currentTruck >= data.numTrucks) {
                    break; // No more trucks available
                }
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