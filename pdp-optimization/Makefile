// 5. Greedy Time-based Initialization for PDP
vector<vector<int>> initGreedyTimeBasedPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    
    // Build distance matrix
    vector<vector<double>> distMatrix = buildDistanceMatrix(data);
    
    for (int p = 0; p < populationSize; p++) {
        vector<vector<int>> routes(data.numTrucks);
        vector<int> routeLoad(data.numTrucks, 0);
        vector<int> routePosition(data.numTrucks, data.depotIndex - 1); // Current position (0-indexed)
        vector<bool> visited(data.numNodes + 1, false);
        
        // Start with each truck
        for (int t = 0; t < data.numTrucks; t++) {
            int currentLoad = 0;
            int currentPos = data.depotIndex - 1; // Start at depot
            
            while (true) {
                double minWeight = numeric_limits<double>::max();
                int bestNode = -1;
                
                // Find the best next node to visit
                for (int i = 1; i <= data.numNodes; i++) {
                    if (!visited[i] && (data.nodeTypes[i - 1] == "P" || data.nodeTypes[i - 1] == "DL")) {
                        double travelTime = distMatrix[currentPos][i - 1] / data.truckSpeed * 60; // Convert to minutes
                        double greedyWeight = data.readyTimes[i - 1] + travelTime;
                        
                        if (greedyWeight < minWeight) {
                            minWeight = greedyWeight;
                            bestNode = i;
                        }
                    }
                }
                
                // If no valid node found, break
                if (bestNode == -1) {
                    break;
                }
                
                // Check if adding this node exceeds capacity
                if (currentLoad + 1 <= data.truckCapacity) {
                    routes[t].push_back(bestNode);
                    visited[bestNode] = true;
                    currentLoad += 1;
                    currentPos = bestNode - 1; // Update current position
                } else {
                    break; // Capacity reached, stop adding to this route
                }
            }
            
            // Return to depot if route is not empty
            if (!routes[t].empty()) {
                routes[t].push_back(0); // 0 represents returning to depot
            }
        }
        
        // Ensure enough routes
        while (routes.size() < data.numTrucks) {
            routes.push_back({});
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