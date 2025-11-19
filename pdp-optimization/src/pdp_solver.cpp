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
        
        for (int t = 0; t < data.numTrucks; t++) {
            int currentLoad = 0;
            int currentPos = currentDepot; // Start at depot
            
            while (true) {
                double minWeight = numeric_limits<double>::max();
                int bestNode = -1;
                
                // Find the best next node to visit
                for (int i = 1; i <= data.numNodes; i++) {
                    if (!visited[i] && (data.nodeTypes[i - 1] == "P" || data.nodeTypes[i - 1] == "DL")) {
                        // Calculate travel time to the next node
                        double travelTime = distMatrix[currentPos][i - 1] / data.truckSpeed * 60; // Convert to minutes
                        double readyTime = data.readyTimes[i - 1];
                        double weight = travelTime + readyTime;
                        
                        if (weight < minWeight) {
                            minWeight = weight;
                            bestNode = i;
                        }
                    }
                }
                
                // If no valid node is found, break the loop
                if (bestNode == -1) {
                    break;
                }
                
                // Update the route and visited status
                routes[t].push_back(bestNode);
                visited[bestNode] = true;
                currentLoad += 1; // Assuming each customer has a load of 1
                currentPos = bestNode - 1; // Update current position
            }
            
            // Return to depot if the route is not empty
            if (!routes[t].empty()) {
                routes[t].insert(routes[t].begin(), data.depotIndex); // Start from depot
                routes[t].push_back(data.depotIndex); // End at depot
            }
        }
        
        // Convert routes to sequence format
        vector<int> seq;
        for (const auto& route : routes) {
            for (int nodeId : route) {
                seq.push_back(nodeId);
            }
            seq.push_back(0); // Route separator
        }
        
        // Remove the last separator if it exists
        if (!seq.empty() && seq.back() == 0) {
            seq.pop_back();
        }
        
        quickRepairPDP(seq, data, gen);
        population.push_back(seq);
    }
    
    return population;
}