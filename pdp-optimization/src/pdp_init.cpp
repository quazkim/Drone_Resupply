// 5. Greedy Time-based Initialization for PDP
vector<vector<int>> initGreedyTimeBasedPDP(int populationSize, const PDPData& data) {
    vector<vector<int>> population;
    random_device rd;
    mt19937 gen(rd());
    
    // Build distance matrix
    vector<vector<double>> distMatrix = buildDistanceMatrix(data);
    
    for (int p = 0; p < populationSize; p++) {
        vector<vector<int>> routes(data.numTrucks);
        vector<bool> visited(data.numNodes + 1, false); // Track visited nodes
        int currentDepot = data.depotIndex - 1; // 0-indexed
        
        // For each truck
        for (int t = 0; t < data.numTrucks; t++) {
            int currentLoad = 0; // Current load of the truck
            int currentPos = currentDepot; // Start at depot
            visited[currentDepot] = true; // Mark depot as visited
            
            while (true) {
                double minWeight = numeric_limits<double>::max();
                int nextNode = -1;

                // Find the next node with the minimum greedy weight
                for (int i = 1; i <= data.numNodes; i++) {
                    if (!visited[i] && (data.nodeTypes[i - 1] == "P" || data.nodeTypes[i - 1] == "DL")) {
                        double travelTime = distMatrix[currentPos][i - 1] / data.truckSpeed * 60; // Convert to minutes
                        double readyTime = data.readyTimes[i - 1];
                        double weight = travelTime + readyTime;

                        if (weight < minWeight) {
                            minWeight = weight;
                            nextNode = i;
                        }
                    }
                }

                // If no next node is found, break
                if (nextNode == -1) {
                    break;
                }

                // Add the selected node to the current route
                routes[t].push_back(nextNode);
                visited[nextNode] = true; // Mark as visited
                currentPos = nextNode; // Update current position
                currentLoad += 1; // Update load (assuming each node has a load of 1)
            }
            
            // Return to depot if the route has customers
            if (!routes[t].empty()) {
                routes[t].insert(routes[t].begin(), data.depotIndex); // Start from depot
                routes[t].push_back(data.depotIndex); // Return to depot
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

        quickRepairPDP(seq, data, gen); // Repair the sequence
        population.push_back(seq);
    }
    
    return population;
}