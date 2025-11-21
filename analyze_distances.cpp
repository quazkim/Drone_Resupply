#include <iostream>
#include <iomanip>
#include <cmath>
#include "pdp_types.h"
#include "pdp_reader.h"
#include "pdp_init.h"
using namespace std;

int main() {
    PDPData data;
    readPDPFile("U_100_3.0_Num_1_pd.txt", data);
    
    cout << "\n=== DRONE ENDURANCE CHECK ===" << endl;
    cout << "Drone Endurance: " << data.droneEndurance << " minutes" << endl;
    cout << "Drone Speed: " << data.droneSpeed << " km/h" << endl;
    
    double maxDistance = 0;
    int maxNode = -1;
    
    for (int i = 0; i < data.numNodes; i++) {
        if (i == data.depotIndex) continue;
        
        double dist = getDroneDistance(data, data.depotIndex, i);
        double timeOneWay = dist / data.droneSpeed * 60.0; // minutes
        double timeRoundTrip = timeOneWay * 2 + data.depotDroneLoadTime + data.resupplyTime;
        
        if (dist > maxDistance) {
            maxDistance = dist;
            maxNode = i;
        }
        
        if (timeRoundTrip > data.droneEndurance) {
            cout << "\n❌ Node " << i << " (" << data.nodeTypes[i] << "): TOO FAR!" << endl;
            cout << "   Distance: " << fixed << setprecision(2) << dist << " km" << endl;
            cout << "   Round trip time: " << timeRoundTrip << " min (Exceeds " 
                 << data.droneEndurance << " min)" << endl;
        }
    }
    
    cout << "\n📊 Farthest node: " << maxNode << " (" << data.nodeTypes[maxNode] << ")" << endl;
    cout << "   Distance: " << fixed << setprecision(2) << maxDistance << " km" << endl;
    double maxTime = (maxDistance / data.droneSpeed * 60.0) * 2 + data.depotDroneLoadTime + data.resupplyTime;
    cout << "   Round trip time: " << maxTime << " min" << endl;
    cout << "   Status: " << (maxTime <= data.droneEndurance ? "✅ OK" : "❌ TOO FAR") << endl;
    
    return 0;
}
