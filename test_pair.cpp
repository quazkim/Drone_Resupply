#include <iostream>
#include "pdp_types.h"
#include "pdp_reader.h"
using namespace std;

int main() {
    PDPData data;
    readPDPFile("U_10_0.5_Num_1_pd.txt", data);
    
    cout << "Node index -> Type, PairID:" << endl;
    for (int i = 0; i < data.numNodes; ++i) {
        cout << "  Node[" << i << "] = " << data.nodeTypes[i] 
             << ", pair=" << data.pairIds[i] << endl;
    }
    return 0;
}
