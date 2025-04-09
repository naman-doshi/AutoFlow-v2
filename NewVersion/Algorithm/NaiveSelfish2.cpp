#include <bits/stdc++.h>
#include <chrono> // For timing functionality
using namespace std;
#include "Landscape.cpp"

#define rep(i, a, b) for(int i = a; i < (b); ++i)

// Basic data structures
unordered_map<int, Intersection> intersections;
unordered_map<int, Road> roads;
unordered_map<int, Vehicle> vehicles;
vector<pair<int, Vehicle>> vehicleVector;
unordered_map<int, unordered_map<int, Road>> graph;

class PathNode {
public:
    Intersection intersection;
    shared_ptr<PathNode> parent;
    float g;
    float h;
    float f;

    PathNode(Intersection intersection, shared_ptr<PathNode> parent = nullptr) {
        this->intersection = intersection;
        this->parent = parent;
        this->g = 0;
        this->h = 0;
        this->f = 0;
    }

    bool operator==(const PathNode& other) const {
        return intersection.id == other.intersection.id;
    }

    bool operator<(const PathNode& other) const {
        return f > other.f; // For priority queue (min-heap)
    }
};

float avgSpeed = 0;
float avgIntersectionDelay = 2.0f;

// Simple heuristic function - Euclidean distance
float heuristic(const Intersection& a, const Intersection& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) / avgSpeed;
}



map<int, map<int, float>> congestionMap; // Map to store congestion levels (car, road) -> avg speed

void readData() {
    
    int numIntersections;
    cin >> numIntersections;
    for (int i = 0; i < numIntersections; ++i) {
        Intersection intersection;
        cin >> intersection.id >> intersection.trafficLightDuration >> intersection.roadCount >> intersection.x >> intersection.y;
        intersections[intersection.id] = intersection;
    }

    int numRoads;
    cin >> numRoads;
    for (int i = 0; i < numRoads; ++i) {
        Road road;
        int numPositions;
        int int1Id, int2Id;
        cin >> road.id >> road.length >> road.speedLimit >> road.capacity >> int1Id >> int2Id >> road.traversalTime >> road.laneCount;
        avgSpeed += road.speedLimit;
        road.int1Id = int1Id;
        road.int2Id = int2Id;
        intersections[int1Id].connectingIntersectionIDs.push_back(int2Id);
        intersections[int2Id].connectingIntersectionIDs.push_back(int1Id);
        cin >> numPositions;
        //cout << numAssociatedVirtualIntersections << endl;
        road.positions = vector<vector<double>>(numPositions, vector<double>(3));
        for (int j = 0; j < numPositions; ++j) {
            cin >> road.positions[j][0] >> road.positions[j][1] >> road.positions[j][2];
        }
        //cout << road.associatedVirtualIntersectionIds.size() << endl;
        roads[road.id] = road;
    }

    avgSpeed /= numRoads;

    int numGraphEntries;
    cin >> numGraphEntries;
    for (int i = 0; i < numGraphEntries; ++i) {
        int num2;
        cin >> num2;
        for (int j = 0; j < num2; ++j) {
            int int1Id, int2Id, roadId;
            cin >> int1Id >> int2Id >> roadId;
            graph[int1Id][int2Id] = roads[roadId];
        }
    }


    int numVehicles;
    cin >> numVehicles;
    for (int i = 0; i < numVehicles; ++i) {
        Vehicle vehicle;
        int roadId;
        double s1, s2, s3, e1, e2, e3;
        int startingRoadId, endingRoadId;
        cin >> vehicle.id >> startingRoadId >> endingRoadId >> s1 >> s2 >> s3 >> e1 >> e2 >> e3 >> vehicle.passengerCount >> vehicle.emissionRate;
        vehicle.startingRoadId = startingRoadId;
        vehicle.endingRoadId = endingRoadId;
        vehicle.starting = {s1, s2, s3};
        vehicle.ending = {e1, e2, e3};
        
        vehicles[vehicle.id] = vehicle;
        vehicleVector.push_back({vehicle.id, vehicle});
    }

    cout << "Loaded " << intersections.size() << " intersections, "
         << roads.size() << " roads, "
         << vehicles.size() << " vehicles." << endl;
    
    // read in vehicle_road_speeds.csv
    ifstream csvFile("vehicle_road_speeds.csv");
    if (!csvFile.is_open()) {
        cout << "Failed to open vehicle_road_speeds.csv" << endl;
    } else {
        string line, value;
        vector<int> roadIds;

        // Read header row to get road IDs
        if (getline(csvFile, line)) {
            stringstream ss(line);
            
            // Skip the first column header
            getline(ss, value, ',');
            
            // Read road IDs from header
            while (getline(ss, value, ',')) {
                if (!value.empty()) {
                    roadIds.push_back(stoi(value));
                }
            }
        }

        // Read each data row
        while (getline(csvFile, line)) {
            stringstream ss(line);
            
            // First value is vehicle ID
            getline(ss, value, ',');
            int vehicleId = stoi(value);
            
            int colIdx = 0;
            
            // Read speeds for each road
            while (getline(ss, value, ',') && colIdx < roadIds.size()) {
                if (!value.empty()) {
                    int roadId = roadIds[colIdx];
                    float speed = stof(value);
                    congestionMap[vehicleId][roadId] = speed;
                    colIdx++;
                }
            }
        }
        csvFile.close();
        cout << "Loaded vehicle-road speeds from CSV." << endl;
    }
}

// Simple traversal time calculation - using only the road's base traversal time
float calculateTraversalTime(Vehicle & vehicle, Road& road) {
    // Just use the road's base traversal time
    if (congestionMap[vehicle.id][road.id] == 0)
        return road.traversalTime;
    else 
        return road.length * congestionMap[vehicle.id][road.id];
}

// Contraction Hierarchies data structures
struct CHNode {
    int id;
    int level = 0;
    int importance = 0;
    bool contracted = false;
};

// Shortcut edge for CH
struct Shortcut {
    int from;
    int to;
    int via;
    float weight;
};

// CH specific variables
unordered_map<int, CHNode> chNodes;
unordered_map<int, unordered_map<int, float>> upwardGraph;   // (u,v) where level(u) < level(v)
unordered_map<int, unordered_map<int, float>> downwardGraph; // (u,v) where level(u) > level(v)
unordered_map<int, unordered_map<int, Shortcut>> shortcuts;

// Calculate node importance for contraction order
int calculateNodeImportance(int nodeId) {
    CHNode& node = chNodes[nodeId];
    
    // Count incoming and outgoing edges
    int edgeCount = 0;
    int shortcutCount = 0;
    
    // Count neighbors that haven't been contracted
    for (int neighborId : intersections[nodeId].connectingIntersectionIDs) {
        if (!chNodes[neighborId].contracted) {
            edgeCount++;
            
            // Estimate shortcuts that would be created
            for (int otherNeighborId : intersections[nodeId].connectingIntersectionIDs) {
                if (otherNeighborId != neighborId && !chNodes[otherNeighborId].contracted) {
                    shortcutCount++;
                }
            }
        }
    }
    
    // Simple importance formula: more shortcuts = more important
    return shortcutCount - edgeCount;
}

// File handling for CH preprocessing data
string getCHDataFilePath() {
    return "ch_preprocessed_data.bin";
}

// Save CH preprocessing data to a file
bool saveCHDataToFile() {
    string filePath = getCHDataFilePath();
    cout << "Saving CH data to file: " << filePath << endl;
    
    ofstream outFile(filePath, ios::binary);
    if (!outFile) {
        cerr << "Failed to open file for writing: " << filePath << endl;
        return false;
    }
    
    // Save number of nodes and their data
    int nodeCount = chNodes.size();
    outFile.write(reinterpret_cast<const char*>(&nodeCount), sizeof(nodeCount));
    for (auto& [id, node] : chNodes) {
        outFile.write(reinterpret_cast<const char*>(&id), sizeof(id));
        outFile.write(reinterpret_cast<const char*>(&node.level), sizeof(node.level));
        outFile.write(reinterpret_cast<const char*>(&node.importance), sizeof(node.importance));
        outFile.write(reinterpret_cast<const char*>(&node.contracted), sizeof(node.contracted));
    }
    
    // Save upward graph
    int upGraphSize = 0;
    for (auto& [from, edges] : upwardGraph) {
        upGraphSize += edges.size();
    }
    outFile.write(reinterpret_cast<const char*>(&upGraphSize), sizeof(upGraphSize));
    for (auto& [from, edges] : upwardGraph) {
        for (auto& [to, weight] : edges) {
            outFile.write(reinterpret_cast<const char*>(&from), sizeof(from));
            outFile.write(reinterpret_cast<const char*>(&to), sizeof(to));
            outFile.write(reinterpret_cast<const char*>(&weight), sizeof(weight));
        }
    }
    
    // Save downward graph
    int downGraphSize = 0;
    for (auto& [from, edges] : downwardGraph) {
        downGraphSize += edges.size();
    }
    outFile.write(reinterpret_cast<const char*>(&downGraphSize), sizeof(downGraphSize));
    for (auto& [from, edges] : downwardGraph) {
        for (auto& [to, weight] : edges) {
            outFile.write(reinterpret_cast<const char*>(&from), sizeof(from));
            outFile.write(reinterpret_cast<const char*>(&to), sizeof(to));
            outFile.write(reinterpret_cast<const char*>(&weight), sizeof(weight));
        }
    }
    
    // Save shortcuts
    int shortcutsSize = 0;
    for (auto& [from, toShortcuts] : shortcuts) {
        shortcutsSize += toShortcuts.size();
    }
    outFile.write(reinterpret_cast<const char*>(&shortcutsSize), sizeof(shortcutsSize));
    for (auto& [from, toShortcuts] : shortcuts) {
        for (auto& [to, shortcut] : toShortcuts) {
            outFile.write(reinterpret_cast<const char*>(&shortcut.from), sizeof(shortcut.from));
            outFile.write(reinterpret_cast<const char*>(&shortcut.to), sizeof(shortcut.to)); // Fixed parenthesis here
            outFile.write(reinterpret_cast<const char*>(&shortcut.via), sizeof(shortcut.via));
            outFile.write(reinterpret_cast<const char*>(&shortcut.weight), sizeof(shortcut.weight));
        }
    }
    
    outFile.close();
    cout << "CH data saved successfully" << endl;
    return true;
}

// Load CH preprocessing data from a file
bool loadCHDataFromFile() {
    string filePath = getCHDataFilePath();
    ifstream inFile(filePath, ios::binary);
    if (!inFile) {
        cout << "CH preprocessed data file not found: " << filePath << endl;
        return false;
    }
    
    cout << "Loading CH data from file: " << filePath << endl;
    
    // Clear existing data
    chNodes.clear();
    upwardGraph.clear();
    downwardGraph.clear();
    shortcuts.clear();
    
    try {
        // Load nodes
        int nodeCount;
        inFile.read(reinterpret_cast<char*>(&nodeCount), sizeof(nodeCount));
        for (int i = 0; i < nodeCount; i++) {
            int id;
            CHNode node;
            inFile.read(reinterpret_cast<char*>(&id), sizeof(id));
            inFile.read(reinterpret_cast<char*>(&node.level), sizeof(node.level));
            inFile.read(reinterpret_cast<char*>(&node.importance), sizeof(node.importance));
            inFile.read(reinterpret_cast<char*>(&node.contracted), sizeof(node.contracted));
            node.id = id;
            chNodes[id] = node;
        }
        
        // Load upward graph
        int upGraphSize;
        inFile.read(reinterpret_cast<char*>(&upGraphSize), sizeof(upGraphSize));
        for (int i = 0; i < upGraphSize; i++) {
            int from, to;
            float weight;
            inFile.read(reinterpret_cast<char*>(&from), sizeof(from));
            inFile.read(reinterpret_cast<char*>(&to), sizeof(to));
            inFile.read(reinterpret_cast<char*>(&weight), sizeof(weight));
            upwardGraph[from][to] = weight;
        }
        
        // Load downward graph
        int downGraphSize;
        inFile.read(reinterpret_cast<char*>(&downGraphSize), sizeof(downGraphSize));
        for (int i = 0; i < downGraphSize; i++) {
            int from, to;
            float weight;
            inFile.read(reinterpret_cast<char*>(&from), sizeof(from));
            inFile.read(reinterpret_cast<char*>(&to), sizeof(to));
            inFile.read(reinterpret_cast<char*>(&weight), sizeof(weight));
            downwardGraph[from][to] = weight;
        }
        
        // Load shortcuts
        int shortcutsSize;
        inFile.read(reinterpret_cast<char*>(&shortcutsSize), sizeof(shortcutsSize));
        for (int i = 0; i < shortcutsSize; i++) {
            Shortcut shortcut;
            inFile.read(reinterpret_cast<char*>(&shortcut.from), sizeof(shortcut.from));
            inFile.read(reinterpret_cast<char*>(&shortcut.to), sizeof(shortcut.to));
            inFile.read(reinterpret_cast<char*>(&shortcut.via), sizeof(shortcut.via));
            inFile.read(reinterpret_cast<char*>(&shortcut.weight), sizeof(shortcut.weight));
            shortcuts[shortcut.from][shortcut.to] = shortcut;
        }
        
        inFile.close();
        cout << "CH data loaded successfully" << endl;
        cout << "Loaded " << chNodes.size() << " nodes, " 
             << upGraphSize << " upward edges, " 
             << downGraphSize << " downward edges, and " 
             << shortcutsSize << " shortcuts" << endl;
        return true;
    }
    catch (const exception& e) {
        cerr << "Error reading CH data file: " << e.what() << endl;
        return false;
    }
}

// Preprocess graph using Contraction Hierarchies
void preprocessCH(bool saveToFile = true) {
    cout << "Preprocessing road network with Contraction Hierarchies..." << endl;
    auto startTime = chrono::high_resolution_clock::now();
    
    // Initialize CH nodes
    for (auto& [id, intersection] : intersections) {
        chNodes[id] = {id, 0, 0, false};
    }
    
    // Initialize CH graphs with original edges
    for (auto& [fromId, neighbors] : graph) {
        for (auto& [toId, road] : neighbors) {
            float weight = calculateTraversalTime(vehicles[road.id], road);
            upwardGraph[fromId][toId] = weight;
            downwardGraph[toId][fromId] = weight;
        }
    }
    
    // Process nodes in order of importance
    vector<pair<int, int>> nodeOrder; // pair<importance, nodeId>
    
    // Calculate initial node importance
    for (auto& [id, _] : chNodes) {
        int importance = calculateNodeImportance(id);
        chNodes[id].importance = importance;
        nodeOrder.push_back({importance, id});
    }
    
    // Sort by importance (lower values first)
    sort(nodeOrder.begin(), nodeOrder.end());
    
    // Contract nodes
    int level = 0;
    for (auto& [_, nodeId] : nodeOrder) {
        CHNode& node = chNodes[nodeId];
        if (node.contracted) continue;
        
        // Contract this node
        node.contracted = true;
        node.level = level++;
        
        // Find all incoming neighbors that haven't been contracted
        vector<int> inNeighbors;
        for (auto& [from, edges] : downwardGraph) {
            if (!chNodes[from].contracted && edges.count(nodeId)) {
                inNeighbors.push_back(from);
            }
        }
        
        // Find all outgoing neighbors that haven't been contracted
        vector<int> outNeighbors;
        for (auto& [to, _] : upwardGraph[nodeId]) {
            if (!chNodes[to].contracted) {
                outNeighbors.push_back(to);
            }
        }
        
        // For each pair of (in, out) neighbors, check if shortcut is needed
        for (int from : inNeighbors) {
            float weightFromVia = downwardGraph[from][nodeId];
            
            for (int to : outNeighbors) {
                if (from == to) continue;
                
                float weightViaTo = upwardGraph[nodeId][to];
                float totalWeight = weightFromVia + weightViaTo;
                
                // Check if we need a shortcut (i.e., if this path is shortest)
                bool needShortcut = true;
                
                // Simplified shortcut witness search - in a real implementation this would be a local Dijkstra
                if (upwardGraph[from].count(to)) {
                    if (upwardGraph[from][to] <= totalWeight) {
                        needShortcut = false;
                    }
                }
                
                if (needShortcut) {
                    // Add shortcut to both graphs based on node levels
                    if (node.level < chNodes[to].level) {
                        upwardGraph[from][to] = totalWeight;
                    }
                    if (node.level < chNodes[from].level) {
                        downwardGraph[to][from] = totalWeight;
                    }
                    
                    // Store shortcut information
                    shortcuts[from][to] = {from, to, nodeId, totalWeight};
                }
            }
        }
        
        // Re-calculate importance for neighbors
        for (int neighborId : intersections[nodeId].connectingIntersectionIDs) {
            if (!chNodes[neighborId].contracted) {
                chNodes[neighborId].importance = calculateNodeImportance(neighborId);
            }
        }
    }
    
    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    cout << "CH preprocessing completed in " << duration.count() << " ms" << endl;
    cout << "Created " << shortcuts.size() << " shortcuts" << endl;
    
    // Save preprocessed data if requested
    if (saveToFile) {
        saveCHDataToFile();
    }
}

// Expand a shortcut into its constituent path segments
vector<int> expandShortcut(int fromId, int toId) {
    // Check if this is a direct shortcut
    auto shortcutIt = shortcuts.find(fromId);
    if (shortcutIt != shortcuts.end() && shortcutIt->second.count(toId)) {
        Shortcut& shortcut = shortcutIt->second[toId];
        
        // This is a shortcut through another node (via)
        int viaId = shortcut.via;
        
        // Recursively expand both segments
        vector<int> firstSegment = expandShortcut(fromId, viaId);
        vector<int> secondSegment = expandShortcut(viaId, toId);
        
        // Combine segments (remove duplicate via node)
        firstSegment.insert(firstSegment.end(), secondSegment.begin() + 1, secondSegment.end());
        return firstSegment;
    }
    
    // This is a direct edge in the original graph or a base CH edge
    return {fromId, toId};
}

// CH bidirectional search
vector<Intersection> CHQuery(int startId, int endId) {
    // Early exit for same node
    if (startId == endId) {
        return {intersections[startId]};
    }
    
    // Forward search from start
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> forwardQueue;
    unordered_map<int, float> forwardDist;
    unordered_map<int, int> forwardPrev;
    
    // Backward search from end
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> backwardQueue;
    unordered_map<int, float> backwardDist;
    unordered_map<int, int> backwardPrev;
    
    // Initialize
    forwardQueue.push({0, startId});
    forwardDist[startId] = 0;
    
    backwardQueue.push({0, endId});
    backwardDist[endId] = 0;
    
    float bestDist = numeric_limits<float>::infinity();
    int meetingNode = -1;
    
    // Bidirectional search
    while (!forwardQueue.empty() && !backwardQueue.empty()) {
        // Check if we can terminate
        if (forwardQueue.top().first + backwardQueue.top().first >= bestDist) {
            break;
        }
        
        // Forward search
        auto [forwardDist_u, u] = forwardQueue.top();
        forwardQueue.pop();
        
        // Already found a better path
        if (forwardDist_u > forwardDist[u]) continue;
        
        // Check for path through this node
        if (backwardDist.count(u)) {
            float pathDist = forwardDist_u + backwardDist[u];
            if (pathDist < bestDist) {
                bestDist = pathDist;
                meetingNode = u;
            }
        }
        
        // Explore upward edges
        for (auto& [v, weight] : upwardGraph[u]) {
            float newDist = forwardDist_u + weight;
            if (!forwardDist.count(v) || newDist < forwardDist[v]) {
                forwardDist[v] = newDist;
                forwardPrev[v] = u;
                forwardQueue.push({newDist, v});
            }
        }
        
        // Backward search
        auto [backwardDist_u, u_back] = backwardQueue.top();
        backwardQueue.pop();
        
        // Already found a better path
        if (backwardDist_u > backwardDist[u_back]) continue;
        
        // Check for path through this node
        if (forwardDist.count(u_back)) {
            float pathDist = backwardDist_u + forwardDist[u_back];
            if (pathDist < bestDist) {
                bestDist = pathDist;
                meetingNode = u_back;
            }
        }
        
        // Explore downward edges
        for (auto& [v, weight] : downwardGraph[u_back]) {
            float newDist = backwardDist_u + weight;
            if (!backwardDist.count(v) || newDist < backwardDist[v]) {
                backwardDist[v] = newDist;
                backwardPrev[v] = u_back;
                backwardQueue.push({newDist, v});
            }
        }
    }
    
    // Reconstruct basic path
    if (meetingNode == -1) {
        return {}; // No path found
    }
    
    // Forward path (start -> meeting)
    vector<int> forwardPath;
    int current = meetingNode;
    while (current != startId) {
        forwardPath.push_back(current);
        current = forwardPrev[current];
    }
    forwardPath.push_back(startId);
    reverse(forwardPath.begin(), forwardPath.end());
    
    // Backward path (meeting -> end)
    vector<int> backwardPath;
    current = meetingNode;
    while (current != endId) {
        current = backwardPrev[current];
        backwardPath.push_back(current);
    }
    
    // Expand all shortcuts in the path
    vector<int> expandedPath;
    expandedPath.push_back(forwardPath[0]);
    
    // Expand each segment in the forward path
    for (size_t i = 0; i < forwardPath.size() - 1; i++) {
        vector<int> expandedSegment = expandShortcut(forwardPath[i], forwardPath[i+1]);
        // Add all nodes except the first (already in expandedPath)
        expandedPath.insert(expandedPath.end(), expandedSegment.begin() + 1, expandedSegment.end());
    }
    
    // Save the meeting point index
    size_t meetingPointIndex = expandedPath.size() - 1;
    
    // Expand each segment in the backward path
    for (size_t i = 0; i < backwardPath.size(); i++) {
        int fromId = (i == 0) ? meetingNode : backwardPath[i-1];
        int toId = backwardPath[i];
        
        vector<int> expandedSegment = expandShortcut(fromId, toId);
        // Add all nodes except the first (already in expandedPath)
        expandedPath.insert(expandedPath.end(), expandedSegment.begin() + 1, expandedSegment.end());
    }
    
    // Remove any duplicate nodes that might occur at junctions
    expandedPath.erase(unique(expandedPath.begin(), expandedPath.end()), expandedPath.end());
    
    // Convert node IDs to Intersection objects
    vector<Intersection> fullPath;
    for (int id : expandedPath) {
        fullPath.push_back(intersections[id]);
    }
    
    // Validate the path to ensure all consecutive nodes are connected
    bool isValid = true;
    string validationError = "";
    
    // Validate that adjacent nodes in path are connected by roads in the original graph
    for (size_t i = 0; i < fullPath.size() - 1; i++) {
        int fromId = fullPath[i].id;
        int toId = fullPath[i + 1].id;
        
        // Check only in the original graph this time
        bool connected = false;
        if ((graph.count(fromId) && graph[fromId].count(toId)) || 
            (graph.count(toId) && graph[toId].count(fromId))) {
            connected = true;
        }
        
        if (!connected) {
            isValid = false;
            validationError = "No connection in original graph between nodes " + 
                              to_string(fromId) + " and " + to_string(toId);
            break;
        }
    }
    
    if (!isValid) {
        // Log the error but don't halt the program
        cerr << "Path validation failed after expansion: " << validationError << endl;
        
        // For debugging purposes, print the invalid path
        cerr << "Invalid expanded path: ";
        for (const auto& intersection : fullPath) {
            cerr << intersection.id << " ";
        }
        cerr << endl;
        
        // Return empty path to indicate failure
        return {};
    }
    
    return fullPath;
}

// CH-based A* implementation
void CHPathfinding() {
    auto startTime = chrono::high_resolution_clock::now();
    
    unordered_map<int, vector<Intersection>> paths;
    int processedCount = 0;
    int pathFoundCount = 0;
    int validationFailures = 0;
    
    cout << "Starting CH-based pathfinding for " << vehicles.size() << " vehicles..." << endl;
    
    // Process each vehicle sequentially
    for (auto& [vehicleId, vehicle] : vehicles) {
        processedCount++;
        
        // Log progress every 100 vehicles
        if (processedCount % 100 == 0) {
            cout << "Processed " << processedCount << "/" << vehicles.size() << " vehicles..." << endl;
        }
        
        // Determine starting and ending intersections
        Intersection starting;
        Road startingRoad = roads[vehicle.startingRoadId];
        if (vehicle.starting[1] == 1) {
            starting = intersections[startingRoad.int2Id];
        } else {
            starting = intersections[startingRoad.int1Id];
        }

        Intersection ending;
        Road endingRoad = roads[vehicle.endingRoadId];
        if (vehicle.ending[1] == 1) {
            ending = intersections[endingRoad.int2Id];
        } else {
            ending = intersections[endingRoad.int1Id];
        }
        
        // Use CH query for pathfinding
        vector<Intersection> path = CHQuery(starting.id, ending.id);
        
        if (!path.empty()) {
            paths[vehicleId] = path;
            pathFoundCount++;
            
            // Only log details for the first 10 vehicles to avoid console spam
            if (pathFoundCount <= 10) {
                cout << "Vehicle " << vehicleId << ": Path found with " << path.size() << " nodes" << endl;
            }
        } else {
            validationFailures++;
            if (pathFoundCount <= 10) {
                cout << "Vehicle " << vehicleId << ": No path found or validation failed!" << endl;
            }
        }
    }
    
    // Stop timing and calculate duration
    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    
    // Output results
    cout << "Found paths for " << paths.size() << "/" << vehicles.size() << " vehicles" << endl;
    cout << "Total execution time: " << duration.count() << " milliseconds" << endl;
    
    // Output validation statistics
    cout << "Path validation failures: " << validationFailures << " vehicles" << endl;
    if (validationFailures > 0) {
        cout << "Path validation failure rate: " << (100.0 * validationFailures / vehicles.size()) << "%" << endl;
    }
    
    // Print detailed statistics
    double pathsPerSecond = (paths.size() * 1000.0) / duration.count();
    cout << "Performance: " << pathsPerSecond << " paths/second" << endl;
    
    cout << "---" << endl;
    
    for (auto& [id, path] : paths) {
        
        
        cout << id << " ";
        for (auto& intersection : path) {
            cout << intersection.id << " ";
        }
        cout << endl;
    }
    
    // Add debug info about path lengths
    int totalNodeCount = 0;
    for (const auto& [_, path] : paths) {
        totalNodeCount += path.size();
    }
    float avgPathLength = paths.empty() ? 0 : float(totalNodeCount) / paths.size();
    //cout << "Average path length: " << avgPathLength << " nodes" << endl;
    
    // if (paths.size() > pathsToShow) {
    //     cout << "... and " << (paths.size() - pathsToShow) << " more paths (not shown)" << endl;
    // }
}

int main() {
    // Start total timing - including data loading
    auto totalStartTime = chrono::high_resolution_clock::now();
    
    readData();
    
    // Try to load CH data from file first
    bool chDataLoaded = loadCHDataFromFile();
    
    // If loading failed, do preprocessing and save to file
    if (!chDataLoaded) {
        cout << "Need to precompute CH data..." << endl;
        preprocessCH(true); // true = save to file
    }
    
    // Run CH pathfinding
    CHPathfinding();
    
    // // Print total execution time
    // auto totalEndTime = chrono::high_resolution_clock::now();
    // auto totalDuration = chrono::duration_cast<chrono::milliseconds>(totalEndTime - totalStartTime);
    // cout << "Total execution time (including " << (chDataLoaded ? "loading" : "preprocessing") << "): " << totalDuration.count() << " ms" << endl;
    
    return 0;
}
