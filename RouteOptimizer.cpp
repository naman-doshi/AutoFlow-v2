#include <bits/stdc++.h>
#include <chrono> // For timing functionality
#include <fstream> // For CSV file reading
using namespace std;
#include "NewVersion/Algorithm/Landscape.cpp"

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
        road.positions = vector<vector<double>>(numPositions, vector<double>(3));
        for (int j = 0; j < numPositions; ++j) {
            cin >> road.positions[j][0] >> road.positions[j][1] >> road.positions[j][2];
        }
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
    cout << numVehicles << endl;
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

    cout << numVehicles << endl;
    cout << "Loaded " << intersections.size() << " intersections, "
         << roads.size() << " roads, "
         << vehicles.size() << " vehicles." << endl;


    // read in vehicle_road_speeds.csv
    ifstream csvFile("vehicle_road_speeds.csv");
    if (!csvFile.is_open()) {
        cerr << "Failed to open vehicle_road_speeds.csv" << endl;
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


float calculateTraversalTime(Vehicle& vehicle, Road& road) {
    // Just use the road's base traversal time
    return road.length / congestionMap[vehicle.id][road.id];
}

// Basic A* implementation without optimizations or reservations
void BasicAStar() {
    // Start timing
    auto startTime = chrono::high_resolution_clock::now();
    
    unordered_map<int, vector<Intersection>> paths;
    
    cout << "Starting basic A* pathfinding..." << endl;
    
    // Process each vehicle sequentially
    for (auto& [vehicleId, vehicle] : vehicles) {
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

        // Standard A* search
        priority_queue<PathNode> openSet;
        unordered_set<int> closedSet;
        unordered_map<int, float> gScore;
        unordered_map<int, shared_ptr<PathNode>> nodeMap;
        
        // Initialize the starting node
        auto startNode = make_shared<PathNode>(starting);
        startNode->g = 0;
        startNode->h = heuristic(starting, ending);
        startNode->f = startNode->g + startNode->h;
        
        openSet.push(*startNode);
        gScore[starting.id] = 0;
        nodeMap[starting.id] = startNode;
        
        bool pathFound = false;
        
        while (!openSet.empty()) {
            PathNode current = openSet.top();
            openSet.pop();
            
            int currentId = current.intersection.id;
            
            // Skip if we've already visited this node
            if (closedSet.count(currentId)) continue;
            closedSet.insert(currentId);
            
            // Check if we reached the destination
            if (currentId == ending.id) {
                pathFound = true;
                
                // Reconstruct path
                vector<Intersection> path;
                shared_ptr<PathNode> currentNode = nodeMap[currentId];
                
                // Backtrack through parents to build path
                vector<shared_ptr<PathNode>> pathNodes;
                while (currentNode) {
                    pathNodes.push_back(currentNode);
                    currentNode = currentNode->parent;
                }
                
                reverse(pathNodes.begin(), pathNodes.end());
                
                // Build path
                for (auto& node : pathNodes) {
                    path.push_back(node->intersection);
                }
                
                // Store the path
                paths[vehicleId] = path;
                //cout << "Vehicle " << vehicleId << ": Path found with " << path.size() << " nodes" << endl;
                break;
            }
            
            // Explore neighbors
            for (int neighId : current.intersection.connectingIntersectionIDs) {
                // Skip if already in closed set
                if (closedSet.count(neighId)) continue;
                
                // Calculate new g score
                float tentativeG = gScore[currentId];
                
                // Add traversal time for this road segment
                Road& road = graph[currentId][neighId];
                tentativeG += calculateTraversalTime(vehicles[vehicleId], road);
                
                // If this is a better path
                if (!gScore.count(neighId) || tentativeG < gScore[neighId]) {
                    // Update g score
                    gScore[neighId] = tentativeG;
                    
                    // Create/update neighbor node
                    auto neighNode = make_shared<PathNode>(intersections[neighId]);
                    neighNode->parent = nodeMap[currentId];
                    neighNode->g = tentativeG;
                    neighNode->h = heuristic(intersections[neighId], ending);
                    neighNode->f = neighNode->g + neighNode->h;
                    
                    nodeMap[neighId] = neighNode;
                    openSet.push(*neighNode);
                }
            }
        }
        
        if (!pathFound) {
            cout << "Vehicle " << vehicleId << ": No path found!" << endl;
        }
    }
    
    // Stop timing and calculate duration
    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    
    // Output results
    cout << "Found paths for " << paths.size() << "/" << vehicles.size() << " vehicles" << endl;
    cout << "Total execution time: " << duration.count() << " milliseconds" << endl;
    cout << "---" << endl;
    for (auto& [id, path] : paths) {
        cout << id << " ";
        for (auto& intersection : path) {
            cout << intersection.id << " ";
        }
        cout << endl;
    }
}
 
int main() {
    // Start total timing - including data loading
    auto totalStartTime = chrono::high_resolution_clock::now();
    
    readData();
    BasicAStar();
  
    
    return 0;
}
