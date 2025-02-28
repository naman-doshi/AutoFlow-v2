#include <bits/stdc++.h>
using namespace std;
#include "Landscape.cpp"
#include "Partition.cpp"
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <future>
#include <chrono>
#pragma GCC target ("avx2")
#pragma GCC optimization ("O3")
#pragma GCC optimization ("unroll-loops")

#define rep(i, a, b) for(int i = a; i < (b); ++i)

mutex reservationTableMutex;
mutex pathsMutex;
mutex coutMutex;

// todo:
// 2. graph partitioning
// 4. make traffic light timings more accurate using more info

unordered_map<int, Intersection> intersections;
unordered_map<int, Road> roads;
unordered_map<int, Vehicle> vehicles;
vector<pair<int, Vehicle>> vehicleVector;
unordered_map<int, unordered_map<int, Road>> graph;

struct Lazy {
    int v;
    bool inc;
    void operator+=(const Lazy &b) {
        if (b.inc) v += b.v;
        else v = b.v, inc = false;
    }
};
 
struct Node {
    int mn, sum;
    Node operator+(const Node &b) {
        return {min(mn, b.mn), sum + b.sum};
    }
    void upd(const Lazy &u, int l, int r) {
        if (u.inc) mn += u.v, sum += u.v * (r - l + 1);
        else mn = u.v, sum = u.v * (r - l + 1);
    }
};
 
template<class T, class U, int SZ> struct LazySeg {
    T NID;
    U UID;
    vector<T> seg;
    vector<U> lazy;
    void init(T _NID, U _UID) {
        NID = _NID;
        UID = _UID;
        seg.resize(2 * SZ, NID);
        lazy.resize(2 * SZ, UID);
    }
    void pull(int i) {
        seg[i] = seg[2 * i] + seg[2 * i + 1];
    }
    void push(int i, int l, int r) {
        seg[i].upd(lazy[i], l, r);
        if (l != r)  rep(j, 0, 2) lazy[2 * i + j] += lazy[i];
        lazy[i] = UID;
    }
    void build() {
        for (int i = SZ - 1; i > 0; i--) pull(i);
    }
    void upd(int lo, int hi, U val, int i = 1, int l = 0, int r = SZ - 1) {
        push(i, l, r);
        if (r < lo || l > hi) return;
        if (lo <= l && r <= hi) {
            lazy[i] += val;
            push(i, l, r);
            return;
        }
        int m = (l + r) / 2;
        upd(lo, hi, val, 2 * i, l, m);
        upd(lo, hi, val, 2 * i + 1, m + 1, r);
        pull(i);
    }
    T query(int lo = 0, int hi = SZ - 1, int i = 1, int l = 0, int r = SZ - 1) {
        push(i, l, r);
        if (r < lo || l > hi) return NID;
        if (lo <= l && r <= hi) return seg[i];
        int m = (l + r) / 2;
        return query(lo, hi, 2 * i, l, m) + query(lo, hi, 2 * i + 1, m + 1, r);
    }
    T& operator[](int i) {
        return seg[i + SZ];
    }
};

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
        return f > other.f;
    }

    size_t operator()(const PathNode& node) const {
        return hash<int>()(node.intersection.id);
    }
};

float avgIntersectionDelay = 2.0f;
float totalIntersectionDelays = 0;
float avgSpeed = 0;
float avgSegmentLength = 0;

// Enhanced heuristic that takes partitions into account
float heuristic(const Intersection& a, const Intersection& b) {
    // Euclidean distance
    float distance = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    
    // Add partition-crossing penalty if intersections are in different partitions
    float partitionPenalty = 0.0f;
    if (a.partition != b.partition && a.partition >= 0 && b.partition >= 0) {
        partitionPenalty = avgIntersectionDelay * 0.5f; // Add delay when crossing partitions
    }
    
    // Estimate number of intersections in path (rough approximation)
    float estimatedIntersections = distance / avgSegmentLength;
    
    return distance / avgSpeed + (avgIntersectionDelay/totalIntersectionDelays) * estimatedIntersections + partitionPenalty;
}

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
        avgSegmentLength += road.length;
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
    avgSegmentLength /= numRoads;

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
        cin >> vehicle.id >> startingRoadId >> endingRoadId >> s1 >> s2 >> s3 >> e1 >> e2 >> e3;
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
}

float calculateTraversalTime(const Road& road, const Intersection& current, 
  const Intersection& next, float arrivalTime, 
  LazySeg<Node, Lazy, 1 << 15>& reservationTable, 
  float autoFlowPercentage) {
  int currentTime = max((int)ceil(arrivalTime), 0);

    // Get current congestion level
    float congestion = reservationTable.query(currentTime, currentTime).sum / autoFlowPercentage;

    // Wait time due to congestion (binary search for earliest available slot)
    int l = currentTime + 1;
    int r = 1 << 15;
    while (l < r) {
        int m = (l + r) / 2;
        if (reservationTable.query(currentTime, m).mn < autoFlowPercentage * road.capacity) {
            r = m;
        } else {
            l = m + 1;
        }
    }
    float waitTime = r - currentTime;

    // Base travel time (adjusted for congestion)
    // Use a more sophisticated model that considers actual vehicle density 
    float densityFactor = min(1.0f, congestion / (road.capacity * autoFlowPercentage));
    float speedReduction = 1.0f - 0.75f * densityFactor;  // Speed reduces up to 75% in heavy traffic
    float baseTravelTime = road.length / (road.speedLimit * speedReduction);

    // Traffic light delay
    float cycleTime = next.trafficLightDuration * next.roadCount;
    // Probability of hitting a red light increases with congestion
    float trafficLightDelay = cycleTime * 0.5f * (1.0f + 0.5f * densityFactor);

    avgIntersectionDelay += trafficLightDelay;
    totalIntersectionDelays += 1;

    return waitTime + baseTravelTime + trafficLightDelay;
}

// Run graph partitioning and set up data structures before pathfinding
pair<vector<unordered_set<int>>, vector<vector<int>>> partitionGraph() {
    // dynamic partition depth
    int partitioningDepth = log2(intersections.size() / 200);
    partitioningDepth = max(1, partitioningDepth); // Ensure at least one level
    int minPartitionSize = 200;
    
    cout << "Starting graph partitioning with depth=" << partitioningDepth 
         << " and minSize=" << minPartitionSize << "..." << endl;
    
    InertialFlowPartitioner partitioner(intersections, roads);
    auto partitions = partitioner.recursivePartition(partitioningDepth, minPartitionSize);
    
    // Assign partition IDs to all intersections
    partitioner.assignPartitionIDs(intersections, partitions);
    
    vector<vector<int>> partitionAdjacencyList;
    vector<int> crossPartitionEdgeCounts;
    buildPartitionGraph(roads, partitions, partitionAdjacencyList, crossPartitionEdgeCounts);
    
    cout << "Created " << partitions.size() << " partitions" << endl;
    
    // Print partition statistics
    int intersectionsWithPartitions = 0;
    for (const auto& [id, intersection] : intersections) {
        if (intersection.partition >= 0) {
            intersectionsWithPartitions++;
        }
    }
    cout << "Total intersections with assigned partitions: " 
         << intersectionsWithPartitions << "/" << intersections.size() << endl;
    
    return {partitions, partitionAdjacencyList};
}

// Update AutoFlow to use the enhanced pathfinder
void AutoFlow() {
    float autoFlowPercentage = 0.9999f;
    
    // Run partitioning before pathfinding
    cout << "Starting graph partitioning..." << endl;
    auto [partitions, partitionAdjList] = partitionGraph();
    cout << "Partitioning completed" << endl;
    
    // Initialize the segment tree for reservations
    LazySeg<Node, Lazy, 1 << 15> tree;
    tree.init({0, 0}, {0, true});
    for (int i = 0; i < 1 << 15; i++) {
        tree[i] = {0, 0};
    }
    tree.build();

    unordered_map<int, LazySeg<Node, Lazy, 1 << 15>> reservationTable;
    for (auto& [id, road] : roads) {
        reservationTable[id] = tree;
    }

    unordered_map<int, float> defaultG;
    for (auto& [id, vi] : intersections) {
        defaultG[vi.id] = 1e9;
    }
    
    // Initialize vehicles in the reservation system
    for (auto& [id, vehicle] : vehicles) {
        Road startingRoad = roads[vehicle.startingRoadId];
        int traversalTime = ceil(startingRoad.traversalTime * (1 - vehicle.position));
        reservationTable[startingRoad.id].upd(0, traversalTime, {1, true});
    }
    
    // Initialize pathfinding results storage
    unordered_map<int, vector<Intersection>> paths;

    
    // Thread pool configuration
    const int CPU_THREADS = min(16, (int)thread::hardware_concurrency());
    cout << "Starting enhanced pathfinding with " << CPU_THREADS << " threads..." << endl;
    
    // Function to process a single vehicle's path
    auto processVehiclePath = [&](int vehicleId, const Vehicle& vehicle) {
        auto startTime = chrono::high_resolution_clock::now();
        
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

        // A*
        PathNode startNode(starting);
        PathNode endNode(ending);
        
        priority_queue<PathNode> openNodes;
        unordered_map<int, float> gScore;
        unordered_set<int> closedNodes;
        unordered_map<int, shared_ptr<PathNode>> nodeMap;
        
        // Initialize search
        startNode.g = 0;
        startNode.h = heuristic(starting, ending);
        startNode.f = startNode.g + startNode.h;
        
        auto startPtr = make_shared<PathNode>(startNode);
        openNodes.push(*startPtr);
        gScore[starting.id] = 0;
        nodeMap[starting.id] = startPtr;
        
        bool pathFound = false;
        int expansions = 0;
        const int MAX_EXPANSIONS = 10000;
        
        while (!openNodes.empty() && expansions < MAX_EXPANSIONS) {
            expansions++;
            
            // Get the node with the lowest f score
            PathNode currentNode = openNodes.top();
            openNodes.pop();
            
            int currentId = currentNode.intersection.id;
            
            // If we've already processed this node with a better path, skip it
            if (closedNodes.find(currentId) != closedNodes.end())
                continue;
                
            // Add to closed set
            closedNodes.insert(currentId);
            
            // Check if we reached the destination
            if (currentId == ending.id) {
                pathFound = true;
                
                // Reconstruct the path
                vector<Intersection> path;
                shared_ptr<PathNode> current = nodeMap[currentId];
                
                // Update reservation table along the path
                shared_ptr<PathNode> prev = nullptr;
                float arrivalTime = 0;
                
                while (current != nullptr) {
                    path.push_back(current->intersection);
                    
                    // Update reservation table for each road segment
                    if (prev != nullptr) {
                        int currentId = current->intersection.id;
                        int prevId = prev->intersection.id;
                        
                        // Find the road between the two intersections
                        if (graph.count(currentId) && graph[currentId].count(prevId)) {
                            Road& road = graph[currentId][prevId];
                            float traversalTime = calculateTraversalTime(
                                road, current->intersection, prev->intersection, 
                                arrivalTime, reservationTable[road.id], autoFlowPercentage
                            );
                            
                            // Update reservation table (with mutex for thread safety)
                            {
                                lock_guard<mutex> lock(reservationTableMutex);
                                reservationTable[road.id].upd(
                                    ceil(arrivalTime), 
                                    ceil(arrivalTime + traversalTime), 
                                    {1, true}
                                );
                            }
                            
                            arrivalTime += traversalTime;
                        }
                    }
                    
                    prev = current;
                    current = current->parent;
                }
                
                reverse(path.begin(), path.end());
                
                // Store the path (with mutex for thread safety)
                {
                    lock_guard<mutex> lock(pathsMutex);
                    paths[vehicleId] = path;
                }
                
                break;
            }
            
            // Expand neighboring nodes
            for (const auto& neighId : currentNode.intersection.connectingIntersectionIDs) {
                // Skip if already closed
                if (closedNodes.find(neighId) != closedNodes.end())
                    continue;
                
                const Intersection& neigh = intersections[neighId];
                
                // Calculate g score for this path
                float g = currentNode.g;
                
                // Add cost for this edge/road
                if (graph.count(currentId) && graph[currentId].count(neighId)) {
                    Road& road = graph[currentId][neighId];
                    g += calculateTraversalTime(
                        road,
                        currentNode.intersection, neigh,
                        currentNode.g, 
                        reservationTable[road.id],
                        autoFlowPercentage
                    );
                } else {
                    // No direct road found, use fallback cost
                    g += sqrt(pow(currentNode.intersection.x - neigh.x, 2) + 
                             pow(currentNode.intersection.y - neigh.y, 2)) / avgSpeed;
                }
                
                // If we found a better path to this node
                if (!gScore.count(neighId) || g < gScore[neighId]) {
                    gScore[neighId] = g;
                    
                    float h = heuristic(neigh, ending);
                    float f = g + h;
                    
                    auto neighPtr = make_shared<PathNode>(neigh);
                    neighPtr->parent = nodeMap[currentId];
                    neighPtr->g = g;
                    neighPtr->h = h;
                    neighPtr->f = f;
                    
                    nodeMap[neighId] = neighPtr;
                    openNodes.push(*neighPtr);
                }
            }
        }
        
        auto endTime = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
        
        {
            lock_guard<mutex> lock(coutMutex);
            if (pathFound) {
                cout << "Vehicle " << vehicleId << ": Path found in " 
                     << duration.count() << "ms (" << expansions << " expansions)" << endl;
            } else {
                cout << "Vehicle " << vehicleId << ": Path NOT found after " 
                     << duration.count() << "ms (" << expansions << " expansions)" << endl;
            }
        }
    };
    
    // Process vehicles in parallel batches
    atomic<int> completedVehicles(0);
    const size_t totalVehicles = vehicleVector.size();
    
    // Create processing threads
    vector<thread> threads;
    for (int t = 0; t < CPU_THREADS; t++) {
        threads.emplace_back([&, t]() {
            for (size_t idx = t; idx < totalVehicles; idx += CPU_THREADS) {
                int vehicleId = vehicleVector[idx].first;
                const Vehicle& vehicle = vehicleVector[idx].second;
                
                // Process this vehicle
                processVehiclePath(vehicleId, vehicle);
                
                // Update progress
                int completed = ++completedVehicles;
                if (completed % 100 == 0 || completed == totalVehicles) {
                    lock_guard<mutex> lock(coutMutex);
                    cout << "Completed " << completed << "/" << totalVehicles 
                         << " vehicles (" << (completed * 100 / totalVehicles) << "%)" << endl;
                }
            }
        });
    }
    
    // Wait for all threads to finish
    for (auto& t : threads) {
        t.join();
    }
    
    // Output results
    {
        lock_guard<mutex> lock(coutMutex);
        cout << "Found paths for " << paths.size() << "/" << vehicleVector.size() << " vehicles" << endl;
        cout << "---" << endl;
        for (auto& [id, path] : paths) {
            cout << id << " ";
            for (auto& intersection : path) {
                cout << intersection.id << " ";
            }
            cout << endl;
        }
    }
}
 
int main() {
  
    readData();
    AutoFlow();


  return 0;
}
