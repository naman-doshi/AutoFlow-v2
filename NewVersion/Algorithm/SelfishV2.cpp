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
// #pragma GCC target ("avx2")
// #pragma GCC optimization ("O3")
// #pragma GCC optimization ("unroll-loops")

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

// Replace the custom heuristic with a simple Euclidean distance function
float heuristic(const Intersection& a, const Intersection& b) {
    // Simple Euclidean distance
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
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

// Add this custom binary heap implementation before the AutoFlow function
template<typename T, typename Compare = less<T>>
class BinaryHeap {
private:
    vector<T> heap;
    unordered_map<int, int> idToPosition; // Maps node ID to position in heap array
    Compare compare;
    
    // Helper functions for heap operations
    void siftUp(int idx) {
        int parent;
        while (idx > 0) {
            parent = (idx - 1) / 2;
            if (compare(heap[parent], heap[idx])) {
                break; // Heap property satisfied
            }
            swap(heap[parent], heap[idx]);
            
            // Update position mapping
            if (is_same<T, PathNode>::value) {
                idToPosition[heap[parent].intersection.id] = parent;
                idToPosition[heap[idx].intersection.id] = idx;
            }
            
            idx = parent;
        }
    }
    
    void siftDown(int idx) {
        int size = heap.size();
        int minChild;
        
        while (2 * idx + 1 < size) {
            int leftChild = 2 * idx + 1;
            int rightChild = leftChild + 1;
            
            minChild = leftChild;
            
            if (rightChild < size && compare(heap[rightChild], heap[leftChild])) {
                minChild = rightChild;
            }
            
            if (compare(heap[idx], heap[minChild])) {
                break; // Heap property satisfied
            }
            
            swap(heap[idx], heap[minChild]);
            
            // Update position mapping
            if (is_same<T, PathNode>::value) {
                idToPosition[heap[idx].intersection.id] = idx;
                idToPosition[heap[minChild].intersection.id] = minChild;
            }
            
            idx = minChild;
        }
    }
    
public:
    BinaryHeap() : compare(Compare()) {}
    
    bool empty() const {
        return heap.empty();
    }
    
    size_t size() const {
        return heap.size();
    }
    
    void push(const T& item) {
        heap.push_back(item);
        
        // Update position mapping if we're storing PathNode objects
        if (is_same<T, PathNode>::value) {
            idToPosition[item.intersection.id] = heap.size() - 1;
        }
        
        siftUp(heap.size() - 1);
    }
    
    T top() const {
        if (empty()) {
            throw runtime_error("Heap is empty");
        }
        return heap[0];
    }
    
    void pop() {
        if (empty()) {
            throw runtime_error("Heap is empty");
        }
        
        // Update position mapping
        if (is_same<T, PathNode>::value) {
            idToPosition.erase(heap[0].intersection.id);
        }
        
        heap[0] = heap.back();
        heap.pop_back();
        
        if (!empty()) {
            // Update position mapping for the relocated item
            if (is_same<T, PathNode>::value) {
                idToPosition[heap[0].intersection.id] = 0;
            }
            siftDown(0);
        }
    }
    
    bool contains(int id) const {
        return idToPosition.find(id) != idToPosition.end();
    }
    
    void update(const T& item) {
        if (!is_same<T, PathNode>::value) {
            throw runtime_error("Update only supported for PathNode");
        }
        
        auto it = idToPosition.find(item.intersection.id);
        if (it == idToPosition.end()) {
            // Item not in heap, just push it
            push(item);
            return;
        }
        
        int idx = it->second;
        
        // Check if the new value is smaller or larger than current
        if (compare(item, heap[idx])) {
            // New value is higher priority (smaller f-value for min-heap)
            heap[idx] = item;
            siftUp(idx);
        } else {
            // New value is lower priority (larger f-value for min-heap)
            heap[idx] = item;
            siftDown(idx);
        }
    }
};

// Custom comparison function for PathNode (inverse of original since we want a min-heap)
struct PathNodeCompare {
    bool operator()(const PathNode& a, const PathNode& b) const {
        return a.f < b.f;  // Min heap based on f-value
    }
};

// Update AutoFlow to remove multithreading along with partitioning, custom heuristic, and reservation table
void AutoFlow() {
    // Initialize pathfinding results storage
    unordered_map<int, vector<Intersection>> paths;
    
    cout << "Starting pathfinding in single-threaded mode..." << endl;
    
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
        
        // Standard A* implementation
        PathNode startNode(starting);
        PathNode endNode(ending);
        
        BinaryHeap<PathNode, PathNodeCompare> openNodes;
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
            if (closedNodes.count(currentId)) continue;
                
            // Add to closed set
            closedNodes.insert(currentId);
            
            // Check if we reached the destination
            if (currentId == ending.id) {
                pathFound = true;
                
                // Reconstruct the path
                vector<Intersection> path;
                shared_ptr<PathNode> current = nodeMap[currentId];
                
                while (current != nullptr) {
                    path.push_back(current->intersection);
                    current = current->parent;
                }
                
                reverse(path.begin(), path.end());
                
                // Store the path (no mutex needed in single-threaded mode)
                paths[vehicleId] = path;
                
                break;
            }
            
            // Expand neighboring nodes
            for (const auto& neighId : currentNode.intersection.connectingIntersectionIDs) {
                // Skip if already closed
                if (closedNodes.count(neighId)) continue;
                
                const Intersection& neigh = intersections[neighId];
                
                // Calculate g score for this path
                float g = currentNode.g;
                
                // Use simple traversal time based on road length and speed limit
                Road& road = graph[currentId][neighId];
                g += road.length / road.speedLimit;
                
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
                    openNodes.update(*neighPtr);
                }
            }
        }
        
        auto endTime = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
        
        if (pathFound) {
            cout << "Vehicle " << vehicleId << ": Path found in " 
                 << duration.count() << "ms (" << expansions << " expansions)" << endl;
        } else {
            cout << "Vehicle " << vehicleId << ": Path NOT found after " 
                 << duration.count() << "ms (" << expansions << " expansions)" << endl;
        }
    };
    
    // Process vehicles sequentially
    size_t totalVehicles = vehicleVector.size();
    int completedVehicles = 0;
    
    for (const auto& [vehicleId, vehicle] : vehicleVector) {
        // Process this vehicle
        processVehiclePath(vehicleId, vehicle);
        
        // Update progress
        completedVehicles++;
        if (completedVehicles % 100 == 0 || completedVehicles == totalVehicles) {
            cout << "Completed " << completedVehicles << "/" << totalVehicles 
                 << " vehicles (" << (completedVehicles * 100 / totalVehicles) << "%)" << endl;
        }
    }
    
    // Output results
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
 
int main() {
  
    readData();
    AutoFlow();


  return 0;
}
