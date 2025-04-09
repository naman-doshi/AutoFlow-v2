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

// Enhanced heuristic that takes partitions and traffic into account
float heuristic(const Intersection& a, const Intersection& b) {
    // Euclidean distance
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) / avgSpeed;

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

// Add work-stealing implementation before AutoFlow function
class WorkStealingQueue {
private:
    queue<pair<int, Vehicle>> tasks;
    mutex queueMutex;
    condition_variable cv;
    atomic<bool> done{false};
    atomic<int> activeThreads{0};

public:
    // Add a batch of tasks to the queue
    void addTasks(const vector<pair<int, Vehicle>>& newTasks) {
        lock_guard<mutex> lock(queueMutex);
        for (const auto& task : newTasks) {
            tasks.push(task);
        }
        cv.notify_all();
    }

    // Try to get a task from the queue
    optional<pair<int, Vehicle>> getTask() {
        unique_lock<mutex> lock(queueMutex);
        if (!tasks.empty()) {
            auto task = tasks.front();
            tasks.pop();
            return task;
        }
        return nullopt;
    }

    // Wait for a task, with timeout for work stealing
    optional<pair<int, Vehicle>> waitForTask(int timeoutMs) {
        unique_lock<mutex> lock(queueMutex);
        activeThreads++;
        
        // Wait until timeout, a task becomes available, or we're done
        auto success = cv.wait_for(lock, chrono::milliseconds(timeoutMs), 
            [this]() { return !tasks.empty() || done; });
        
        activeThreads--;
        
        if (success && !tasks.empty()) {
            auto task = tasks.front();
            tasks.pop();
            return task;
        }
        return nullopt;
    }

    // Signal that no more tasks will be added
    void setDone() {
        done = true;
        cv.notify_all();
    }

    // Check if there might be more work to do
    bool isDone() {
        lock_guard<mutex> lock(queueMutex);
        return done && tasks.empty() && activeThreads == 0;
    }

    // Get number of tasks waiting
    size_t size() {
        lock_guard<mutex> lock(queueMutex);
        return tasks.size();
    }
};

// Calculate traversal time considering congestion and traffic lights
float calculateTraversalTime(const Road& road, const Intersection& current, 
  const Intersection& next, float arrivalTime, 
  LazySeg<Node, Lazy, 1 << 15>& reservationTable, 
  float autoFlowPercentage) {
    int currentTime = max((int)ceil(arrivalTime), 0);

    // Get current congestion level
    float congestion = reservationTable.query(currentTime, currentTime).sum / autoFlowPercentage;

    // Wait time due to congestion (binary search for earliest available slot)
    int l = currentTime + 1;
    int r = min(currentTime + 1000, (1 << 15) - 1); // Limit search range
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
    float densityFactor = min(1.0f, congestion / (road.capacity * autoFlowPercentage));
    float speedReduction = 1.0f - 0.75f * densityFactor;  // Speed reduces up to 75% in heavy traffic
    float baseTravelTime = road.traversalTime / speedReduction;

    // Traffic light delay
    float cycleTime = next.trafficLightDuration * next.roadCount;
    // Probability of hitting a red light increases with congestion
    float trafficLightDelay = cycleTime * 0.5f * (1.0f + 0.5f * densityFactor);

    totalIntersectionDelays += 1;
    avgIntersectionDelay = (avgIntersectionDelay * (totalIntersectionDelays - 1) + trafficLightDelay) / totalIntersectionDelays;

    return waitTime + baseTravelTime + trafficLightDelay;
}

// Add a struct to hold the reservation updates
struct ReservationUpdate {
    int roadId;
    int startTime;
    int endTime;
    int value;
};

// Update AutoFlow to include reservation table
void AutoFlow() {
    // Congestion control parameter
    float autoFlowPercentage = 0.9f;
    
    // Initialize pathfinding results storage
    unordered_map<int, vector<Intersection>> paths;
    
    // Thread pool configuration
    const int CPU_THREADS = min(16, (int)thread::hardware_concurrency());
    cout << "Starting pathfinding with " << CPU_THREADS << " threads using graph partitioning..." << endl;
    
    // Create graph partitions for better locality
    cout << "Partitioning graph into regions..." << endl;
    InertialFlowPartitioner partitioner(intersections, roads);
    
    // Calculate partition count based on graph size and thread count
    int partitionDepth = 2;  // Start with 2^2 = 4 partitions
    if (intersections.size() > 1000) partitionDepth = 3;  // Use 2^3 = 8 partitions for larger graphs
    if (intersections.size() > 5000) partitionDepth = 4;  // Use 2^4 = 16 partitions for very large graphs
    
    // Ensure we don't create too many partitions for small graphs
    int minPartitionSize = max(50, (int)(intersections.size() / (4 * CPU_THREADS)));
    
    // Perform recursive partitioning
    auto partitions = partitioner.recursivePartition(partitionDepth, minPartitionSize);
    
    cout << "Created " << partitions.size() << " graph partitions" << endl;
    
    // Assign partition IDs to intersections
    InertialFlowPartitioner::assignPartitionIDs(intersections, partitions);
    
    // Build partition connectivity graph
    vector<vector<int>> partitionAdjacency;
    vector<int> partitionEdgeCounts;
    buildPartitionGraph(roads, partitions, partitionAdjacency, partitionEdgeCounts);
    
    // Initialize the reservation tables for each road
    // Create a segment tree for reservations
    LazySeg<Node, Lazy, 1 << 15> tree;
    tree.init({0, 0}, {0, true});
    for (int i = 0; i < 1 << 15; i++) {
        tree[i] = {0, 0};
    }
    tree.build();

    // Create reservation table for each road
    unordered_map<int, LazySeg<Node, Lazy, 1 << 15>> reservationTable;
    for (auto& [id, road] : roads) {
        reservationTable[id] = tree;
    }

    // Group vehicles by starting partition
    vector<vector<pair<int, Vehicle>>> vehiclesByPartition(partitions.size());
    
    for (const auto& [vehicleId, vehicle] : vehicles) {
        // Determine starting intersection
        Road startingRoad = roads[vehicle.startingRoadId];
        int startIntersectionId;
        if (vehicle.starting[1] == 1) {
            startIntersectionId = startingRoad.int2Id;
        } else {
            startIntersectionId = startingRoad.int1Id;
        }
        
        // Find which partition this intersection belongs to
        int partitionId = intersections[startIntersectionId].partition;
        vehiclesByPartition[partitionId].push_back({vehicleId, vehicle});
    }
    
    // // Sort vehicles within each partition by priority score (passengers * emission rate)
    // for (auto& partitionVehicles : vehiclesByPartition) {
    //     sort(partitionVehicles.begin(), partitionVehicles.end(), 
    //         [](const pair<int, Vehicle>& a, const pair<int, Vehicle>& b) {
    //             return a.second.getPriorityScore() > b.second.getPriorityScore(); // Higher priority first
    //         });
    // }
    
    // Print partition statistics with priority info
    cout << "Vehicle distribution across partitions:" << endl;
    for (size_t i = 0; i < vehiclesByPartition.size(); i++) {
        cout << "  Partition " << i << ": " << vehiclesByPartition[i].size() << " vehicles";
        
        // Print priority range if there are vehicles
        if (!vehiclesByPartition[i].empty()) {
            float minPriority = vehiclesByPartition[i].back().second.getPriorityScore();
            float maxPriority = vehiclesByPartition[i].front().second.getPriorityScore();
            cout << " (priority range: " << minPriority << " - " << maxPriority << ")";
        }
        cout << endl;
    }
    
    // Work stealing queue
    WorkStealingQueue workQueue;
    
    // Add tasks ordered by partition and priority to optimize locality and processing order
    for (const auto& partitionVehicles : vehiclesByPartition) {
        workQueue.addTasks(partitionVehicles);
    }
    
    // Function to process a single vehicle's path
    auto processVehiclePath = [&](int vehicleId, const Vehicle& vehicle, 
                                  vector<ReservationUpdate>& localUpdates) {
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
        
        // A* implementation with partition awareness and congestion
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
        
        // Track which partitions we've encountered for logging
        unordered_set<int> traversedPartitions;
        traversedPartitions.insert(starting.partition);
        
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
                
                // Also update the reservation table along the path
                float arrivalTime = 0.0f;
                vector<shared_ptr<PathNode>> reversePath;
                
                while (current != nullptr) {
                    reversePath.push_back(current);
                    current = current->parent;
                }
                
                // Reconstruct path in correct order
                for (auto it = reversePath.rbegin(); it != reversePath.rend(); ++it) {
                    path.push_back((*it)->intersection);
                }
                
                // Update the reservation table along the path
                for (size_t i = 0; i < path.size() - 1; ++i) {
                    int fromId = path[i].id;
                    int toId = path[i + 1].id;
                    
                    Road& road = graph[fromId][toId];
                    float traversalTime = calculateTraversalTime(
                        road, path[i], path[i + 1], arrivalTime, 
                        reservationTable[road.id], autoFlowPercentage
                    );
                    
                    // Add to local updates instead of locking
                    localUpdates.push_back({
                        road.id,
                        static_cast<int>(ceil(arrivalTime)),
                        static_cast<int>(ceil(arrivalTime + traversalTime)),
                        1
                    });
                    
                    arrivalTime += traversalTime;
                }
                
                // Store the path (with mutex for thread safety)
                {
                    lock_guard<mutex> lock(pathsMutex);
                    paths[vehicleId] = path;
                }
                
                break;
            }
            
            // Track which partition we're in for logging
            traversedPartitions.insert(intersections[currentId].partition);
            
            // Expand neighboring nodes
            for (const auto& neighId : currentNode.intersection.connectingIntersectionIDs) {
                // Skip if already closed
                if (closedNodes.count(neighId)) continue;
                
                const Intersection& neigh = intersections[neighId];
                
                // Calculate g score for this path
                float g = currentNode.g;
                
                // Use traversal time based on congestion and reservation table
                Road& road = graph[currentId][neighId];
                g += calculateTraversalTime(
                    road, currentNode.intersection, neigh, currentNode.g,
                    reservationTable[road.id], autoFlowPercentage
                );
                
                // Add a small penalty for crossing partition boundaries
                if (neigh.partition != currentNode.intersection.partition) {
                    g += 0.1; // Small penalty for crossing partitions
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
                    openNodes.update(*neighPtr);
                }
            }
        }
        
        auto endTime = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
        
        {
            lock_guard<mutex> lock(coutMutex);
            // if (pathFound) {
            //     cout << "Vehicle " << vehicleId << ": Path found in " 
            //          << duration.count() << "ms (" << expansions << " expansions, "
            //          << traversedPartitions.size() << " partitions)" << endl;
            // } else {
            //     cout << "Vehicle " << vehicleId << ": Path NOT found after " 
            //          << duration.count() << "ms (" << expansions << " expansions)" << endl;
            // }
        }
    };
    
    // Process vehicles using work stealing with batched reservation updates
    atomic<int> completedVehicles(0);
    const size_t totalVehicles = vehicleVector.size();
    
    // Function to apply batched updates to the global reservation table
    auto applyReservationUpdates = [&](const vector<ReservationUpdate>& updates) {
        lock_guard<mutex> lock(reservationTableMutex);
        for (const auto& update : updates) {
            reservationTable[update.roadId].upd(update.startTime, update.endTime, {update.value, true});
        }
    };
    
    // Define the batch size for reservation updates
    const int BATCH_UPDATE_SIZE = 50; // Tune this value
    
    // Create processing threads with work stealing and batched reservation updates
    vector<thread> threads;
    for (int t = 0; t < CPU_THREADS; t++) {
        threads.emplace_back([&, t]() {
            bool idle = false;
            vector<ReservationUpdate> localUpdates;
            localUpdates.reserve(BATCH_UPDATE_SIZE * 2); // Pre-allocate
            
            while (true) {
                // Try to get a task
                optional<pair<int, Vehicle>> task;
                
                if (idle) {
                    // If we were idle before, actively try to steal work
                    task = workQueue.getTask();
                    
                    if (!task && workQueue.isDone()) {
                        // Apply any remaining updates before exiting
                        if (!localUpdates.empty()) {
                            applyReservationUpdates(localUpdates);
                            localUpdates.clear();
                        }
                        // No more work to do, exit the thread
                        break;
                    } else if (!task) {
                        // No tasks available right now, wait a bit before trying again
                        // This is a good time to apply any accumulated updates
                        if (!localUpdates.empty()) {
                            applyReservationUpdates(localUpdates);
                            localUpdates.clear();
                        }
                        this_thread::sleep_for(chrono::milliseconds(1));
                        continue;
                    }
                    
                    idle = false;
                } else {
                    // Wait for a task with timeout
                    task = workQueue.waitForTask(1); // 10ms timeout
                    
                    if (!task) {
                        if (workQueue.isDone()) {
                            // Apply any remaining updates before exiting
                            if (!localUpdates.empty()) {
                                applyReservationUpdates(localUpdates);
                                localUpdates.clear();
                            }
                            // No more work to do, exit the thread
                            break;
                        }
                        // Couldn't get a task, go into idle/stealing mode
                        // This is a good time to apply any accumulated updates
                        if (!localUpdates.empty()) {
                            applyReservationUpdates(localUpdates);
                            localUpdates.clear();
                        }
                        idle = true;
                        continue;
                    }
                }
                
                // Process this vehicle
                int vehicleId = task->first;
                const Vehicle& vehicle = task->second;
                processVehiclePath(vehicleId, vehicle, localUpdates);
                
                // Apply updates if we've accumulated enough
                if (localUpdates.size() >= BATCH_UPDATE_SIZE) {
                    applyReservationUpdates(localUpdates);
                    localUpdates.clear();
                }
                
                // Update progress
                int completed = ++completedVehicles;
                if (completed % 100 == 0 || completed == totalVehicles) {
                    lock_guard<mutex> lock(coutMutex);
                    cout << "Completed " << completed << "/" << totalVehicles 
                         << " vehicles (" << (completed * 100 / totalVehicles) << "%), "
                         << workQueue.size() << " tasks remaining" << endl;
                }
            }
        });
    }
    
    // Signal that no more tasks will be added
    workQueue.setDone();
    
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
