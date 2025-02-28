#include <algorithm>
#include <random>
#include <cmath>
#include <queue>
#include <iostream>
#include <set>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <limits>

using namespace std;


const double M_PI = 3.14159265358979323846;

// Class declaration for InertialFlowPartitioner
class InertialFlowPartitioner {
private:
    const unordered_map<int, Intersection>& intersections;
    const unordered_map<int, Road>& roads;
    
    // Creates a comparator for spatial ordering along a given angle
    function<bool(const Intersection&, const Intersection&)> 
    createSpatialComparator(double angle);
    
    // Finds source and sink nodes for the flow network
    pair<vector<int>, vector<int>> 
    findSourcesAndSinks(double angle, double balanceParam = 0.2);
    
    // Creates a subgraph from a set of intersection IDs
    pair<unordered_map<int, Intersection>, unordered_map<int, Road>>
    createSubgraph(const unordered_set<int>& intersectionIds) const;
    
public:
    // Constructor
    InertialFlowPartitioner(
        const unordered_map<int, Intersection>& intersections,
        const unordered_map<int, Road>& roads
    );
    
    // Partition the graph using inertial flow
    pair<unordered_set<int>, unordered_set<int>> 
    partition(double balanceParam = 0.2, int numAngles = 8);
    
    // Recursively partition the graph
    vector<unordered_set<int>> 
    recursivePartition(int depth = 3, int minSize = 50);
    
    // Assign partition IDs to intersections
    static void assignPartitionIDs(
        unordered_map<int, Intersection>& intersections,
        const vector<unordered_set<int>>& partitions
    );
};

// Function to build partition adjacency graph
void buildPartitionGraph(
    const unordered_map<int, Road>& roads,
    const vector<unordered_set<int>>& partitions,
    vector<vector<int>>& adjacencyList,
    vector<int>& edgeCounts
);

// Flip a binary comparator
template<typename Compare>
class Flip {
private:
    Compare comp;
    
public:
    Flip(Compare c) : comp(c) {}
    
    template<typename T>
    bool operator()(const T& a, const T& b) const {
        return comp(b, a);
    }
};

template<typename Compare>
Flip<Compare> flip(Compare comp) {
    return Flip<Compare>(comp);
}

// Structure to represent residual graph edges
struct Edge {
    int to;
    int capacity;
    int flow;
    int rev; // Index of reverse edge
    
    Edge(int t, int c, int f, int r) {
        to = t;
        capacity = c;
        flow = f;
        rev = r;
    }
};

// Edmonds-Karp max flow implementation with residual graph
class MaxFlowSolver {
private:
    vector<vector<Edge>> graph;
    vector<int> parent;
    vector<int> edgeIndex; // Store edge indices separately from parent vertices
    int source, sink;
    int vertices;
    
    // BFS to find augmenting path
    bool bfs() {
        fill(parent.begin(), parent.end(), -1);
        fill(edgeIndex.begin(), edgeIndex.end(), -1);
        
        queue<int> q;
        q.push(source);
        parent[source] = -2; // Mark source as visited
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            
            for (size_t i = 0; i < graph[u].size(); i++) {
                Edge& e = graph[u][i];
                if (parent[e.to] == -1 && e.capacity > e.flow) {
                    parent[e.to] = u;
                    edgeIndex[e.to] = i;  // Save edge index properly
                    if (e.to == sink) return true;
                    q.push(e.to);
                }
            }
        }
        return false;
    }
    
public:
    MaxFlowSolver(int n, int s, int t) : vertices(n), source(s), sink(t) {
        graph.resize(n);
        parent.resize(n, -1);
        edgeIndex.resize(n, -1); // Initialize edge index tracking
    }
    
    // Add an edge to the flow network
    void addEdge(int from, int to, int capacity) {
        Edge forward(to, capacity, 0, graph[to].size());
        Edge backward(from, 0, 0, graph[from].size());
        
        graph[from].push_back(forward);
        graph[to].push_back(backward);
    }
    
    // Build the flow network from a collection of intersections and roads
    void buildNetwork(const unordered_map<int, Intersection>& intersections, 
                     const unordered_map<int, Road>& roads,
                     const vector<int>& sources, 
                     const vector<int>& sinks) {
        // Create a mapping from intersection IDs to indices
        unordered_map<int, int> idToIndex;
        int index = 0;
        
        for (const auto& [id, _] : intersections) {
            idToIndex[id] = index++;
        }
        
        vertices = intersections.size() + 2; // +2 for super source and super sink
        source = intersections.size();      // Super source
        sink = intersections.size() + 1;    // Super sink
        
        graph.clear();
        graph.resize(vertices);
        parent.resize(vertices, -1);
        edgeIndex.resize(vertices, -1);
        
        // Connect super source to all sources
        for (int s : sources) {
            if (idToIndex.find(s) != idToIndex.end()) {
                addEdge(source, idToIndex[s], numeric_limits<int>::max());
            }
        }
        
        // Connect all sinks to super sink
        for (int t : sinks) {
            if (idToIndex.find(t) != idToIndex.end()) {
                addEdge(idToIndex[t], sink, numeric_limits<int>::max());
            }
        }
        
        // Add all roads as edges with capacity proportional to their lane count
        for (const auto& [id, road] : roads) {
            if (idToIndex.find(road.int1Id) != idToIndex.end() && 
                idToIndex.find(road.int2Id) != idToIndex.end()) {
                int from = idToIndex[road.int1Id];
                int to = idToIndex[road.int2Id];
                // Use lane count as capacity to favor cutting smaller roads
                addEdge(from, to, road.laneCount);
                addEdge(to, from, road.laneCount); // Assuming bidirectional roads
            }
        }
    }
    
    // Calculate max flow
    int calculateMaxFlow() {
        int maxFlow = 0;
        
        while (bfs()) {
            // Find the minimum residual capacity along the augmenting path
            int pathFlow = numeric_limits<int>::max();
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                int ei = edgeIndex[v]; // Get the edge index properly
                pathFlow = min(pathFlow, graph[u][ei].capacity - graph[u][ei].flow);
            }
            
            // Update flow along the augmenting path
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                int ei = edgeIndex[v];
                graph[u][ei].flow += pathFlow;
                graph[graph[u][ei].to][graph[u][ei].rev].flow -= pathFlow; // Update reverse edge
            }
            
            maxFlow += pathFlow;
        }
        
        return maxFlow;
    }
    
    // Find the min cut (vertices reachable from source in residual graph)
    unordered_set<int> findMinCut(const unordered_map<int, Intersection>& intersections) {
        unordered_set<int> cut;
        vector<bool> visited(vertices, false);
        queue<int> q;
        
        // Create reverse mapping from indices to intersection IDs
        unordered_map<int, int> indexToId;
        int index = 0;
        for (const auto& [id, _] : intersections) {
            indexToId[index++] = id;
        }
        
        q.push(source);
        visited[source] = true;
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            
            if (u != source && u != sink) {
                // Map index back to intersection ID using the reverse mapping
                auto it = indexToId.find(u);
                if (it != indexToId.end()) {
                    cut.insert(it->second);
                }
            }
            
            for (size_t i = 0; i < graph[u].size(); i++) {
                const Edge& e = graph[u][i];
                if (!visited[e.to] && e.capacity > e.flow) {
                    visited[e.to] = true;
                    q.push(e.to);
                }
            }
        }
        
        return cut;
    }
};

// Constructor implementation
InertialFlowPartitioner::InertialFlowPartitioner(
    const unordered_map<int, Intersection>& inters, 
    const unordered_map<int, Road>& rds
) : intersections(inters), roads(rds) {}

// Implementation of spatial comparator function
function<bool(const Intersection&, const Intersection&)> 
InertialFlowPartitioner::createSpatialComparator(double angle) {
    return [angle](const Intersection& a, const Intersection& b) {
        // Project coordinates onto a line with the given angle
        double a_proj = a.x * cos(angle) + a.y * sin(angle);
        double b_proj = b.x * cos(angle) + b.y * sin(angle);
        return a_proj < b_proj;
    };
}

// Implementation of find sources and sinks
pair<vector<int>, vector<int>> 
InertialFlowPartitioner::findSourcesAndSinks(double angle, double balanceParam) {
    vector<Intersection> intersectionList;
    for (const auto& [id, intersection] : intersections) {
        intersectionList.push_back(intersection);
    }
    
    size_t k = static_cast<size_t>(balanceParam * intersectionList.size());
    k = max(size_t(1), k); // Ensure at least one source and sink
    
    auto spatialComp = createSpatialComparator(angle);
    
    // Sort intersections by projected coordinate
    sort(intersectionList.begin(), intersectionList.end(), spatialComp);
    
    // Extract source and sink intersection IDs
    vector<int> sources, sinks;
    sources.reserve(k);
    sinks.reserve(k);
    
    for (size_t j = 0; j < k; ++j) {
        sources.push_back(intersectionList[j].id);
    }
    
    for (size_t j = intersectionList.size() - k; j < intersectionList.size(); ++j) {
        sinks.push_back(intersectionList[j].id);
    }
    
    return {sources, sinks};
}

// Implementation of create subgraph
pair<unordered_map<int, Intersection>, unordered_map<int, Road>>
InertialFlowPartitioner::createSubgraph(const unordered_set<int>& intersectionIds) const {
    unordered_map<int, Intersection> subIntersections;
    unordered_map<int, Road> subRoads;
    
    // Add intersections to the subgraph
    for (int id : intersectionIds) {
        auto it = intersections.find(id);
        if (it != intersections.end()) {
            subIntersections[id] = it->second;
            // Filter connecting IDs to only those in the subgraph
            auto& newIntersection = subIntersections[id];
            vector<int> filteredConnections;
            for (int connId : newIntersection.connectingIntersectionIDs) {
                if (intersectionIds.find(connId) != intersectionIds.end()) {
                    filteredConnections.push_back(connId);
                }
            }
            newIntersection.connectingIntersectionIDs = filteredConnections;
        }
    }
    
    // Add roads that connect intersections within the subgraph
    for (const auto& [id, road] : roads) {
        if (intersectionIds.find(road.int1Id) != intersectionIds.end() && 
            intersectionIds.find(road.int2Id) != intersectionIds.end()) {
            subRoads[id] = road;
        }
    }
    
    return {subIntersections, subRoads};
}

// Implementation of partition method
pair<unordered_set<int>, unordered_set<int>> 
InertialFlowPartitioner::partition(double balanceParam, int numAngles) {
    if (intersections.size() <= 1) {
        unordered_set<int> part;
        if (!intersections.empty()) {
            part.insert(intersections.begin()->first);
        }
        return {part, {}};
    }
    
    unordered_set<int> bestCut;
    int bestCutSize = numeric_limits<int>::max();
    
    // Try different angles for the spatial ordering
    for (int i = 0; i < numAngles; ++i) {
        double angle = M_PI * i / numAngles;
        
        auto [sources, sinks] = findSourcesAndSinks(angle, balanceParam);
        
        // Calculate max flow and find min cut
        MaxFlowSolver solver(intersections.size() + 2, intersections.size(), intersections.size() + 1);
        solver.buildNetwork(intersections, roads, sources, sinks);
        int flow = solver.calculateMaxFlow();
        unordered_set<int> cut = solver.findMinCut(intersections);
        
        // Keep track of the best cut
        if ((cut.size() < bestCutSize && !cut.empty() && cut.size() < intersections.size()) ||
            bestCut.empty()) {
            bestCut = cut;
            bestCutSize = cut.size();
        }
    }
    
    // Create the two partitions based on the best cut
    unordered_set<int> partition1 = bestCut;
    unordered_set<int> partition2;
    
    // All intersections not in partition1 go to partition2
    for (const auto& [id, _] : intersections) {
        if (partition1.find(id) == partition1.end()) {
            partition2.insert(id);
        }
    }
    
    return {partition1, partition2};
}

// Implementation of recursive partitioning
vector<unordered_set<int>> InertialFlowPartitioner::recursivePartition(int depth, int minSize) {
    if (intersections.size() <= minSize || depth <= 0) {
        // Base case: return all intersections as a single partition
        unordered_set<int> allIntersections;
        for (const auto& [id, _] : intersections) {
            allIntersections.insert(id);
        }
        return {allIntersections};
    }
    
    // Partition the graph
    auto [part1, part2] = partition();
    
    // If either partition is empty, return the other as a single partition
    if (part1.empty() || part2.empty()) {
        unordered_set<int> allIntersections;
        for (const auto& [id, _] : intersections) {
            allIntersections.insert(id);
        }
        return {allIntersections};
    }
    
    // Create subgraphs
    auto [subIntersections1, subRoads1] = createSubgraph(part1);
    auto [subIntersections2, subRoads2] = createSubgraph(part2);
    
    // Recursively partition the subgraphs
    InertialFlowPartitioner partitioner1(subIntersections1, subRoads1);
    InertialFlowPartitioner partitioner2(subIntersections2, subRoads2);
    
    auto partitions1 = partitioner1.recursivePartition(depth - 1, minSize);
    auto partitions2 = partitioner2.recursivePartition(depth - 1, minSize);
    
    // Combine the results
    vector<unordered_set<int>> result;
    result.insert(result.end(), partitions1.begin(), partitions1.end());
    result.insert(result.end(), partitions2.begin(), partitions2.end());
    
    return result;
}

// Implementation of assignPartitionIDs
void InertialFlowPartitioner::assignPartitionIDs(
    unordered_map<int, Intersection>& intersections,
    const vector<unordered_set<int>>& partitions
) {
    // Assign partition ID to each intersection
    for (size_t partitionId = 0; partitionId < partitions.size(); partitionId++) {
        for (int intersectionId : partitions[partitionId]) {
            auto it = intersections.find(intersectionId);
            if (it != intersections.end()) {
                it->second.partition = partitionId;
            }
        }
    }
}

// Implementation of buildPartitionGraph
void buildPartitionGraph(
    const unordered_map<int, Road>& roads,
    const vector<unordered_set<int>>& partitions,
    vector<vector<int>>& adjacencyList,
    vector<int>& edgeCounts
) {
    size_t numPartitions = partitions.size();
    adjacencyList.resize(numPartitions);
    edgeCounts.resize(numPartitions, 0);
    
    // Create a mapping from intersection IDs to the partition they belong to
    unordered_map<int, int> intersectionToPartition;
    for (size_t i = 0; i < numPartitions; ++i) {
        for (int intId : partitions[i]) {
            intersectionToPartition[intId] = i;
        }
    }
    
    // Find connections between partitions based on roads that cross partition boundaries
    set<pair<int, int>> partitionConnections; // To avoid duplicate edges
    
    for (const auto& [roadId, road] : roads) {
        int partition1 = -1, partition2 = -1;
        
        auto it1 = intersectionToPartition.find(road.int1Id);
        auto it2 = intersectionToPartition.find(road.int2Id);
        
        if (it1 != intersectionToPartition.end()) {
            partition1 = it1->second;
        }
        
        if (it2 != intersectionToPartition.end()) {
            partition2 = it2->second;
        }
        
        // If both intersections are in different partitions, add connection
        if (partition1 != -1 && partition2 != -1 && partition1 != partition2) {
            // Ensure smaller index is first for undirected edge
            int minPartition = min(partition1, partition2);
            int maxPartition = max(partition1, partition2);
            
            // Use a set to avoid duplicate edges
            partitionConnections.insert({minPartition, maxPartition});
            
            // Count number of roads connecting partitions
            edgeCounts[partition1]++;
            edgeCounts[partition2]++;
        }
    }
    
    // Build adjacency list from unique partition connections
    for (const auto& [part1, part2] : partitionConnections) {
        adjacencyList[part1].push_back(part2);
        adjacencyList[part2].push_back(part1); // Undirected graph
    }
}