#include <bits/stdc++.h>
using namespace std;
#include "Landscape.cpp"
#include <chrono>

#define rep(i, a, b) for(int i = a; i < (b); ++i)

// todo:
// 1. custom heuristic
// 2. graph partitioning
// 3. treat every lane as its own road (more accurate)
// 4. make traffic light timings more accurate using more info

unordered_map<int, Intersection> intersections;
unordered_map<int, VirtualIntersection> virtualIntersections;
unordered_map<int, Road> roads;
unordered_map<int, Vehicle> vehicles;
unordered_map<int, vector<int>> roadToVI;

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
    shared_ptr<VirtualIntersection> intersection;
    shared_ptr<PathNode> parent;
    float g;
    float h;
    float f;

    PathNode(shared_ptr<VirtualIntersection> intersection, shared_ptr<PathNode> parent = nullptr) {
        this->intersection = intersection;
        this->parent = parent;
        this->g = 0;
        this->h = 0;
        this->f = 0;
    }

    bool operator==(const PathNode& other) const {
        return intersection == other.intersection;
    }

    bool operator<(const PathNode& other) const {
        return f > other.f;
    }

    size_t operator()(const PathNode& node) const {
        return hash<int>()(node.intersection->id);
    }
};

float heuristic(VirtualIntersection& a, VirtualIntersection& b) {
    // euclidean
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) / 28;
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
        int numAssociatedVirtualIntersections;
        int int1Id, int2Id;
        cin >> road.id >> road.length >> road.speedLimit >> road.capacity >> int1Id >> int2Id >> road.traversalTime >> road.laneCount;
        road.int1 = make_shared<Intersection>(intersections[int1Id]);
        road.int2 = make_shared<Intersection>(intersections[int2Id]);
        cin >> numAssociatedVirtualIntersections;
        //cout << numAssociatedVirtualIntersections << endl;
        road.associatedVirtualIntersectionIds.resize(numAssociatedVirtualIntersections);
        for (int j = 0; j < numAssociatedVirtualIntersections; ++j) {
            cin >> road.associatedVirtualIntersectionIds[j];
        }
        //cout << road.associatedVirtualIntersectionIds.size() << endl;
        roads[road.id] = road;
    }

    int numVirtualIntersections;
    cin >> numVirtualIntersections;
    for (int i = 0; i < numVirtualIntersections; ++i) {
        VirtualIntersection vi;
        int numConnectingVirtualInts;
        int correspondingRealIntersectionId, roadId;
        cin >> vi.id >> correspondingRealIntersectionId >> vi.direction >> vi.x >> vi.y >> vi.roadID;
        vi.road = make_shared<Road>(roads[vi.roadID]);
        roadToVI[vi.roadID].push_back(vi.id);
        if (correspondingRealIntersectionId != -1) {
            vi.correspondingRealIntersection = make_shared<Intersection>(intersections[correspondingRealIntersectionId]);
        }
        cin >> numConnectingVirtualInts;
        vector<int> connectingVIIds(numConnectingVirtualInts);
        for (int j = 0; j < numConnectingVirtualInts; ++j) {
            cin >> connectingVIIds[j];
        }
        vi.connectingVIIds = connectingVIIds;
        virtualIntersections[vi.id] = vi;
    }

    for (auto& [id, vi] : virtualIntersections) {
        for (int connectingVIId : vi.connectingVIIds) {
            shared_ptr<VirtualIntersection> connectingVI = make_shared<VirtualIntersection>(virtualIntersections[connectingVIId]);
            vi.connectingVirtualInts.push_back(connectingVI);
        }
    }

    for (auto& [id, road] : roads) {
        //cout << road.id << endl;
        for (int associatedVIId : road.associatedVirtualIntersectionIds) {
            shared_ptr<VirtualIntersection> associatedVI = make_shared<VirtualIntersection>(virtualIntersections[associatedVIId]);
            road.associatedVirtualIntersections.push_back(associatedVI);
            //cout << associatedVI->id << endl;
        }
        for (int viId : roadToVI[road.id]) {
            virtualIntersections[viId].road = make_shared<Road>(road);
        }
        //cout << road.associatedVirtualIntersections.size() << endl;
    }

    int numVehicles;
    cin >> numVehicles;
    for (int i = 0; i < numVehicles; ++i) {
        Vehicle vehicle;
        int roadId, startingIntersectionId, endingIntersectionId;
        cin >> vehicle.id >> roadId >> vehicle.position >> startingIntersectionId >> endingIntersectionId;
        vehicle.road = make_shared<Road>(roads[roadId]);
        vehicle.starting = make_shared<VirtualIntersection>(virtualIntersections[startingIntersectionId]);
        vehicle.ending = make_shared<VirtualIntersection>(virtualIntersections[endingIntersectionId]);
        vehicles[vehicle.id] = vehicle;
    }

    cout << "Loaded " << intersections.size() << " intersections, "
         << roads.size() << " roads, "
         << virtualIntersections.size() << " virtual intersections, and "
         << vehicles.size() << " vehicles." << endl;
}

void AutoFlow() {

  float autoFlowPercentage = 0.9999f;
  
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
  for (auto& [id, vi] : virtualIntersections) {
    defaultG[vi.id] = 1e9;
  }
  
  for (auto& [id, vehicle] : vehicles) {
    int traversalTime = ceil(vehicle.road->traversalTime * (1 - vehicle.position));
    reservationTable[vehicle.road->id].upd(0, traversalTime, {1, true});
  }

  unordered_map<int, vector<VirtualIntersection>> paths;


  for (auto& [id, vehicle] : vehicles) {

    auto startTime = chrono::high_resolution_clock::now();

    priority_queue<PathNode> openNodes;
    set<int> openSet, closedSet;
    PathNode start(vehicle.starting);
    PathNode end(vehicle.ending);
    unordered_map<int, float> gScore = defaultG;
    gScore[start.intersection->id] = 0;
    openNodes.push(start);
    openSet.insert(start.intersection->id);
    vector<VirtualIntersection> path;

    while (!openNodes.empty()) {
      PathNode current = openNodes.top();
      //cout << current.intersection->id << endl;
      //cout << openNodes.size() << endl;
      openNodes.pop();
      openSet.erase(current.intersection->id);
      closedSet.insert(current.intersection->id);

      if ((*current.intersection).road == (*end.intersection).road) {

        cout << "Path found" << endl;
        
        vector<PathNode> temppath;
        temppath.push_back(current);
        while (current.parent != nullptr) {
          current = *current.parent;
          temppath.push_back(current);
        }
        // double check
        temppath.pop_back();
        reverse(temppath.begin(), temppath.end());

        // update restable
        for (int i = 0; i < temppath.size() - 1; i++) {
          int curTime = (int)temppath[i].g;
          int nextTime = (int)temppath[i + 1].g;
          int roadID = temppath[i].intersection->road->id;
          reservationTable[roadID].upd(curTime, nextTime, {1, true});
          path.push_back(*temppath[i+1].intersection);
        }

        break;
        
      }

      //cout << "hi" << endl;

      //cout << current.intersection->road->associatedVirtualIntersections.size() << endl;
      for (auto avi : current.intersection->road->associatedVirtualIntersections) {
        //cout << "Checking avi " << avi->id << endl;
        Intersection nodeNeeded;
        if (current.intersection->direction == 1) {
          nodeNeeded = *current.intersection->road->int2;
        } else {
          nodeNeeded = *current.intersection->road->int1;
        }

        if (avi->correspondingRealIntersection->id != nodeNeeded.id || avi->direction != current.intersection->direction) {
          continue;
        }

        for (auto neighbour : avi->connectingVirtualInts) {

          //cout << "Checking neighbour " << neighbour->id << endl;

          if (closedSet.count(neighbour->id)) {
            //cout << neighbour->id << " found" << endl;
            continue;
          }

          PathNode intermediary(neighbour, make_shared<PathNode>(current));
          PathNode neighNode(neighbour, make_shared<PathNode>(intermediary));

          neighNode.intersection->road->associatedVirtualIntersections.clear();

          for (auto avi2 : neighNode.intersection->road->associatedVirtualIntersectionIds) {
            neighNode.intersection->road->associatedVirtualIntersections.push_back(make_shared<VirtualIntersection>(virtualIntersections[avi2]));
          }

          
          Road road = *current.intersection->road;
          int currentTime = max((int)ceil(current.g), 0);
          float roadLeavingTime = currentTime;
          float congestion = reservationTable[road.id].query(roadLeavingTime, roadLeavingTime).sum / autoFlowPercentage;

          // binary search on the first index such that the range min is < capacity (log2(x)^2 complexity)
          int l = currentTime + 1;
          int r = 1 << 15;
          LazySeg tree = reservationTable[road.id];
          while (l < r) {
            int m = (l + r) / 2;
            if (tree.query(currentTime, m).mn < autoFlowPercentage * road.capacity) {
              r = m;
            } else {
              l = m + 1;
            }
          }
          roadLeavingTime = r;

          roadLeavingTime += max(0.001f, (road.length - 5 * congestion / road.laneCount) / road.speedLimit);

          Intersection rInt = *neighbour->correspondingRealIntersection;
          float cycleTime = rInt.trafficLightDuration * rInt.roadCount;
          roadLeavingTime += cycleTime * congestion / (road.laneCount * 5);

          neighNode.g = ceil(roadLeavingTime);
          neighNode.h = heuristic(*neighbour, *end.intersection);
          neighNode.f = neighNode.g + neighNode.h;
          intermediary.g = neighNode.g;
          intermediary.h = heuristic(*avi, *neighbour);
          intermediary.f = intermediary.g + intermediary.h;

          //cout << neighNode.g << " " << neighNode.h << " " << neighNode.f << endl;

          if (!openSet.count(neighbour->id)) {
            openNodes.push(neighNode);
            openSet.insert(neighbour->id);
          } else if (gScore[neighNode.intersection->id] < neighNode.g) {
            continue;
          }

          gScore[neighNode.intersection->id] = neighNode.g;
          gScore[intermediary.intersection->id] = intermediary.g;

        }


      }

      
    }

    paths[vehicle.id] = path;

    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    cout << "Vehicle " << vehicle.id << " path found in " << duration.count() << " ms" << endl;
    cout << path.size() << endl;

  }


}
 
int main() {
  
    readData();
    AutoFlow();


  return 0;
}
