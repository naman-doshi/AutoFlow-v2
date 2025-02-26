#include <bits/stdc++.h>
using namespace std;

class VirtualIntersection;

class Intersection {
public:
    int id;
    float trafficLightDuration;
    int roadCount;
    float x;
    float y;
    int roadID;
};

class Road {
public:
    int id;
    float length;
    int speedLimit;
    float capacity;
    int laneCount;
    shared_ptr<Intersection> int1;
    shared_ptr<Intersection> int2;
    vector<shared_ptr<VirtualIntersection> > associatedVirtualIntersections;
    vector<int> associatedVirtualIntersectionIds;
    float traversalTime;
};

class VirtualIntersection : public Intersection {
public:
    shared_ptr<Intersection> correspondingRealIntersection;
    int direction;
    vector<shared_ptr<VirtualIntersection> > connectingVirtualInts;
    shared_ptr<Road> road;
    vector<int> connectingVIIds;
};

class Vehicle {
public:
    int id;
    shared_ptr<Road> road;
    float position;
    shared_ptr<VirtualIntersection> starting;
    shared_ptr<VirtualIntersection> ending;
};