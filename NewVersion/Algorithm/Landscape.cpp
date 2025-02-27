#include <bits/stdc++.h>
using namespace std;

class Intersection {
public:
    int id;
    float trafficLightDuration;
    int roadCount;
    float x;
    float y;
    int roadID;
    vector<int> connectingIntersectionIDs;
};

class Road {
public:
    int id;
    float length;
    int speedLimit;
    float capacity;
    int laneCount;
    int int1Id;
    int int2Id;
    float traversalTime;
    vector<vector<double>> positions;
};

class Vehicle {
public:
    int id;
    Road road;
    Road endingRoad;
    float position;
    vector<double> starting, ending;
};