#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>

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
    int partition = 0; // Added partition ID field
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
    int startingRoadId;
    int endingRoadId;
    float position;
    vector<double> starting, ending;
    int passengerCount = 1;     // Default 1 passenger
    float emissionRate = 1.0f;  // Default emission rate (normalized)
    
    // Calculate priority score (higher is more important)
    float getPriorityScore() const {
        return passengerCount * emissionRate;
    }
};