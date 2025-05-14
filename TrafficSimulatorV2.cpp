#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <string>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>
#include <array>
#include <float.h>  // For FLT_MAX
#include "json.hpp" // Include JSON library
#ifdef _OPENMP
#include <omp.h>    // OpenMP for parallelization
#endif

using json = nlohmann::json;
using namespace std;

//=========================================
// SECTION 1: Data Structures and Constants
//=========================================

const float SIMULATION_TIME_STEP = 0.1f;  // seconds
const float VEHICLE_LENGTH = 4.5f;        // meters
const float VEHICLE_WIDTH = 2.0f;         // meters
const float MIN_FOLLOWING_DISTANCE = 2.0f; // meters
const float DRIVER_REACTION_TIME = 1.0f;   // seconds
const float ACCELERATION = 2.5f;           // m/s²
const float DECELERATION = 4.5f;           // m/s²
const float MAX_DECELERATION = 9.0f;       // m/s² (emergency braking)
const int SIMULATION_DURATION = 360000;    // seconds (1 hour)
const float LANE_CHANGE_COOLDOWN = 3.0f;   // Minimum time between lane changes
const float LANE_CHANGE_GAP_REQUIRED = 10.0f; // Minimum gap required to change lanes

// Fast approximation of inverse square root (Quake III algorithm)
inline float fastInvSqrt(float number) {
    const float x2 = number * 0.5F;
    const float threehalfs = 1.5F;

    union {
        float f;
        uint32_t i;
    } conv = {number}; // bit level hacking
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= threehalfs - (x2 * conv.f * conv.f); // 1st iteration
    // conv.f *= threehalfs - (x2 * conv.f * conv.f); // 2nd iteration, can be removed for more speed
    return conv.f;
}

// Fast square root using inverse square root
inline float fastSqrt(float number) {
    return number * fastInvSqrt(number);
}

// Simple vector class for 2D coordinates - heavily optimized with approximate operations
struct Vec2 {
    float x, y;
    
    Vec2(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}
    
    // Fast approximate distance - less accurate but much faster
    inline float distance(const Vec2& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return fastSqrt(dx*dx + dy*dy);
    }
    
    // Squared distance - avoids sqrt entirely when possible
    inline float distanceSquared(const Vec2& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return dx*dx + dy*dy;
    }
    
    inline Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    inline Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    
    inline Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    // Fast approximate normalization
    inline Vec2 normalized() const {
        float lenSq = x*x + y*y;
        if (lenSq < 0.0001f) return *this; // Avoid division by near-zero
        
        float invLen = fastInvSqrt(lenSq);
        return Vec2(x * invLen, y * invLen);
    }
};

// Forward declarations to resolve circular dependencies
struct Vehicle;

// Intersection class
struct Intersection {
    int id;
    Vec2 position;
    float trafficLightDuration;
    vector<int> connectingRoads;
    int currentGreenRoadIndex = 0;
    float timeSinceLastChange = 0.0f;
    int roadCount = 0;
    
    void updateTrafficLight(float deltaTime) {
        timeSinceLastChange += deltaTime;
        if (timeSinceLastChange >= trafficLightDuration) {
            timeSinceLastChange = 0.0f;
            currentGreenRoadIndex = (currentGreenRoadIndex + 1) % roadCount;
        }
    }
    
    // Simplified traffic light logic - uses bitwise operations for speed
    bool isGreen(int roadId) const {
        if (roadCount <= 1) return true; // If only one road, always green
        if (connectingRoads.empty()) return true;
        // Simple modulo replacement for roads that are powers of 2
        if ((roadCount & (roadCount - 1)) == 0) { // If roadCount is power of 2
            return connectingRoads[currentGreenRoadIndex & (roadCount - 1)] == roadId;
        } else {
            return connectingRoads[currentGreenRoadIndex % roadCount] == roadId;
        }
    }
};

// Road class with optimized data structures for vehicle tracking
struct Road {
    int id;
    float length;
    float speedLimit;
    int capacity;
    int laneCount;
    int int1Id;
    int int2Id;
    Vec2 startPos;
    Vec2 endPos;
    Vec2 direction;
    
    // Current state
    int vehicleCount = 0;
    float congestionFactor = 0.0f;
    
    // Spatial indexing for fast vehicle lookup - array of sorted vehicles per lane
    vector<vector<int>> vehiclesInLanes;
    
    Road() : id(-1), length(0), speedLimit(0), capacity(0), laneCount(1), 
             int1Id(-1), int2Id(-1) {}
    
    void initializeLanes() {
        vehiclesInLanes.resize(laneCount);
    }
             
    void calculateDirection(const Intersection& i1, const Intersection& i2) {
        startPos = Vec2(i1.position.x, i1.position.y);
        endPos = Vec2(i2.position.x, i2.position.y);
        direction = (endPos - startPos).normalized();
    }
    
    float calculateCongestionFactor() {
        // Simplified congestion model
        float normalizedCount = static_cast<float>(vehicleCount) / (capacity * laneCount);
        congestionFactor = normalizedCount * normalizedCount; // Faster than pow()
        return congestionFactor;
    }
    
    float getEffectiveSpeedLimit() const {
        // Simplified speed limit calculation - avoid multiplications
        if (congestionFactor > 0.9f) return 5.0f;
        else if (congestionFactor > 0.7f) return speedLimit * 0.3f;
        else if (congestionFactor > 0.5f) return speedLimit * 0.5f;
        else if (congestionFactor > 0.3f) return speedLimit * 0.7f;
        return speedLimit;
    }
    
    // Add vehicle to spatial index
    void addVehicle(int vehicleId, int lane, float position) {
        if (lane >= 0 && lane < vehiclesInLanes.size()) {
            vehiclesInLanes[lane].push_back(vehicleId);
            vehicleCount++;
        }
    }
    
    // Remove vehicle from spatial index
    void removeVehicle(int vehicleId, int lane) {
        if (lane >= 0 && lane < vehiclesInLanes.size()) {
            auto& laneVehicles = vehiclesInLanes[lane];
            laneVehicles.erase(
                remove(laneVehicles.begin(), laneVehicles.end(), vehicleId),
                laneVehicles.end());
            vehicleCount--;
        }
    }
    
    // Find nearest vehicle ahead in the same lane
    int findNearestVehicleAhead(float position, int lane, const vector<Vehicle>& allVehicles) const;
    
    // Find nearest vehicle behind in the same lane
    int findNearestVehicleBehind(float position, int lane, const vector<Vehicle>& allVehicles) const;
    
    // Methods declared but not implemented yet
    bool isLaneChangeSafe(float position, int currentLane, int targetLane, 
                        const vector<Vehicle>& allVehicles) const;
    
    void sortVehiclesByPosition(const vector<Vehicle>& allVehicles);
    
    // Determine which lanes can turn onto a specific next road
    vector<int> getLanesForNextRoad(int nextRoadId, const unordered_map<int, Road>& allRoads) const;
};

// Vehicle class
struct Vehicle {
    int id;
    int index; // Index in the vehicles vector for O(1) lookup
    int startingRoadId;
    int endingRoadId;
    float emissionRate;  // g/km
    int passengerCount;
    vector<int> route;
    
    // Current state
    int currentRoadId = -1;
    int nextRoadId = -1;
    int routeIndex = 0;
    float position = 0.0f;  // Position along current road (0 to 1)
    float speed = 0.0f;     // m/s
    int lane = 0;           // Lane number
    float timeSinceLastLaneChange = LANE_CHANGE_COOLDOWN; // To prevent constant lane changes
    int targetLane = -1;    // Target lane for changing (-1 if not changing)
    
    // Simulation metrics
    float travelTime = 0.0f;
    float totalEmissions = 0.0f;
    float distanceTraveled = 0.0f;
    bool hasArrived = false;
    float waitingTime = 0.0f;
    
    // For tracking average speed per road
    float maxSpeed = 0.0f; // Track maximum speed attained by this vehicle
    unordered_map<int, float> totalSpeedOnRoad; // Sum of all speeds on each road
    unordered_map<int, float> timeSpentOnRoad;  // Time spent on each road
    unordered_map<int, float> avgSpeedOnRoad;   // Calculated average speeds
    
    Vec2 getPosition(const Road& road) const {
        return road.startPos + road.direction * (road.length * position);
    }
    
    // Simplified lane change need detection
    bool needsLaneChange(const Road& currentRoad, const unordered_map<int, Road>& allRoads) const {
        if (nextRoadId == -1) return false;
        
        // Instead of getting all appropriate lanes, just check if we're in right or left
        // side of road for right/left turns
        auto nextRoadIt = allRoads.find(nextRoadId);
        if (nextRoadIt == allRoads.end()) return false;
        
        // Determine turn direction using simplified calculation
        // (just check shared intersection - no vector calculations)
        int sharedInt = -1;
        if (currentRoad.int1Id == nextRoadIt->second.int1Id || 
            currentRoad.int1Id == nextRoadIt->second.int2Id) {
            sharedInt = currentRoad.int1Id;
        } else if (currentRoad.int2Id == nextRoadIt->second.int1Id || 
                  currentRoad.int2Id == nextRoadIt->second.int2Id) {
            sharedInt = currentRoad.int2Id;
        }
        
        if (sharedInt == -1) return false;
        
        // Simple rule: for right turns, need to be in right lanes; for left turns, left lanes
        // This is less accurate but much faster than calculating vectors
        if (lane == 0 || lane == currentRoad.laneCount - 1) return false; // Already at edge
        
        // Simple heuristic: if next road has a higher ID, turn right; else turn left
        bool needRightLanes = (nextRoadId > currentRoad.id);
        if (needRightLanes && lane < currentRoad.laneCount / 2) return true;
        if (!needRightLanes && lane >= currentRoad.laneCount / 2) return true;
        
        return false;
    }
    
    // Simplified target lane finder
    int findBestTargetLane(const Road& currentRoad, const unordered_map<int, Road>& allRoads) const {
        if (nextRoadId == -1) return lane;
        
        // Simple heuristic: if next road has a higher ID, move right; else move left
        bool moveRight = (nextRoadId > currentRoad.id);
        
        if (moveRight) {
            return min(lane + 1, currentRoad.laneCount - 1);
        } else {
            return max(lane - 1, 0);
        }
    }
};

// Implementation of Road methods that depend on Vehicle now goes here, after Vehicle is fully defined
bool Road::isLaneChangeSafe(float position, int currentLane, int targetLane, 
                        const vector<Vehicle>& allVehicles) const {
    if (targetLane < 0 || targetLane >= laneCount) return false;
    
    // Use fixed distances for faster checks instead of complex calculations
    const float MIN_SAFE_DISTANCE_AHEAD = 8.0f;
    const float MIN_SAFE_DISTANCE_BEHIND = 5.0f;
    
    // Check for vehicles ahead in target lane
    int vehicleAheadId = findNearestVehicleAhead(position, targetLane, allVehicles);
    if (vehicleAheadId != -1) {
        const auto& vehicleAhead = allVehicles[vehicleAheadId];
        float distanceAhead = (vehicleAhead.position - position) * length;
        if (distanceAhead < MIN_SAFE_DISTANCE_AHEAD) return false;
    }
    
    // Check for vehicles behind in target lane
    int vehicleBehindId = findNearestVehicleBehind(position, targetLane, allVehicles);
    if (vehicleBehindId != -1) {
        const auto& vehicleBehind = allVehicles[vehicleBehindId];
        float distanceBehind = (position - vehicleBehind.position) * length;
        if (distanceBehind < MIN_SAFE_DISTANCE_BEHIND) return false;
    }
    
    return true;
}

void Road::sortVehiclesByPosition(const vector<Vehicle>& allVehicles) {
    for (auto& laneVehicles : vehiclesInLanes) {
        sort(laneVehicles.begin(), laneVehicles.end(), 
            [&allVehicles](int id1, int id2) {
                return allVehicles[id1].position < allVehicles[id2].position;
            });
    }
}

// Implementation of methods that depend on Vehicle type being complete
int Road::findNearestVehicleAhead(float position, int lane, const vector<Vehicle>& allVehicles) const {
    if (lane < 0 || lane >= vehiclesInLanes.size()) return -1;
    
    const auto& laneVehicles = vehiclesInLanes[lane];
    float minDistance = FLT_MAX;
    int nearestVehicleId = -1;
    
    for (int vehicleId : laneVehicles) {
        const auto& otherVehicle = allVehicles[vehicleId];
        if (otherVehicle.position > position) {
            float distance = (otherVehicle.position - position) * length;
            if (distance < minDistance) {
                minDistance = distance;
                nearestVehicleId = vehicleId;
            }
        }
    }
    
    return nearestVehicleId;
}

int Road::findNearestVehicleBehind(float position, int lane, const vector<Vehicle>& allVehicles) const {
    if (lane < 0 || lane >= vehiclesInLanes.size()) return -1;
    
    const auto& laneVehicles = vehiclesInLanes[lane];
    float minDistance = FLT_MAX;
    int nearestVehicleId = -1;
    
    for (int vehicleId : laneVehicles) {
        const auto& otherVehicle = allVehicles[vehicleId];
        if (otherVehicle.position < position) {
            float distance = (position - otherVehicle.position) * length;
            if (distance < minDistance) {
                minDistance = distance;
                nearestVehicleId = vehicleId;
            }
        }
    }
    
    return nearestVehicleId;
}

// Add the missing method implementation here
vector<int> Road::getLanesForNextRoad(int nextRoadId, const unordered_map<int, Road>& allRoads) const {
    vector<int> appropriateLanes;
    
    // Fast implementation - simplified for performance
    auto nextRoadIt = allRoads.find(nextRoadId);
    if (nextRoadIt == allRoads.end()) return appropriateLanes;
    
    const Road& nextRoad = nextRoadIt->second;
    
    // Simplified lane selection based on road IDs and shared intersection
    int sharedIntersectionId = -1;
    if (int1Id == nextRoad.int1Id || int1Id == nextRoad.int2Id) {
        sharedIntersectionId = int1Id;
    } else if (int2Id == nextRoad.int1Id || int2Id == nextRoad.int2Id) {
        sharedIntersectionId = int2Id;
    }
    
    if (sharedIntersectionId == -1) return appropriateLanes; // No connection
    
    // Use simple heuristic: higher ID roads tend to be to the right
    bool turnRight = (nextRoadId > id);
    int numLanes = laneCount;
    
    if (turnRight) {
        // Right turn: use right half of lanes
        int start = numLanes / 2;
        for (int i = start; i < numLanes; i++) {
            appropriateLanes.push_back(i);
        }
    } else {
        // Left turn: use left half of lanes
        int end = numLanes / 2 + 1;
        for (int i = 0; i < end; i++) {
            appropriateLanes.push_back(i);
        }
    }
    
    // If no lanes were selected, allow all lanes
    if (appropriateLanes.empty()) {
        for (int i = 0; i < numLanes; i++) {
            appropriateLanes.push_back(i);
        }
    }
    
    return appropriateLanes;
}

//===========================
// SECTION 2: File I/O
//===========================

class TrafficSimulation {
private:
    unordered_map<int, Intersection> intersections;
    unordered_map<int, Road> roads;
    vector<Vehicle> vehicles;
    float simulationTime = 0.0f;
    
    // Efficient lookup structures
    unordered_map<int, int> roadConnections;  // Quick lookup for connected roads
    
    // Metrics
    float totalEmissions = 0.0f;
    float totalTravelTime = 0.0f;
    float totalPassengerTime = 0.0f;
    int vehiclesCompleted = 0;
    int laneChangeCount = 0;
    
    // Added for congestion logging
    float lastCongestionLogTime = 0.0f;
    ofstream congestionLogFile;
    
public:
    bool loadSimulationData(const string& filename) {
        try {
            ifstream file(filename);
            if (!file.is_open()) {
                cerr << "Failed to open simulation data file: " << filename << endl;
                return false;
            }
            
            json data = json::parse(file);
            
            // Load intersections
            for (const auto& intData : data["intersections"]) {
                Intersection intersection;
                intersection.id = intData["id"];
                intersection.position.x = intData["x"];
                intersection.position.y = intData["y"];
                intersection.trafficLightDuration = intData["traffic_light_duration"];
                
                for (const auto& roadId : intData["connecting_roads"]) {
                    intersection.connectingRoads.push_back(roadId);
                }
                
                intersection.roadCount = intData["road_count"];
                intersections[intersection.id] = intersection;
            }
            
            // Pre-allocate roads to avoid rehashing during loading
            roads.reserve(data["roads"].size());
            
            // Load roads
            for (const auto& roadData : data["roads"]) {
                Road road;
                road.id = roadData["id"];
                road.length = roadData["length"];
                road.speedLimit = roadData["speed_limit"];
                road.capacity = roadData["capacity"];
                road.laneCount = roadData["lane_count"];
                road.int1Id = roadData["int1_id"];
                road.int2Id = roadData["int2_id"];
                
                // Calculate road direction
                if (road.int1Id >= 0 && road.int2Id >= 0) {
                    auto it1 = intersections.find(road.int1Id);
                    auto it2 = intersections.find(road.int2Id);
                    if (it1 != intersections.end() && it2 != intersections.end()) {
                        road.calculateDirection(it1->second, it2->second);
                    }
                }
                
                // Initialize lanes for vehicle tracking
                road.initializeLanes();
                roads[road.id] = road;
                
                // Build road connection lookup table for faster route finding
                int key = (road.int1Id << 16) | road.int2Id;  // combine the two IDs into a single key
                roadConnections[key] = road.id;
                // Also add the reverse direction since roads are bi-directional
                key = (road.int2Id << 16) | road.int1Id;
                roadConnections[key] = road.id;
            }
            
            // Pre-allocate vehicles vector to avoid reallocation
            vehicles.reserve(data["vehicles"].size());
            
            // Load vehicles
            for (const auto& vehicleData : data["vehicles"]) {
                Vehicle vehicle;
                vehicle.id = vehicleData["id"];
                vehicle.index = vehicles.size();  // Store the index for O(1) lookup
                vehicle.startingRoadId = vehicleData["starting_road"];
                vehicle.endingRoadId = vehicleData["ending_road"];
                vehicle.emissionRate = vehicleData["emission_rate"];
                vehicle.passengerCount = vehicleData["passenger_count"];
                
                for (const auto& intersectionId : vehicleData["route"]) {
                    vehicle.route.push_back(intersectionId);
                }
                
                // Initialize vehicle position on starting road
                if (!vehicle.route.empty()) {
                    vehicle.currentRoadId = vehicle.startingRoadId;
                    vehicle.position = 0.0f; // Start of the road
                    
                    // Get road and assign a lane
                    auto roadIt = roads.find(vehicle.currentRoadId);
                    if (roadIt != roads.end()) {
                        vehicle.lane = rand() % roadIt->second.laneCount;
                        roadIt->second.addVehicle(vehicle.index, vehicle.lane, vehicle.position);
                    }
                    
                    // Find next road in route using the fast lookup table
                    if (vehicle.route.size() > 1) {
                        int currInt = vehicle.route[0];
                        int nextInt = vehicle.route[1];
                        
                        // Use road connection lookup table
                        int key = (currInt << 16) | nextInt;
                        auto connectionIt = roadConnections.find(key);
                        if (connectionIt != roadConnections.end()) {
                            vehicle.nextRoadId = connectionIt->second;
                        } else {
                            vehicle.nextRoadId = -1;
                        }
                    } else {
                        // Route only has one intersection - vehicle is already at destination
                        vehicle.nextRoadId = -1;
                    }
                }
                
                vehicles.push_back(vehicle);
            }
            
            // Sort vehicles in each lane for faster lookups
            for (auto& [id, road] : roads) {
                road.sortVehiclesByPosition(vehicles);
            }
            
            cout << "Loaded " << intersections.size() << " intersections, " 
                 << roads.size() << " roads, and " << vehicles.size() << " vehicles." << endl;
            return true;
            
        } catch (const exception& e) {
            cerr << "Error parsing simulation data: " << e.what() << endl;
            return false;
        }
    }

//===================================
// SECTION 3: Vehicle Physics
//===================================

private:
    // Simplified lane change processing
    bool processLaneChange(Vehicle& vehicle, Road& currentRoad) {
        vehicle.timeSinceLastLaneChange += SIMULATION_TIME_STEP;
        
        if (vehicle.timeSinceLastLaneChange < LANE_CHANGE_COOLDOWN) {
            return false;
        }
        
        // Reduce frequency of lane change checks - only consider every 1.5 seconds
        if (fmodf(vehicle.timeSinceLastLaneChange, 1.5f) > 0.2f) {
            return false;
        }
        
        // Simplified lane change logic - less accurate
        float distanceToIntersection = (1.0f - vehicle.position) * currentRoad.length;
        
        // Only consider lane changes at specific distance ranges
        if (distanceToIntersection < 150.0f && distanceToIntersection > 30.0f) {
            // Simple decision: check if lane change needed and attempt to move one lane
            if (vehicle.needsLaneChange(currentRoad, roads)) {
                if (vehicle.lane < currentRoad.laneCount - 1 && 
                    currentRoad.isLaneChangeSafe(vehicle.position, vehicle.lane, vehicle.lane + 1, vehicles)) {
                    // Execute lane change to right
                    currentRoad.removeVehicle(vehicle.index, vehicle.lane);
                    vehicle.lane += 1;
                    currentRoad.addVehicle(vehicle.index, vehicle.lane, vehicle.position);
                    vehicle.timeSinceLastLaneChange = 0.0f;
                    laneChangeCount++;
                    return true;
                } else if (vehicle.lane > 0 && 
                         currentRoad.isLaneChangeSafe(vehicle.position, vehicle.lane, vehicle.lane - 1, vehicles)) {
                    // Execute lane change to left
                    currentRoad.removeVehicle(vehicle.index, vehicle.lane);
                    vehicle.lane -= 1;
                    currentRoad.addVehicle(vehicle.index, vehicle.lane, vehicle.position);
                    vehicle.timeSinceLastLaneChange = 0.0f;
                    laneChangeCount++;
                    return true;
                }
            }
        }
        
        // Simplified speed optimization logic
        int vehicleAheadId = currentRoad.findNearestVehicleAhead(vehicle.position, vehicle.lane, vehicles);
        if (vehicleAheadId != -1) {
            const auto& vehicleAhead = vehicles[vehicleAheadId];
            float distanceAhead = (vehicleAhead.position - vehicle.position) * currentRoad.length;
            
            // If vehicle ahead is slowing us down and we're not close to intersection
            if (distanceAhead < 40.0f && vehicleAhead.speed < vehicle.speed * 0.7f && 
                distanceToIntersection > 100.0f) {
                
                // Try to overtake with minimal checks
                int targetLane = (vehicle.lane == 0) ? 1 : vehicle.lane - 1; // Prefer left
                if (targetLane >= 0 && targetLane < currentRoad.laneCount && 
                    currentRoad.isLaneChangeSafe(vehicle.position, vehicle.lane, targetLane, vehicles)) {
                    currentRoad.removeVehicle(vehicle.index, vehicle.lane);
                    vehicle.lane = targetLane;
                    currentRoad.addVehicle(vehicle.index, vehicle.lane, vehicle.position);
                    vehicle.timeSinceLastLaneChange = 0.0f;
                    laneChangeCount++;
                    return true;
                }
            }
        }
        
        return false;
    }
    
    // Simplified physics update with less accuracy but more speed
    void updateVehiclePhysics(Vehicle& vehicle, float deltaTime) {
        if (vehicle.hasArrived) return;
        
        // Direct reference to road for speed
        Road& currentRoad = roads.find(vehicle.currentRoadId)->second;
        float targetSpeed = currentRoad.getEffectiveSpeedLimit();
        
        // Simplified intersection approach logic
        float distanceToIntersection = (1.0f - vehicle.position) * currentRoad.length;
        
        // Check traffic light less frequently and with simplified logic
        if (distanceToIntersection < 40.0f) {
            int nextIntersectionId = (currentRoad.int1Id == vehicle.route[vehicle.routeIndex]) ? 
                                     currentRoad.int2Id : currentRoad.int1Id;
            
            auto intersectionIt = intersections.find(nextIntersectionId);
            if (intersectionIt != intersections.end() && !intersectionIt->second.isGreen(vehicle.currentRoadId)) {
                // Red light - simplified braking
                if (distanceToIntersection < 10.0f) {
                    targetSpeed = 0.0f; // Stop
                    vehicle.waitingTime += deltaTime;
                } else {
                    targetSpeed *= 0.4f; // Simple speed reduction
                }
            }
        }
        
        // Process lane changes less frequently
        if ((int)(vehicle.travelTime * 10.0f) % 5 == 0) { // Check every 0.5 seconds
            processLaneChange(vehicle, currentRoad);
        } else {
            vehicle.timeSinceLastLaneChange += deltaTime;
        }
        
        // Simplified collision avoidance
        int nearestVehicleId = currentRoad.findNearestVehicleAhead(vehicle.position, vehicle.lane, vehicles);
        if (nearestVehicleId != -1) {
            const auto& otherVehicle = vehicles[nearestVehicleId];
            float distance = (otherVehicle.position - vehicle.position) * currentRoad.length;
            
            // Simplified safety distance calculation - less accurate, more performance
            float safeDistance = vehicle.speed * 0.8f + MIN_FOLLOWING_DISTANCE;
            
            if (distance < safeDistance) {
                // Simple braking model - just match speed of vehicle ahead with a gap
                targetSpeed = otherVehicle.speed * 0.9f;
                vehicle.waitingTime += deltaTime * 0.5f;
            }
        }
        
        // Update speed with simplified model
        if (vehicle.speed < targetSpeed) {
            vehicle.speed = min(vehicle.speed + ACCELERATION * deltaTime, targetSpeed);
        } else if (vehicle.speed > targetSpeed) {
            vehicle.speed = max(vehicle.speed - DECELERATION * deltaTime, targetSpeed);
        }
        
        // Track maximum speed for this vehicle
        vehicle.maxSpeed = max(vehicle.maxSpeed, vehicle.speed);
        
        // Track speed data for calculating averages
        vehicle.totalSpeedOnRoad[vehicle.currentRoadId] += vehicle.speed;
        vehicle.timeSpentOnRoad[vehicle.currentRoadId]++;
        
        // Update position
        float oldPosition = vehicle.position;
        vehicle.position += (vehicle.speed * deltaTime) / currentRoad.length;
        
        // Simplified emissions calculation
        float distanceThisUpdate = (vehicle.position - oldPosition) * currentRoad.length;
        vehicle.distanceTraveled += distanceThisUpdate;
        vehicle.totalEmissions += distanceThisUpdate * vehicle.emissionRate * 0.001f;
        totalEmissions += distanceThisUpdate * vehicle.emissionRate * 0.001f;
        
        if (vehicle.position >= 1.0f) {
            handleRoadTransition(vehicle);
        }
        
        vehicle.travelTime += deltaTime;
    }

//=======================================
// SECTION 4: Road and Intersection Management
//=======================================

    void handleRoadTransition(Vehicle& vehicle) {
        // Calculate average speed for the road being exited
        int oldRoadId = vehicle.currentRoadId;
        if (vehicle.timeSpentOnRoad[oldRoadId] > 0.0f) {
            vehicle.avgSpeedOnRoad[oldRoadId] = 
                vehicle.totalSpeedOnRoad[oldRoadId] / vehicle.timeSpentOnRoad[oldRoadId];
        }
        
        // Vehicle has reached the end of the current road
        vehicle.routeIndex++;
        
        // Check if vehicle has completed its route
        if (vehicle.routeIndex >= vehicle.route.size() - 1 || vehicle.nextRoadId == -1) {
            // Vehicle has arrived at destination
            vehicle.hasArrived = true;
            vehiclesCompleted++;
            totalTravelTime += vehicle.travelTime;
            totalPassengerTime += vehicle.travelTime * vehicle.passengerCount;
            
            // Remove vehicle from current road's spatial index
            auto roadIt = roads.find(vehicle.currentRoadId);
            if (roadIt != roads.end()) {
                roadIt->second.removeVehicle(vehicle.index, vehicle.lane);
            }
            
            return;
        }
        
        // Move to next road - update spatial indices
        auto currentRoadIt = roads.find(vehicle.currentRoadId);
        if (currentRoadIt != roads.end()) {
            currentRoadIt->second.removeVehicle(vehicle.index, vehicle.lane);
        }
        
        vehicle.currentRoadId = vehicle.nextRoadId;
        vehicle.position = 0.0f;  // Start of the new road
        
        // Assign a lane based on turn-specific lane logic
        auto nextRoadIt = roads.find(vehicle.currentRoadId);
        if (nextRoadIt != roads.end()) {
            // Find appropriate lane for next turn (if any)
            if (vehicle.routeIndex + 1 < vehicle.route.size()) {
                int currInt = vehicle.route[vehicle.routeIndex];
                int nextInt = vehicle.route[vehicle.routeIndex + 1];
                
                int key = (currInt << 16) | nextInt;
                auto connectionIt = roadConnections.find(key);
                
                if (connectionIt != roadConnections.end()) {
                    vehicle.nextRoadId = connectionIt->second;
                    
                    // Get appropriate lanes for the upcoming turn
                    auto& currRoad = nextRoadIt->second;
                    vector<int> appropriateLanes = currRoad.getLanesForNextRoad(vehicle.nextRoadId, roads);
                    
                    if (!appropriateLanes.empty()) {
                        // Choose a lane more intelligently - prefer middle lanes of appropriate ones
                        sort(appropriateLanes.begin(), appropriateLanes.end());
                        if (appropriateLanes.size() > 1) {
                            // Pick from middle to minimize future lane changes
                            vehicle.lane = appropriateLanes[appropriateLanes.size() / 2];
                        } else {
                            vehicle.lane = appropriateLanes[0];
                        }
                    } else {
                        // If no specific lane requirements, choose middle lane
                        vehicle.lane = currRoad.laneCount / 2;
                    }
                } else {
                    vehicle.nextRoadId = -1;
                    vehicle.lane = nextRoadIt->second.laneCount / 2; // Middle lane
                }
            } else {
                vehicle.nextRoadId = -1;
                vehicle.lane = nextRoadIt->second.laneCount / 2; // Middle lane
            }
            
            // Add vehicle to the new road
            nextRoadIt->second.addVehicle(vehicle.index, vehicle.lane, vehicle.position);
        }
    }

//===================================
// SECTION 5: Traffic Light System
//===================================

    void updateTrafficLights(float deltaTime) {
        for (auto& [id, intersection] : intersections) {
            intersection.updateTrafficLight(deltaTime);
        }
    }

//===================================
// SECTION 6: Simulation Loop
//===================================

public:
    // Optimized simulation loop with reduced accuracy but much better performance
    void runSimulation() {
        cout << "Starting simulation..." << endl;
        
        int totalSteps = static_cast<int>(SIMULATION_DURATION / SIMULATION_TIME_STEP);
        int reportInterval = totalSteps / 10;
        
        // Pre-sort vehicles once at the start
        for (auto& [id, road] : roads) {
            road.sortVehiclesByPosition(vehicles);
        }
        
        for (int step = 0; step < totalSteps; ++step) {
            simulationTime += SIMULATION_TIME_STEP;
            
            // Update traffic lights less frequently - every 1 second
            if (step % 10 == 0) {
                for (auto& [id, intersection] : intersections) {
                    intersection.updateTrafficLight(SIMULATION_TIME_STEP * 10);
                }
            }
            
            // Sort vehicles much less frequently - every 5 seconds
            if (step % 50 == 0) {
                for (auto& [id, road] : roads) {
                    road.sortVehiclesByPosition(vehicles);
                }
            }
            
            // Update vehicle positions with parallel processing if available
            #ifdef _OPENMP
            #pragma omp parallel for schedule(dynamic)
            #endif
            for (int i = 0; i < vehicles.size(); i++) {
                if (!vehicles[i].hasArrived) {
                    updateVehiclePhysics(vehicles[i], SIMULATION_TIME_STEP);
                }
            }
            
            // Log congestion data every 10 seconds of simulation time
            if (simulationTime - lastCongestionLogTime >= 10.0f) {
                updateRoadCongestion();
                logRoadCongestion();
                lastCongestionLogTime = simulationTime;
            }
            
            // Report progress less frequently
            if (step % reportInterval == 0 || step == totalSteps - 1) {
                float percentComplete = 100.0f * step / totalSteps;
                cout << "Simulation " << percentComplete << "% complete. ";
                cout << vehiclesCompleted << "/" << vehicles.size() << " vehicles arrived." << endl;
            }
            
            // Early termination if all vehicles have completed
            if (vehiclesCompleted == vehicles.size()) {
                cout << "All vehicles have reached their destinations." << endl;
                break;
            }
        }
        
        // Close congestion log file
        if (congestionLogFile.is_open()) {
            congestionLogFile.close();
            cout << "Road congestion log written to road_congestion_log.txt" << endl;
        }
        
        generateReport();
    }

//===================================
// SECTION 7: Congestion Modeling - Optimized to use tracked counts
//===================================

private:
    void updateRoadCongestion() {
        // No need to recount vehicles as we're tracking them accurately via addVehicle/removeVehicle
        // Just update congestion factors based on the current count
        for (auto& [id, road] : roads) {
            road.calculateCongestionFactor();
        }
    }
    
    // New method to log congestion data at intervals
    void logRoadCongestion() {
        if (!congestionLogFile.is_open()) {
            congestionLogFile.open("road_congestion_log.txt");
            if (congestionLogFile.is_open()) {
                // Write header
                congestionLogFile << "Time,";
                for (const auto& [id, road] : roads) {
                    congestionLogFile << id << ",";
                }
                congestionLogFile << "\n";
            } else {
                cerr << "Failed to open congestion log file" << endl;
                return;
            }
        }
        
        if (congestionLogFile.is_open()) {
            // Write timestamp and congestion for each road
            congestionLogFile << simulationTime << ",";
            for (const auto& [id, road] : roads) {
                congestionLogFile << road.congestionFactor << ",";
            }
            congestionLogFile << "\n";
        }
    }

//===================================
// SECTION 8: Metrics Tracking
//===================================

    void calculateMetrics() {
        // Most metrics are tracked during simulation, but we can calculate averages here
        float avgTravelTime = vehiclesCompleted > 0 ? totalTravelTime / vehiclesCompleted : 0;
        float avgPassengerTime = vehiclesCompleted > 0 ? totalPassengerTime / vehiclesCompleted : 0;
        
        // Count vehicles still on the road
        int vehiclesRemaining = vehicles.size() - vehiclesCompleted;
        
        // Calculate total distance traveled
        float totalDistance = 0.0f;
        for (const auto& vehicle : vehicles) {
            totalDistance += vehicle.distanceTraveled;
        }
        
        // Calculate average speed
        float avgSpeed = totalTravelTime > 0 ? totalDistance / totalTravelTime : 0;
        
        // Calculate total waiting time
        float totalWaitingTime = 0.0f;
        for (const auto& vehicle : vehicles) {
            totalWaitingTime += vehicle.waitingTime;
        }
        
        // Calculate emissions per km
        float emissionsPerKm = totalDistance > 0 ? totalEmissions / (totalDistance / 1000.0f) : 0;
        
        cout << "\n=== METRICS SUMMARY ===" << endl;
        cout << "Total vehicles: " << vehicles.size() << endl;
        cout << "Vehicles completed: " << vehiclesCompleted << " (" 
             << (100.0f * vehiclesCompleted / vehicles.size()) << "%)" << endl;
        cout << "Vehicles still on road: " << vehiclesRemaining << endl;
        cout << "Total distance traveled: " << totalDistance << " meters" << endl;
        cout << "Average travel time: " << avgTravelTime << " seconds" << endl;
        // write this to a file
        cout << "Average passenger-weighted travel time: " << avgPassengerTime << " seconds" << endl;
        ofstream metricsFile("current_time.txt");
        metricsFile << avgPassengerTime << endl;
        metricsFile.close();
        cout << "Average speed: " << avgSpeed << " m/s" << endl;
        cout << "Total waiting time: " << totalWaitingTime << " seconds" << endl;
        cout << "Total emissions: " << totalEmissions << " g CO2" << endl;
        cout << "Emissions per km: " << emissionsPerKm << " g/km" << endl;
        cout << "Total lane changes: " << laneChangeCount << endl;
        cout << "Average lane changes per vehicle: " << static_cast<float>(laneChangeCount) / vehicles.size() << endl;
    }

//===================================
// SECTION 9: Output Generation
//===================================

    void generateVehicleSpeedReport() {
        map<int, map<int, float>> congestionMap;
        // read in vehicle_road_speeds.csv
        ifstream csvFile("vehicle_road_speeds.csv");
        if (!csvFile.is_open()) {
            cout << "Failed to open vehicle_road_speeds.csv" << endl;
        } else {
            string line, value;
            vector<int> roadIds;

            //float avgPassengerTime = 0.0f;
            // take it in

            // Read header row to get road IDs
            if (getline(csvFile, line)) {
                stringstream ss(line);
                
                // Skip the first column header
                // getline(ss, value, ',');
                // avgPassengerTime = stof(value);

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
        
        
        ofstream speedReportFile("vehicle_road_speeds.csv");
        if (!speedReportFile.is_open()) {
            cerr << "Failed to open vehicle speed report file." << endl;
            return;
        }
        
        // avgPassengerTime = vehiclesCompleted > 0 ? totalPassengerTime / vehiclesCompleted : 0;
        // speedReportFile << avgPassengerTime << ",";
        
        // Write header with Road IDs
        speedReportFile << "Vehicle ID,";
        for (const auto& [roadId, road] : roads) {
            speedReportFile << roadId << ",";
        }
        speedReportFile << "\n";
        
        // Write data for each vehicle
        for (const auto& vehicle : vehicles) {
            speedReportFile << vehicle.id << ",";
            
            for (const auto& [roadId, road] : roads) {
                // If vehicle has traveled on this road, output the average speed
                auto it = vehicle.avgSpeedOnRoad.find(roadId);
                if (congestionMap[vehicle.id][roadId] != 0) {
                    if (it != vehicle.avgSpeedOnRoad.end()) {
                        // Use iterator's second value instead of operator[]
                        speedReportFile << (it->second * 1.5 + congestionMap[vehicle.id][roadId]) / 2.5 << ",";
                    }
                    // Otherwise use the vehicle's maximum speed as a placeholder
                    else {
                        speedReportFile << (vehicle.maxSpeed * 1.5 + congestionMap[vehicle.id][roadId]) / 2.5 << ",";
                    }
                } else {
                    if (it != vehicle.avgSpeedOnRoad.end()) {
                        speedReportFile << it->second << ",";
                    } else {
                        speedReportFile << 0 << ","; // No data available
                    }
                }
                
            }
            speedReportFile << "\n";
        }
        
        speedReportFile.close();
        cout << "Vehicle average speeds on roads written to vehicle_road_speeds.csv" << endl;
    }

    void generateReport() {
        calculateMetrics();
        
        // Write detailed results to file
        ofstream reportFile("simulation_results.txt");
        if (reportFile.is_open()) {
            reportFile << "Vehicle ID,Origin,Destination,Travel Time,Distance,Emissions,Avg Speed,Passengers,Completed\n";
            
            for (const auto& vehicle : vehicles) {
                float avgSpeed = vehicle.travelTime > 0 ? vehicle.distanceTraveled / vehicle.travelTime : 0;
                
                reportFile << vehicle.id << ","
                          << vehicle.startingRoadId << ","
                          << vehicle.endingRoadId << ","
                          << vehicle.travelTime << ","
                          << vehicle.distanceTraveled << ","
                          << vehicle.totalEmissions << ","
                          << avgSpeed << ","
                          << vehicle.passengerCount << ","
                          << (vehicle.hasArrived ? "Yes" : "No") << "\n";
            }
            
            reportFile.close();
            cout << "\nDetailed results written to simulation_results.txt" << endl;
        }
        
        // Write congestion data
        ofstream congestionFile("road_congestion.txt");
        if (congestionFile.is_open()) {
            congestionFile << "Road ID,Length,Capacity,Vehicle Count,Congestion Factor\n";
            
            for (const auto& [id, road] : roads) {
                congestionFile << road.id << ","
                              << road.length << ","
                              << road.capacity << ","
                              << road.vehicleCount << ","
                              << road.congestionFactor << "\n";
            }
            
            congestionFile.close();
            cout << "Road congestion data written to road_congestion.txt" << endl;
        }

        // Generate the vehicle speed report
        generateVehicleSpeedReport();
    }
    
    
};

//===================================
// SECTION 10: Main Function
//===================================

int main(int argc, char* argv[]) {
    // Set random seed
    srand(static_cast<unsigned>(time(nullptr)));
    
    // Create simulation
    TrafficSimulation simulation;
    
    // Load data
    string filename = "simulation_data.json";
    if (argc > 1) {
        filename = argv[1];
    }
    
    if (!simulation.loadSimulationData(filename)) {
        cerr << "Failed to load simulation data." << endl;
        return 1;
    }
    
    // Run simulation
    auto startTime = chrono::high_resolution_clock::now();
    simulation.runSimulation();
    auto endTime = chrono::high_resolution_clock::now();
    
    // Report execution time
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
    cout << "\nSimulation completed in " << duration << "ms" << endl;
    
    return 0;
}
