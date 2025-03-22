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

// Simple vector class for 2D coordinates - optimized with inline operations
struct Vec2 {
    float x, y;
    
    Vec2(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}
    
    inline float distance(const Vec2& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return sqrt(dx*dx + dy*dy);
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
    
    inline Vec2 normalized() const {
        float len = sqrt(x*x + y*y);
        if (len > 0.001f) {
            return Vec2(x / len, y / len);
        }
        return *this;
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
    
    bool isGreen(int roadId) const {
        if (roadCount <= 1) return true; // If only one road, always green
        if (connectingRoads.empty()) return true;
        return connectingRoads[currentGreenRoadIndex] == roadId;
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
        // Simple model: congestion increases non-linearly with vehicle count
        float normalizedCount = static_cast<float>(vehicleCount) / (capacity * laneCount);
        congestionFactor = pow(normalizedCount, 2);
        return congestionFactor;
    }
    
    float getEffectiveSpeedLimit() const {
        // Speed drops with congestion
        return max(speedLimit * (1.0f - 0.7f * congestionFactor), 5.0f); // Minimum 5 m/s
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
    
    // Check if lane change is safe
    bool isLaneChangeSafe(float position, int currentLane, int targetLane, const vector<Vehicle>& allVehicles) const;
    
    // Sort vehicles in each lane by position for faster lookup
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
    
    Vec2 getPosition(const Road& road) const {
        return road.startPos + road.direction * (road.length * position);
    }
    
    // Check if we need to change lanes based on our next turn
    bool needsLaneChange(const Road& currentRoad, const unordered_map<int, Road>& allRoads) const {
        if (nextRoadId == -1) return false;
        
        vector<int> appropriateLanes = currentRoad.getLanesForNextRoad(nextRoadId, allRoads);
        if (appropriateLanes.empty()) return false; // No specific lanes required
        
        // Check if we're already in an appropriate lane
        return std::find(appropriateLanes.begin(), appropriateLanes.end(), lane) == appropriateLanes.end();
    }
    
    // Find best target lane when a lane change is needed
    int findBestTargetLane(const Road& currentRoad, const unordered_map<int, Road>& allRoads) const {
        if (nextRoadId == -1) return lane; // No change needed
        
        vector<int> appropriateLanes = currentRoad.getLanesForNextRoad(nextRoadId, allRoads);
        if (appropriateLanes.empty()) return lane; // No specific lanes required
        
        // Find the closest appropriate lane that is adjacent or current
        int bestLane = lane;
        
        // Check if current lane is already appropriate
        if (find(appropriateLanes.begin(), appropriateLanes.end(), lane) != appropriateLanes.end()) {
            return lane; // Already in an appropriate lane
        }
        
        // Find the closest lane that is appropriate and adjacent to current lane
        int closestLane = -1;
        int minDistance = currentRoad.laneCount;
        
        for (int candidateLane : appropriateLanes) {
            int distance = abs(candidateLane - lane);
            if (distance < minDistance) {
                minDistance = distance;
                closestLane = candidateLane;
            }
        }
        
        // If closest lane is directly adjacent, choose it
        if (closestLane != -1 && abs(closestLane - lane) == 1) {
            bestLane = closestLane;
        } 
        // Otherwise, move one lane in the direction of the closest appropriate lane
        else if (closestLane != -1) {
            bestLane = lane + (closestLane > lane ? 1 : -1);
        }
        
        return bestLane;
    }
};

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

bool Road::isLaneChangeSafe(float position, int currentLane, int targetLane, 
                           const vector<Vehicle>& allVehicles) const {
    if (targetLane < 0 || targetLane >= laneCount) return false;
    
    // Check for vehicles ahead in target lane
    int vehicleAheadId = findNearestVehicleAhead(position, targetLane, allVehicles);
    if (vehicleAheadId != -1) {
        const auto& vehicleAhead = allVehicles[vehicleAheadId];
        float distanceAhead = (vehicleAhead.position - position) * length;
        if (distanceAhead < LANE_CHANGE_GAP_REQUIRED) return false;
    }
    
    // Check for vehicles behind in target lane
    int vehicleBehindId = findNearestVehicleBehind(position, targetLane, allVehicles);
    if (vehicleBehindId != -1) {
        const auto& vehicleBehind = allVehicles[vehicleBehindId];
        float distanceBehind = (position - vehicleBehind.position) * length;
        
        // Allow less space behind if the vehicle is moving slower
        float requiredDistanceBehind = max(LANE_CHANGE_GAP_REQUIRED / 2, 
                                          vehicleBehind.speed * 1.0f); // 1 second of travel time
        if (distanceBehind < requiredDistanceBehind) return false;
    }
    
    return true;
}

vector<int> Road::getLanesForNextRoad(int nextRoadId, const unordered_map<int, Road>& allRoads) const {
    vector<int> appropriateLanes;
    
    // Find the next road
    auto nextRoadIt = allRoads.find(nextRoadId);
    if (nextRoadIt == allRoads.end()) return appropriateLanes;
    
    const Road& nextRoad = nextRoadIt->second;
    
    // Determine which intersection connects current road to next road
    int sharedIntersectionId = -1;
    if (int1Id == nextRoad.int1Id || int1Id == nextRoad.int2Id) {
        sharedIntersectionId = int1Id;
    } else if (int2Id == nextRoad.int1Id || int2Id == nextRoad.int2Id) {
        sharedIntersectionId = int2Id;
    }
    
    if (sharedIntersectionId == -1) return appropriateLanes; // No shared intersection
    
    // Determine turning direction (left, straight, right)
    bool isGoingToInt1 = (sharedIntersectionId == int1Id);
    bool nextRoadFromSharedInt1 = (sharedIntersectionId == nextRoad.int1Id);
    
    // Simplified lane mapping rules:
    // - Left turns: use leftmost lanes
    // - Right turns: use rightmost lanes
    // - Going straight: use middle lanes or all lanes if only 2 lanes
    
    // Calculate road direction vectors to determine turn type
    Vec2 currentDir = isGoingToInt1 ? Vec2(-direction.x, -direction.y) : direction;
    Vec2 nextDir = nextRoadFromSharedInt1 ? nextRoad.direction : Vec2(-nextRoad.direction.x, -nextRoad.direction.y);
    
    // Cross product to determine left/right turn
    float cross = currentDir.x * nextDir.y - currentDir.y * nextDir.x;
    float dot = currentDir.x * nextDir.x + currentDir.y * nextDir.y;
    
    // Assign appropriate lanes
    int numLanesToUse = min(laneCount, nextRoad.laneCount);
    
    if (cross > 0.3) {
        // Left turn - use leftmost lanes
        for (int i = 0; i < numLanesToUse; i++) {
            appropriateLanes.push_back(i);
        }
    } else if (cross < -0.3) {
        // Right turn - use rightmost lanes
        for (int i = 0; i < numLanesToUse; i++) {
            appropriateLanes.push_back(laneCount - 1 - i);
        }
    } else if (dot > 0) {
        // Roughly straight - can use all lanes or middle lanes
        if (laneCount <= 2) {
            for (int i = 0; i < laneCount; i++) {
                appropriateLanes.push_back(i);
            }
        } else {
            // Use middle lanes, proportionally distributed
            int startLane = (laneCount - numLanesToUse) / 2;
            for (int i = 0; i < numLanesToUse; i++) {
                appropriateLanes.push_back(startLane + i);
            }
        }
    } else {
        // Complicated turn or U-turn - allow all lanes for simplicity
        for (int i = 0; i < laneCount; i++) {
            appropriateLanes.push_back(i);
        }
    }
    
    return appropriateLanes;
}

void Road::sortVehiclesByPosition(const vector<Vehicle>& allVehicles) {
    for (auto& laneVehicles : vehiclesInLanes) {
        sort(laneVehicles.begin(), laneVehicles.end(), 
            [&allVehicles](int id1, int id2) {
                return allVehicles[id1].position < allVehicles[id2].position;
            });
    }
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
    // Process lane change if needed and possible
    bool processLaneChange(Vehicle& vehicle, Road& currentRoad) {
        // Update cooldown timer
        vehicle.timeSinceLastLaneChange += SIMULATION_TIME_STEP;
        
        // If already changing lanes or cooldown not expired, skip
        if (vehicle.timeSinceLastLaneChange < LANE_CHANGE_COOLDOWN) {
            return false;
        }
        
        // Check if we need to change lanes based on upcoming turn
        if (vehicle.nextRoadId != -1) {
            // When we're getting close to the intersection, consider lane changes
            float distanceToIntersection = (1.0f - vehicle.position) * currentRoad.length;
            
            if (distanceToIntersection < 200.0f && distanceToIntersection > 30.0f) {
                // Determine if we need a lane change and the target lane
                if (vehicle.needsLaneChange(currentRoad, roads)) {
                    int bestLane = vehicle.findBestTargetLane(currentRoad, roads);
                    
                    if (bestLane != vehicle.lane && abs(bestLane - vehicle.lane) == 1) {
                        // Only set target lane if it's adjacent to current lane
                        vehicle.targetLane = bestLane;
                    }
                }
            }
        } else {
            // If no next road (end of route), move to rightmost lane if adjacent
            if (vehicle.lane > 0 && vehicle.position > 0.7f) {
                vehicle.targetLane = vehicle.lane - 1;
            }
        }
        
        // If no target lane set, consider changing for speed optimization
        if (vehicle.targetLane == -1) {
            // Check for slow vehicle ahead
            int vehicleAheadId = currentRoad.findNearestVehicleAhead(vehicle.position, vehicle.lane, vehicles);
            if (vehicleAheadId != -1) {
                const auto& vehicleAhead = vehicles[vehicleAheadId];
                float distanceAhead = (vehicleAhead.position - vehicle.position) * currentRoad.length;
                
                // If vehicle ahead is slowing us down and we're not close to intersection
                if (distanceAhead < 50.0f && vehicleAhead.speed < vehicle.speed * 0.8f && 
                    (1.0f - vehicle.position) * currentRoad.length > 100.0f) {
                    
                    // Try to overtake on the left if possible (adjacent lane only)
                    if (vehicle.lane > 0 && 
                        currentRoad.isLaneChangeSafe(vehicle.position, vehicle.lane, vehicle.lane - 1, vehicles)) {
                        vehicle.targetLane = vehicle.lane - 1;
                    } 
                    // If left lane not available, try right lane (adjacent lane only)
                    else if (vehicle.lane < currentRoad.laneCount - 1 &&
                            currentRoad.isLaneChangeSafe(vehicle.position, vehicle.lane, vehicle.lane + 1, vehicles)) {
                        vehicle.targetLane = vehicle.lane + 1;
                    }
                }
            }
        }
        
        // If we have a target lane, check if safe to change
        if (vehicle.targetLane != -1 && vehicle.targetLane != vehicle.lane) {
            // Ensure target lane is adjacent to current lane
            if (abs(vehicle.targetLane - vehicle.lane) != 1) {
                vehicle.targetLane = -1;
                return false;
            }
            
            if (currentRoad.isLaneChangeSafe(vehicle.position, vehicle.lane, vehicle.targetLane, vehicles)) {
                // Execute lane change
                currentRoad.removeVehicle(vehicle.index, vehicle.lane);
                int oldLane = vehicle.lane;
                vehicle.lane = vehicle.targetLane;
                currentRoad.addVehicle(vehicle.index, vehicle.lane, vehicle.position);
                vehicle.timeSinceLastLaneChange = 0.0f;
                vehicle.targetLane = -1;
                laneChangeCount++;
                return true;
            }
        }
        
        return false;
    }

    void updateVehiclePhysics(Vehicle& vehicle, float deltaTime) {
        if (vehicle.hasArrived) return;
        
        // Get current road with O(1) lookup
        auto roadIt = roads.find(vehicle.currentRoadId);
        if (roadIt == roads.end()) return;
        
        Road& currentRoad = roadIt->second;
        float targetSpeed = currentRoad.getEffectiveSpeedLimit();
        
        // Check if approaching intersection
        float distanceToIntersection = (1.0f - vehicle.position) * currentRoad.length;
        
        if (distanceToIntersection < 50.0f) {  // Detection range
            // Get next intersection
            int nextIntersectionId = (currentRoad.int1Id == vehicle.route[vehicle.routeIndex]) ? 
                                     currentRoad.int2Id : currentRoad.int1Id;
            
            // O(1) lookup for intersection
            auto intersectionIt = intersections.find(nextIntersectionId);
            if (intersectionIt != intersections.end()) {
                // Check if light is green for this road
                if (!intersectionIt->second.isGreen(vehicle.currentRoadId)) {
                    // Red light - slow down
                    float stopDistance = max(5.0f, distanceToIntersection - 5.0f);
                    float requiredDeceleration = (vehicle.speed * vehicle.speed) / (2.0f * stopDistance);
                    
                    if (requiredDeceleration > 0.1f) {
                        targetSpeed = max(0.0f, vehicle.speed - requiredDeceleration * deltaTime);
                    } else if (distanceToIntersection < 7.0f) {
                        targetSpeed = 0.0f; // Stop at the line
                        vehicle.waitingTime += deltaTime;
                    }
                }
            }
        }
        
        // Process lane changes
        processLaneChange(vehicle, currentRoad);
        
        // Check for vehicles ahead using the optimized spatial index - O(log n) complexity
        int nearestVehicleId = currentRoad.findNearestVehicleAhead(vehicle.position, vehicle.lane, vehicles);
        if (nearestVehicleId != -1) {
            const auto& otherVehicle = vehicles[nearestVehicleId];
            float distance = (otherVehicle.position - vehicle.position) * currentRoad.length;
            float safeDistance = vehicle.speed * DRIVER_REACTION_TIME + MIN_FOLLOWING_DISTANCE;
            
            if (distance < safeDistance) {
                float requiredDeceleration = (vehicle.speed * vehicle.speed) / (2.0f * max(0.1f, distance - MIN_FOLLOWING_DISTANCE));
                requiredDeceleration = min(requiredDeceleration, MAX_DECELERATION);
                
                targetSpeed = max(0.0f, vehicle.speed - requiredDeceleration * deltaTime);
                vehicle.waitingTime += deltaTime;
            }
        }
        
        // Update speed based on acceleration/deceleration limits
        if (vehicle.speed < targetSpeed) {
            vehicle.speed = min(vehicle.speed + ACCELERATION * deltaTime, targetSpeed);
        } else if (vehicle.speed > targetSpeed) {
            vehicle.speed = max(vehicle.speed - DECELERATION * deltaTime, targetSpeed);
        }
        
        // Update position
        float oldPosition = vehicle.position;
        vehicle.position += (vehicle.speed * deltaTime) / currentRoad.length;
        
        // Calculate emissions and distance
        float distanceThisUpdate = (vehicle.position - oldPosition) * currentRoad.length;
        vehicle.distanceTraveled += distanceThisUpdate;
        vehicle.totalEmissions += (distanceThisUpdate / 1000.0f) * vehicle.emissionRate; // g/km * km
        totalEmissions += (distanceThisUpdate / 1000.0f) * vehicle.emissionRate;
        
        // Check if vehicle has reached the end of the road
        if (vehicle.position >= 1.0f) {
            handleRoadTransition(vehicle);
        }
        
        // Update travel time
        vehicle.travelTime += deltaTime;
    }

//=======================================
// SECTION 4: Road and Intersection Management
//=======================================

    void handleRoadTransition(Vehicle& vehicle) {
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
    void runSimulation() {
        cout << "Starting simulation..." << endl;
        
        int totalSteps = static_cast<int>(SIMULATION_DURATION / SIMULATION_TIME_STEP);
        int reportInterval = totalSteps / 10;  // Report progress 10 times
        
        for (int step = 0; step < totalSteps; ++step) {
            simulationTime += SIMULATION_TIME_STEP;
            
            // Update traffic lights
            updateTrafficLights(SIMULATION_TIME_STEP);
            
            // Sort vehicles in each lane by position for optimized lookups
            // Do this less frequently to save processing time
            if (step % 10 == 0) {  // Every second (10 * 0.1s)
                for (auto& [id, road] : roads) {
                    road.sortVehiclesByPosition(vehicles);
                }
            }
            
            // Update vehicle positions with the optimized physics calculation
            for (auto& vehicle : vehicles) {
                if (!vehicle.hasArrived) {
                    updateVehiclePhysics(vehicle, SIMULATION_TIME_STEP);
                }
            }
            
            // Report progress
            if (step % reportInterval == 0 || step == totalSteps - 1) {
                float percentComplete = 100.0f * step / totalSteps;
                cout << "Simulation " << percentComplete << "% complete. ";
                cout << vehiclesCompleted << "/" << vehicles.size() << " vehicles arrived." << endl;
                cout << "Lane changes so far: " << laneChangeCount << endl;
            }
            
            // Check if all vehicles have completed their routes
            if (vehiclesCompleted == vehicles.size()) {
                cout << "All vehicles have reached their destinations." << endl;
                cout << "Simulation ended after " << step * SIMULATION_TIME_STEP << " seconds of simulated time." << endl;
                break;
            }
        }
        
        // Final report
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
        cout << "Average passenger-weighted travel time: " << avgPassengerTime << " seconds" << endl;
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
