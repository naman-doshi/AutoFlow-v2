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
const int SIMULATION_DURATION = 360000;      // seconds (1 hour)

// Simple vector class for 2D coordinates
struct Vec2 {
    float x, y;
    
    Vec2(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}
    
    float distance(const Vec2& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
    
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    Vec2 normalized() const {
        float len = sqrt(x*x + y*y);
        if (len > 0.001f) {
            return Vec2(x / len, y / len);
        }
        return *this;
    }
};

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

// Road class
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
    
    Road() : id(-1), length(0), speedLimit(0), capacity(0), laneCount(1), 
             int1Id(-1), int2Id(-1) {}
             
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
};

// Vehicle class
struct Vehicle {
    int id;
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
    
    // Simulation metrics
    float travelTime = 0.0f;
    float totalEmissions = 0.0f;
    float distanceTraveled = 0.0f;
    bool hasArrived = false;
    float waitingTime = 0.0f;
    
    Vec2 getPosition(const Road& road) const {
        return road.startPos + road.direction * (road.length * position);
    }
};

//===========================
// SECTION 2: File I/O
//===========================

class TrafficSimulation {
private:
    unordered_map<int, Intersection> intersections;
    unordered_map<int, Road> roads;
    vector<Vehicle> vehicles;
    float simulationTime = 0.0f;
    
    // Metrics
    float totalEmissions = 0.0f;
    float totalTravelTime = 0.0f;
    float totalPassengerTime = 0.0f;
    int vehiclesCompleted = 0;
    
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
                
                roads[road.id] = road;
            }
            
            // Load vehicles
            for (const auto& vehicleData : data["vehicles"]) {
                Vehicle vehicle;
                vehicle.id = vehicleData["id"];
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
                    vehicle.lane = rand() % roads[vehicle.currentRoadId].laneCount;
                    
                    // Find next road in route
                    if (vehicle.route.size() > 1) {
                        int currInt = vehicle.route[0];
                        int nextInt = vehicle.route[1];
                        
                        // Find road connecting these intersections
                        for (const auto& [id, road] : roads) {
                            if ((road.int1Id == currInt && road.int2Id == nextInt) || 
                                (road.int2Id == currInt && road.int1Id == nextInt)) {
                                vehicle.nextRoadId = id;
                                break;
                            }
                        }
                    } else {
                        // Route only has one intersection - vehicle is already at destination
                        vehicle.nextRoadId = -1;
                    }
                }
                
                vehicles.push_back(vehicle);
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
    void updateVehiclePhysics(Vehicle& vehicle, float deltaTime) {
        if (vehicle.hasArrived) return;
        
        // Get current road
        auto roadIt = roads.find(vehicle.currentRoadId);
        if (roadIt == roads.end()) return;
        
        Road& currentRoad = roadIt->second;
        float targetSpeed = currentRoad.getEffectiveSpeedLimit();
        
        // Check if approaching intersection
        bool approachingIntersection = false;
        float distanceToIntersection = (1.0f - vehicle.position) * currentRoad.length;
        
        if (distanceToIntersection < 50.0f) {  // Detection range
            approachingIntersection = true;
            
            // Get next intersection
            int nextIntersectionId = (currentRoad.int1Id == vehicle.route[vehicle.routeIndex]) ? 
                                     currentRoad.int2Id : currentRoad.int1Id;
            
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
        
        // Check for vehicles ahead (basic car-following model)
        float minFollowingDistance = 0.0f;
        for (const auto& otherVehicle : vehicles) {
            if (otherVehicle.id != vehicle.id && 
                otherVehicle.currentRoadId == vehicle.currentRoadId &&
                otherVehicle.lane == vehicle.lane &&
                otherVehicle.position > vehicle.position) {
                
                float distance = (otherVehicle.position - vehicle.position) * currentRoad.length;
                float safeDistance = vehicle.speed * DRIVER_REACTION_TIME + MIN_FOLLOWING_DISTANCE;
                
                if (distance < safeDistance) {
                    float requiredDeceleration = (vehicle.speed * vehicle.speed) / (2.0f * max(0.1f, distance - MIN_FOLLOWING_DISTANCE));
                    requiredDeceleration = min(requiredDeceleration, MAX_DECELERATION);
                    
                    targetSpeed = max(0.0f, vehicle.speed - requiredDeceleration * deltaTime);
                    vehicle.waitingTime += deltaTime;
                    break;
                }
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
            
            // Remove vehicle from current road
            auto roadIt = roads.find(vehicle.currentRoadId);
            if (roadIt != roads.end()) {
                roadIt->second.vehicleCount--;
            }
            
            return;
        }
        
        // Move to next road
        auto currentRoadIt = roads.find(vehicle.currentRoadId);
        if (currentRoadIt != roads.end()) {
            currentRoadIt->second.vehicleCount--;
        }
        
        vehicle.currentRoadId = vehicle.nextRoadId;
        vehicle.position = 0.0f;  // Start of the new road
        
        // Assign a lane
        auto nextRoadIt = roads.find(vehicle.currentRoadId);
        if (nextRoadIt != roads.end()) {
            vehicle.lane = rand() % nextRoadIt->second.laneCount;
            nextRoadIt->second.vehicleCount++;
        }
        
        // Determine next road in route
        if (vehicle.routeIndex + 1 < vehicle.route.size()) {
            int currInt = vehicle.route[vehicle.routeIndex];
            int nextInt = vehicle.route[vehicle.routeIndex + 1];
            
            // Find road connecting these intersections
            for (const auto& [id, road] : roads) {
                if ((road.int1Id == currInt && road.int2Id == nextInt) || 
                    (road.int2Id == currInt && road.int1Id == nextInt)) {
                    vehicle.nextRoadId = id;
                    break;
                }
            }
        } else {
            vehicle.nextRoadId = -1;  // No next road
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
            
            // Update congestion on roads
            updateRoadCongestion();
            
            // Update vehicle positions
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
// SECTION 7: Congestion Modeling
//===================================

private:
    void updateRoadCongestion() {
        // Reset vehicle counts for accuracy
        for (auto& [id, road] : roads) {
            road.vehicleCount = 0;
        }
        
        // Count vehicles on each road
        for (const auto& vehicle : vehicles) {
            if (!vehicle.hasArrived) {
                auto roadIt = roads.find(vehicle.currentRoadId);
                if (roadIt != roads.end()) {
                    roadIt->second.vehicleCount++;
                }
            }
        }
        
        // Update congestion factors
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
