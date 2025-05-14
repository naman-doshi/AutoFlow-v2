from NewVersion.LandscapeComponents import * 
from NewVersion.AutoFlow import *
import random
import time
import json
import os

toTest = 1000

landscape = Landscape(1000, 
                      1000,
                      gridSparseness=0.4,
                      gridCoverage=0.6)
#landscape.generate()
#landscape.store("NewVersion/benchmark.txt")
landscape.load("NewVersion/benchmark.txt")

landscape.show()
plt.show()

print("Landscape loaded")

allStartingPositions = []
allEndingPositions = []
autoFlowVehicles = []

for road in landscape.roads:
    positionObject = [[road] + i for i in road.availablePositions()]
    allStartingPositions += positionObject
    allEndingPositions += positionObject

# start timer
start = time.time()

for i in range(toTest):
    vehicle = ConventionalVehicle(i)
    vehicle.setRoutingSystem(1)
    autoFlowVehicles.append(vehicle)
    startPos = random.choice(allStartingPositions)
    # print(len(startPos))
    vehicle.startingActualRoad = startPos[0]
    vehicle.startingRoadId = startPos[0].id
    allStartingPositions.remove(startPos)
    vehicle.starting = startPos[1:]
    
    endPos = random.choice(allEndingPositions)
    vehicle.endingActualRoad = endPos[0]
    vehicle.endingRoadId = endPos[0].id
    allEndingPositions.remove(endPos)
    vehicle.ending = endPos[1:]
    # Add emission rate and passenger count for simulation metrics
    vehicle.emissionRate = random.uniform(1.0, 2.0)  # g/km of CO2
    vehicle.passengerCount = random.randint(1, 5)

end = time.time()
print("Time taken to generate vehicles: ", end - start)

allRoutes2 = computeRoutes([], autoFlowVehicles, landscape)
print("Time taken to compute routes: ", time.time() - end)

sim_data = {
    "roads": [
        {
            "id": road.id,
            "length": road.length,
            "speed_limit": road.speedLimit,
            "capacity": road.capacity,
            "lane_count": road.laneCount,
            "int1_id": road.int1.id if road.int1 else -1,
            "int2_id": road.int2.id if road.int2 else -1
        } for road in landscape.roads
    ],
    "intersections": [
        {
            "id": intersection.id,
            "x": intersection.x,
            "y": intersection.y,
            "traffic_light_duration": intersection.trafficLightDuration,
            "connecting_roads": [r.id for r in intersection.connectingRoads],
            "road_count": intersection.roadCount  
        } for intersection in landscape.intersections.values()
    ],
    "vehicles": [
        {
            "id": vehicle.id,
            "starting_road": vehicle.startingRoadId,
            "ending_road": vehicle.endingRoadId,
            "emission_rate": vehicle.emissionRate,
            "passenger_count": vehicle.passengerCount,
            "route": [intersection.id for intersection in allRoutes2.get(vehicle.id, [])]
        } for vehicle in autoFlowVehicles
    ]
}

with open("simulation_data.json", "w") as f:
    json.dump(sim_data, f)

print(f"Saved simulation data for {len(autoFlowVehicles)} vehicles to simulation_data.json")

subprocess.run(["/Users/namandoshi/Desktop/AutoFlow-v2/TrafficSimulatorV2"])

# delete vehicle_road_speeds.csv
os.remove("vehicle_road_speeds.csv")



# OPTIONAL (storing visualisation)
#landscape.storeImage("landscape.png")


# coords = []
# for route in allRoutes.values():
#     coords.append([i.coordinates() for i in route])

# for route in coords:
#     for i in range(len(route)-1):
#         plt.plot([route[i][0], route[i+1][0]], [route[i][1], route[i+1][1]], 'b')


