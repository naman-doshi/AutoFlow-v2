from NewVersion.LandscapeComponents import * 
from NewVersion.AutoFlow import *
import random
import time

toTest = 5

landscape = Landscape(500, 
                      500,
                      gridSparseness=0.4,
                      gridCoverage=0.6)
landscape.generate()
landscape.load("NewVersion/benchmark.txt")

print("Landscape loaded")

allStartingPositions = []
allEndingPositions = []
autoFlowVehicles = []

for road in landscape.roads:
    allStartingPositions += [road.id] + road.availablePositions()
    allEndingPositions += [road.id] + road.availablePositions()

# start timer
start = time.time()

for i in range(toTest):
    vehicle = ConventionalVehicle(i)
    vehicle.setRoutingSystem(1)
    autoFlowVehicles.append(vehicle)
    startPos = random.choice(allStartingPositions)
    vehicle.startingRoadId = startPos[0]
    allStartingPositions.remove(startPos)
    vehicle.starting = startPos[1:]
    endPos = random.choice(allEndingPositions)
    vehicle.endingRoadId = endPos[0]
    allEndingPositions.remove(endPos)
    vehicle.ending = endPos[1:]

end = time.time()
print("Time taken to generate vehicles: ", end - start)

allRoutes = computeRoutes([], autoFlowVehicles, landscape)
print("Time taken to compute routes: ", time.time() - end)

landscape.show()

# OPTIONAL (storing visualisation)
#landscape.storeImage("landscape.png")


# coords = []
# for route in allRoutes.values():
#     coords.append([i.coordinates() for i in route])

# for route in coords:
#     for i in range(len(route)-1):
#         plt.plot([route[i][0], route[i+1][0]], [route[i][1], route[i+1][1]], 'b')


# plt.show()