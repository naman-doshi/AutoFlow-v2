from NewVersion.LandscapeComponents import * 
from NewVersion.AutoFlow import *
import random

# ========================================= LANDSCAPE GENERATION =========================================

# these are the optimal, recommended settings for testing
landscape = Landscape(3000, 
                      3000,
                      gridSparseness=0.4,
                      gridCoverage=0.6)
landscape.generate()

# # OPTIONAL (saving to a file)
# landscape.store("benchmark.txt")

# OPTIONAL (loading from a file)
# landscape.load("NewVersion/sydney.txt")

# populate the possible starting positions
allStartingPositions = []
allEndingPositions = []
for road in landscape.roads:
    positionObject = [[road] + i for i in road.availablePositions()]
    allStartingPositions += positionObject
    allEndingPositions += positionObject

# ========================================= VEHICLE GENERATION =========================================
autoFlowVehicleCount = 5
selfishVehicleCount = 0

totalVehicleCount = autoFlowVehicleCount + selfishVehicleCount
autoFlowVehicles = []
selfishVehicles = []

for i in range(totalVehicleCount):
    
    # 50% chance of conventional, 30% chance of electric, 20% chance of bus
    chance = uniform(0, 1)
    if chance <= 0.5:
        vehicle = ConventionalVehicle(i)
    elif chance <= 0.8:
        vehicle = ElectricVehicle(i)
    else:
        vehicle = Bus(i)
    
    # set routing system
    if i < autoFlowVehicleCount:
        vehicle.setRoutingSystem(1)
        autoFlowVehicles.append(vehicle)
    else:
        vehicle.setRoutingSystem(0)
        selfishVehicles.append(vehicle)

    # set starting and ending positions
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

# # ========================================= ROUTE COMPUTATION =========================================

allRoutes = computeRoutes(selfishVehicles, autoFlowVehicles, landscape)

# # ========================================= DISPLAY (optional) =========================================


landscape.show()

# OPTIONAL (storing visualisation)
#landscape.storeImage("landscape.png")


coords = []
for route in allRoutes.values():
    coords.append([i.coordinates() for i in route])

for route in coords:
    for i in range(len(route)-1):
        plt.plot([route[i][0], route[i+1][0]], [route[i][1], route[i+1][1]], 'b')


plt.show()





