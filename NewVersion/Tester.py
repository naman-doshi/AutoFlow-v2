from LandscapeComponents import * 
from AutoFlow import *
import random

# ========================================= LANDSCAPE GENERATION =========================================

# these are the optimal, recommended settings for testing
landscape = Landscape(1000, 
                      1000,
                      gridSparseness=0.4,
                      gridCoverage=0.6)
landscape.generate()

# OPTIONAL (saving to a file)
landscape.store("landscape.txt")

# OPTIONAL (loading from a file)
landscape.load("landscape.txt")

# populate the possible starting positions
allStartingPositions = []
allEndingPositions = []
for road in landscape.roads:
    allStartingPositions += road.availablePositions()
    allEndingPositions += road.availablePositions()

# ========================================= VEHICLE GENERATION =========================================
autoFlowVehicleCount = 5
selfishVehicleCount = 5

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
    allStartingPositions.remove(startPos)
    vehicle.setLocation(startPos)

    endPos = random.choice(allEndingPositions)
    allEndingPositions.remove(endPos)
    vehicle.setDestination(endPos)

# ========================================= ROUTE COMPUTATION =========================================

allRoutes = computeRoutes(selfishVehicles, autoFlowVehicles, landscape)

# ========================================= DISPLAY (optional) =========================================

landscape.show()

# OPTIONAL (storing visualisation
landscape.storeImage("landscape.png")


coords = []
for route in allRoutes.values():
    coords.append([i.coordinates() for i in route])

for route in coords:
    for i in range(len(route)-1):
        plt.plot([route[i][0], route[i+1][0]], [route[i][1], route[i+1][1]], 'b')


plt.show()





