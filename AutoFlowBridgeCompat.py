# ================ IMPORTS ================
from AutoFlow import *

# from TestHelper import *

from copy import deepcopy

# =========================================


# Literals
COMMERCIAL_BLOCK = LandPlotDescriptor((2, 2), (2, 2), False)  # 2x2 land blocks
COMMERCIAL_BLOCK_LARGE = LandPlotDescriptor((3, 3), (3, 3), False)  # 3x3 land blocks
HORIZONTAL_RESIDENTIAL_ROW = LandPlotDescriptor(
    (5, 8), (1, 1), False
)  # north-facing horizontal residential rows of 5-8 continuous land blocks
VERTICAL_RESIDENTIAL_ROW = LandPlotDescriptor(
    (1, 1), (5, 8), False
)  # east-facing vertical residential rows of 5-8 continuous land blocks
SCHOOL_ZONE = LandPlotDescriptor((4, 4), (3, 3))  # randomly oriented 4x3 school zones
LARGE_PARK_AREA = LandPlotDescriptor(
    (4, 6), (4, 6)
)  # randomly oriented (4-6)x(4-6) park area


# ================ INPUTS =================
LANDSCAPE_SIZE = (15, 15)
LANDSCAPE_FEATURES = [
    #(COMMERCIAL_BLOCK, 15),
    (COMMERCIAL_BLOCK_LARGE, 1),
    #(LandPlotDescriptor((2, 2), (1, 1)), 20)
    # (HORIZONTAL_RESIDENTIAL_ROW, 3),
    # (VERTICAL_RESIDENTIAL_ROW, 4),
    # (SCHOOL_ZONE, 2),
    (LARGE_PARK_AREA, 1)
]
#LANDSCAPE_FILLER = LandPlotDescriptor((1, 1), (1, 1), False)  # 1x1 land block fillers
#LANDSCAPE_FILLER = LandPlotDescriptor((2, 2), (1, 1))  # 2x1 randomly oriented land block fillers
LANDSCAPE_FILLER = LandPlotDescriptor((2, 2), (2, 2), False)  # 2x2 land block fillers
# VEHICLE_COUNT = 20 # size constraint in place, may not always fit
# =========================================


# ===============================================================================================
# Landscape Generation
# ===============================================================================================

# Generate virtual landscape
landscape = Landscape(*LANDSCAPE_SIZE)
landscape.generate_new_landscape(
    desiredFeatures=LANDSCAPE_FEATURES, filler=LANDSCAPE_FILLER
)
# landscape.generate_new_landscape()

# There must be at least one road within the map area
assert len(landscape.intersections) > 4

# # Check for overlapping roads
# for roadA in landscape.roads:
#     for roadB in landscape.roads:
#         if roadA != roadB:
#             if doIntersect(
#                 Point(*roadA.startPosReal),
#                 Point(*roadA.endPosReal),
#                 Point(*roadB.startPosReal),
#                 Point(*roadB.endPosReal)
#             ):
#                 print(roadA.startPosReal, roadA.endPosReal)
#                 print(roadB.startPosReal, roadB.endPosReal)
#                 raise Exception("Roads overlap, this should not occur")

# Compute average road speed for all roads within map area
road_speeds = 0
road_count = 0
for road in landscape.roads:
    if (
        1 <= road.start[0] <= landscape.xSize and 1 <= road.start[1] <= landscape.ySize
    ) or (1 <= road.end[0] <= landscape.xSize and 1 <= road.end[1] <= landscape.ySize):
        road_speeds += road.speedLimit
        road_count += 1

AVERAGE_ROAD_SPEED = road_speeds / road_count
AVERAGE_ROAD_SPEED_MPS = AVERAGE_ROAD_SPEED * 1000 / 3600

# Compute max road speed for all roads within map area
MAX_ROAD_SPEED = 0
for road in landscape.roads:
    if (
        1 <= road.start[0] <= landscape.xSize and 1 <= road.start[1] <= landscape.ySize
    ) or (1 <= road.end[0] <= landscape.xSize and 1 <= road.end[1] <= landscape.ySize):
        if road.speedLimit > MAX_ROAD_SPEED:
            MAX_ROAD_SPEED = road.speedLimit

MAX_ROAD_SPEED_MPS = MAX_ROAD_SPEED * 1000 / 3600


# ===============================================================================================
# Spawn Vehicle Agents
# ===============================================================================================


def getCarPositions(road: Road):
    positions = []
    pos = 0
    if road.cellSpan == 0:
        return []
    else:
        increment = 1 / (road.cellSpan * 4)
    while pos < 1:
        positions.append(pos)
        pos += increment
    # positions.append(1)
    # print(positions)
    return positions

def getBusPositions(road: Road, start: bool):
    positions = []
    if road.cellSpan == 0:
        return []
    else:
        increment = 1 / (road.cellSpan * 4)


    if start:
        available = road.available_starting_positions
    else:
        return [(road, i) for i in road.available_ending_positions]

    for position in available:
        second = position + increment
        if (position in available) and (second in available):
            positions.append(position)
    
    return [(road, i) for i in positions]



# # Create two pools of available starting coordinates (as every road segment has a pair of opposite roads)
# available_starting_coordinates: list[tuple[Road, float]] = []
# for road in landscape.roads:
#     for pos in getPositions(road):
#         realpos = getRealPositionOnRoad(road, pos)
#         if (
#             CELL_SIZE_METRES <= realpos[0] <= CELL_SIZE_METRES * landscape.xSize // 2
#         ) and (CELL_SIZE_METRES <= realpos[1] <= CELL_SIZE_METRES * (landscape.ySize+1)):
#             available_starting_coordinates.append((road, pos))

# # Create two pools of available destination coordinates (as every road segment has a pair of opposite roads)
# available_destination_coordinates: list[tuple[Road, float]] = []
# for road in landscape.roads:
#     for pos in getPositions(road):
#         realpos = getRealPositionOnRoad(road, pos)
#         if (
#             CELL_SIZE_METRES * landscape.xSize // 2 <= realpos[0] <= CELL_SIZE_METRES * (landscape.xSize+1)
#         ) and (CELL_SIZE_METRES <= realpos[1] <= CELL_SIZE_METRES * (landscape.ySize+1)):
#             available_destination_coordinates.append((road, pos))

# # Create two pools of available starting coordinates (as every road segment has a pair of opposite roads)
# available_starting_coordinates: list[tuple[Road, float]] = []
# for road in landscape.roads:
#     for pos in getPositions(road):
#         realpos = getRealPositionOnRoad(road, pos)
#         if (
#             CELL_SIZE_METRES <= realpos[0] <= CELL_SIZE_METRES * landscape.xSize // 2
#         ) and (CELL_SIZE_METRES <= realpos[1] <= CELL_SIZE_METRES * (landscape.ySize+1)):
#             available_starting_coordinates.append((road, pos))

# # Create two pools of available destination coordinates (as every road segment has a pair of opposite roads)
# available_destination_coordinates: list[tuple[Road, float]] = []
# for road in landscape.roads:
#     for pos in getPositions(road):
#         realpos = getRealPositionOnRoad(road, pos)
#         if (
#             CELL_SIZE_METRES <= realpos[0] <= CELL_SIZE_METRES * (landscape.xSize+1)
#         ) and (CELL_SIZE_METRES <= realpos[1] <= CELL_SIZE_METRES * (landscape.ySize+1)):
#             available_destination_coordinates.append((road, pos))

available_starting_coordinates: list[tuple[Road, float]] = []
available_destination_coordinates: list[tuple[Road, float]] = []
for road in landscape.roads:
    for pos in getCarPositions(road):
        realpos = getRealPositionOnRoad(road, pos)
        if (CELL_SIZE_METRES <= realpos[0] <= CELL_SIZE_METRES * (landscape.xSize+1)) and (CELL_SIZE_METRES <= realpos[1] <= CELL_SIZE_METRES * (landscape.ySize+1)):
            available_starting_coordinates.append((road, pos))
            available_destination_coordinates.append((road, pos))
            road.available_starting_positions.append(pos)
            road.available_ending_positions.append(pos)


# Generate a valid vehicle count
MAX_VEHICLE_COUNT = min(len(available_starting_coordinates), len(available_destination_coordinates))
print()
print("Number of starting and ending coordinates:", len(available_starting_coordinates))
print()
#VEHICLE_COUNT = randint(int(MAX_VEHICLE_COUNT * 9 / 10), MAX_VEHICLE_COUNT)
#VEHICLE_COUNT = randint(int(MAX_VEHICLE_COUNT * 6 / 10), int(MAX_VEHICLE_COUNT * 7 / 10)) # auto-generated
vehicle_density = float(input("Enter vehicle density as a percentage (0-100): "))
print()
VEHICLE_COUNT = int(MAX_VEHICLE_COUNT * vehicle_density / 100)

# Check that the vehicle count does not exceed the maximum allowed vehicle count
assert VEHICLE_COUNT <= MAX_VEHICLE_COUNT, "Vehicle count exceeds maximum allowed vehicle count"

# Array storing all vehicle agents
vehicles: list[Vehicle] = []

# Determine EV distribution
EV_percentage = randint(10, 20)  # based on real world data
EV_COUNT = EV_percentage * VEHICLE_COUNT // 100  # number of EVs to spawn

# Bus distribution (realistically, around 5% that of cars)
BUS_COUNT = VEHICLE_COUNT // 20

current_index = 0

# Spawn EVs
while current_index < EV_COUNT:
    # if current_index in marked_indexes:
    #     vehicle = ElectricVehicle(useAutoFlow=True)
    #     autoflow_vehicles.append(vehicle)
    # else:
    #     vehicle = ElectricVehicle(useAutoFlow=False)
    #     selfish_vehicles.append(vehicle)
    vehicle = ElectricVehicle(current_index)
    vehicles.append(vehicle)
    current_index += 1

# Spawn conventional vehicles
while current_index < VEHICLE_COUNT:
    # if current_index in marked_indexes:
    #     vehicle = ConventionalVehicle(useAutoFlow=True)
    #     autoflow_vehicles.append(vehicle)
    # else:
    #     vehicle = ConventionalVehicle(useAutoFlow=False)
    #     selfish_vehicles.append(vehicle)
    vehicle = ConventionalVehicle(current_index)
    vehicles.append(vehicle)
    current_index += 1

# Assign random starting coordinates to all vehicles
for vehicle in vehicles:
    coordIndex = randint(0, len(available_starting_coordinates) - 1)  # select random location from pool
    road, position = available_starting_coordinates[coordIndex]  # unpack location into road and position
    vehicle.setLocation(road, position)  # Set vehicle's starting location
    available_starting_coordinates.pop(coordIndex)  # Remove assigned position from pool
    road.available_starting_positions.remove(position)


# Assign buses to the remaining spots
buses = []
while current_index < VEHICLE_COUNT + BUS_COUNT:
    vehicle = Bus(current_index)
    buses.append(vehicle)
    current_index += 1

i = 0
placedBuses = []
for road in landscape.roads:
    positions = getBusPositions(road, True)
    for position in positions:
        position = float(position[1])
        if (i == len(buses)):
            break

        increment = 1 / (road.cellSpan * 4)

        bus = buses[i]
        bus.setLocation(road, position)

        placedBuses.append(buses[i])

        road.available_starting_positions.remove(position)

        if position + increment in road.available_starting_positions:
            road.available_starting_positions.remove(position + increment)

        if position + increment in positions:
            break

        i += 1
    
    if (i == len(buses)):
            break

vehicles.extend(placedBuses)

TOTAL_VEHICLE_COUNT = len(vehicles)

print(len(placedBuses), "buses placed.")

for road in landscape.roads:  # sort vehicle stacks, cars at the front are at the front/start of the deque
    road.vehicleStack = deque(
        sorted(road.vehicleStack, key=lambda vehicle: vehicle.position, reverse=True)
    )

# # Check that no vehicle spawns in overlapping positions
# seen = set()
# for vehicle in vehicles:
#     start = (vehicle.road.start, vehicle.road.end, vehicle.position)
#     if start in seen:
#         raise Exception("Overlapping vehicle positions")
#     seen.add(start)

print("Number of total vehicles:", len(vehicles))
print()

vehicleLookup = {}

# Assign random destination coordinates to all vehicles
for vehicle in vehicles:
    coordIndex = randint(0, len(available_destination_coordinates) - 1)  # select random location from pool
    road, position = available_destination_coordinates[coordIndex]  # unpack location into road and position
    vehicle.setDestination(road, position)  # Set vehicle's destination location
    available_destination_coordinates.pop(coordIndex)  # Remove assigned position from pool
    road.available_ending_positions.remove(position)

# # Checking for existence of double intersections
# di_exists = False
# for y in range(1, landscape.ySize + 1):
#     for x in range(1, landscape.xSize + 1):
#         if (
#             landscape.landscapeMatrix[y][x]
#             == landscape.landscapeMatrix[y + 1][x]
#             == "IS"
#         ):
#             di_exists = True
#             break
#         elif (
#             landscape.landscapeMatrix[y][x]
#             == landscape.landscapeMatrix[y - 1][x]
#             == "IS"
#         ):
#             di_exists = True
#             break
#         elif (
#             landscape.landscapeMatrix[y][x]
#             == landscape.landscapeMatrix[y][x + 1]
#             == "IS"
#         ):
#             di_exists = True
#             break
#         elif (
#             landscape.landscapeMatrix[y][x]
#             == landscape.landscapeMatrix[y][x - 1]
#             == "IS"
#         ):
#             di_exists = True
#             break
#     if di_exists:
#         break
# if di_exists:
#     print("Double intersection detected")

# ===============================================================================================
# AutoFlow Distribution Functions
# ===============================================================================================


def modify_population(
    original_vehicle_population: list[Vehicle], autoflow_percentage: float
):
    """
    Returns a new population of vehicle agents with a specified percentage having the AutoFlow routing system.
    A tuple of two lists of vehicles are returned: (selfish_vehicles, autoflow_vehicles).
    """

    # Determine AutoFlow distribution
    numAutoFlow = int(autoflow_percentage * TOTAL_VEHICLE_COUNT // 100)  # number of AutoFlow vehicles to spawn
    marked_indexes = sample(range(TOTAL_VEHICLE_COUNT), numAutoFlow)  # indexes of AutoFlow vehicles
    population = deepcopy(original_vehicle_population)

    # Distribute AutoFlow & categorise vehicles
    for i in range(TOTAL_VEHICLE_COUNT):
        if i in marked_indexes:
            population[i].setRoutingSystem(1)  # switch to AutoFlow
        else:
            population[i].setRoutingSystem(0)

    return population

landscape.precomputeUnityCache()
allVehicles = vehicles

def outputToBridge(autoflowPercentage : float) -> tuple[
    dict[int, tuple[float, float, Vehicle]],
    Landscape,
    dict[int, list[tuple[float, float, float]]],
    list[Vehicle],
]:

        vehicles = modify_population(allVehicles, autoflowPercentage)

        selfish_vehicles = []
        autoflow_vehicles = []

        for vehicle in vehicles:
            if vehicle.routingSystem == "Autoflow":
                autoflow_vehicles.append(vehicle)
            else:
                selfish_vehicles.append(vehicle)

        print(len(autoflow_vehicles), "AutoFlow vehicles")
        print(len(selfish_vehicles), "selfish vehicles")

        routes = computeRoutes(
            selfish_vehicles, autoflow_vehicles, landscape, MAX_ROAD_SPEED_MPS
        )

        routes = deepcopy(routes)


        initPos: dict[int, tuple[float, float, Vehicle]] = {}
        for i, vehicle in enumerate(vehicles):
            pos = getRealPositionOnRoad(vehicle.road, vehicle.position)
            initPos[i] = (pos[0], pos[1], vehicle)

        routes2: dict[int, list[tuple[float, float, float]]] = {}
        for id, route in routes.items():
            vehicle = initPos[id][2]
            routes2[vehicle.id] = [(node[0][0], node[0][1], node[1]) for node in route]

        # print(routes2)

        return (initPos, landscape, routes2, vehicles)
