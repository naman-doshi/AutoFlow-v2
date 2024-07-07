# ideas
# fix emission calculation
# tweak sorting

"""
This script contains the multiple route computing algorithms, including:
- selfish A* routing algorithm
- AutoFlow collaborative routing algorithm
"""


#================ IMPORTS ================
from LandscapeComponents import *
from VehicleAgents import *

from random import sample
from heapq import *
from math import ceil
import random
#=========================================

# ===============================================================================================
# Helper Functions
# ===============================================================================================

def getRealPositionOnRoad(road: Road, position: float) -> tuple[float, float]:
    """
    Calculates the real 2D position given road and a normalised position.
    """
    if road.direction == "N":
        return (road.startPosReal[0], road.startPosReal[1] + road.length * position)
    elif road.direction == "S":
        return (road.startPosReal[0], road.startPosReal[1] - road.length * position)
    elif road.direction == "E":
        return (road.startPosReal[0] + road.length * position, road.startPosReal[1])
    elif road.direction == "W":
        return (road.startPosReal[0] - road.length * position, road.startPosReal[1])
    
def euclideanDistance(pos1: tuple[float, float], pos2: tuple[float, float]) -> float:
    """
    Calculates the euclidean distance between two positions.
    The unit of measurement is metres.
    """
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

def manhattanDistance(pos1: tuple[float, float], pos2: tuple[float, float]) -> float:
    """
    Calculates the manhattan distance between two positions.
    The unit of measurement is metres.
    """
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


# ===============================================================================================
# Main Functions
# ===============================================================================================

def computeRoutes(selfish_vehicles: list[Vehicle], autoflow_vehicles: list[Vehicle], landscape: Landscape, MAX_ROAD_SPEED_MPS: float, carPositions = {}):
    """
    Compute the routes for selfish vehicles first, then AutoFlow vehicles.
    """
    routes = {}
    selfish_vehicle_routes = computeSelfishVehicleRoutes(selfish_vehicles, landscape, MAX_ROAD_SPEED_MPS, carPositions=carPositions)
    autoflow_vehicle_routes = computeAutoflowVehicleRoutes(autoflow_vehicles, landscape, MAX_ROAD_SPEED_MPS, carPositions=carPositions)

    for i in range(len(autoflow_vehicle_routes)):
        routes[autoflow_vehicles[i].id] = autoflow_vehicle_routes[i]
    
    for i in range(len(selfish_vehicle_routes)):
        routes[selfish_vehicles[i].id] = selfish_vehicle_routes[i]

    return routes

def computeSelfishVehicleRoutes(selfish_vehicles: list[Vehicle], landscape: Landscape, MAX_ROAD_SPEED_MPS: float, carPositions={}) -> list[list[tuple[float, float]]]:
    """
    Selfish routing algorithm of Google Maps.
    Vehicles are not knowledgeable of future traffic and therefore only aware of congestion AFTER they occur.

    Each node is a tuple that stores (fcost, hcost, gcost, tiebreaker, road, position).
    - fcost: sum of gcost and hcost, node with lowest fcost will be evaluated first
    - hcost: optimistic approximate time required to reach destination using MAX_ROAD_SPEED
    - gcost: cost so far i.e. time taken so far, represents the ABSOLUTE time
    - tiebreaker: an unique integer used as a tiebreaker when all costs are equal

    Every node pushed into the Open list will be the start of a road (or the starting position of the vehicle).
    The Closed list contains all visited nodes (including end points of a road as well as the starting position).
    """

    routes: dict[int, list[tuple[tuple[float, float], int]]] = {}

    for vehicle in selfish_vehicles:

        tiebreaker = 0 # tiebreaker value for when all costs are equal

        # Hashmap that maps each node to their fcost
        node_fcost: dict[tuple[int, int], float] = defaultdict(lambda: float("inf"))

        # Hashmap that stores the previous node's roadID and normalised position of each node
        previous_node: dict[tuple[int, float], tuple[int, int]] = {}        
        # previous_node[(roadID, normalised position)] => (roadID, normalised position)
        # NOTE: normalised position is used to handle roads where startPosReal and endPosReal are equal
        
        # Calculate real destination position
        destination_position = getRealPositionOnRoad(vehicle.destinationRoad, vehicle.destinationPosition)

        # Nodes are the starting points of each road, can also be the starting point of the vehicle
        open_nodes: list[float, float, float, Road, float] = [] # Open is a priority queue
        closed_nodes: set[tuple(Road, float)] = set() # Closed can just be a set

        # Calculate the cost variables of the starting position
        gcost = 0
        hcost = manhattanDistance(
            getRealPositionOnRoad(vehicle.road, vehicle.position), 
            destination_position
        ) / MAX_ROAD_SPEED_MPS
        fcost = gcost + hcost

        # Add starting position of the vehicle to open_nodes
        start_node = (fcost, hcost, gcost, tiebreaker, vehicle.road, vehicle.position)
        tiebreaker += 1
        heappush(open_nodes, start_node) 

        broken = False

        while True: # loop until target point has been reached

            if len(open_nodes) == 0:
                if carPositions == {}:
                    raise Exception("Path does not exist")
                else:
                    broken = True
                    route = carPositions[vehicle.id]["Routes"]
                    route = [((round(x[0]), round(x[1])), x[2]) for x in route]
                    routes[vehicle.id] = route
                    break

            # Explore the node with the lowest fcost (hcost is tiebreaker)
            fcost, hcost, gcost, tb, road, position = heappop(open_nodes)
            real_position = getRealPositionOnRoad(road, position)

            # Add current to closed_nodes
            closed_nodes.add((road, position))

            # If destination is same as current position (by chance) then skip this vehicle
            if road == vehicle.destinationRoad and position == vehicle.destinationPosition:
                break

            # If destination is on the same road in front of the current position then calculate single instruction
            if road == vehicle.destinationRoad and position < vehicle.destinationPosition:
                previous_node[(vehicle.destinationRoad.roadID, vehicle.destinationPosition)] = (road.roadID, position)
                break

            if (road, 1) in closed_nodes: # if current road is the starting road, skip
                continue

            # Otherwise, create instruction to move to the end of the road as there is no other choice
            roadEndPosition: tuple[float, float] = road.endPosReal

            # Store references to road intersection for easy reference
            road_start_intersection: Intersection = landscape.intersections[road.start]
            road_end_intersection: Intersection = landscape.intersections[road.end]

            # Initiate time cost of reaching road end node (ignoring traffic lights)
            time_taken = euclideanDistance(
                real_position,
                roadEndPosition
            ) / road.speedLimit_MPS

            # Set previous node of road end node to road start node
            previous_node[(road.roadID, 1)] = (road.roadID, position)

            # Add road end to closed nodes
            closed_nodes.add((road, 1))

            # Update variables
            position = 1
            real_position = roadEndPosition
            gcost += time_taken
            hcost = manhattanDistance(
                real_position, 
                destination_position
            ) / MAX_ROAD_SPEED_MPS
            fcost = gcost + hcost

            # Examine neighbours
            for neighbour_intersection in road_end_intersection.neighbours:

                # No U turns allowed
                if neighbour_intersection == road_start_intersection:
                    continue

                # Store reference to neighbour road for easy reference
                neighbour_road = landscape.roadmap[road_end_intersection.coordinates()][neighbour_intersection.coordinates()]
                
                # If neighbour is in closed, skip
                if (neighbour_road, 0) in closed_nodes:
                    continue
                
                # Time cost of reaching neighbour node is the traversal time of virtual pathway
                time_taken = road_end_intersection.intersectionPathways[road_start_intersection][neighbour_intersection].traversalTime     
                # NOTE: traffic light waiting time is already accounted for by the gcost of reaching road end node

                # Compute all cost values
                neighbour_gcost = gcost + time_taken
                neighbour_hcost = manhattanDistance(
                    neighbour_road.startPosReal, 
                    destination_position
                ) / MAX_ROAD_SPEED_MPS
                neighbour_fcost = neighbour_gcost + neighbour_hcost

                neighbour_node = (neighbour_fcost, neighbour_hcost, neighbour_gcost, tiebreaker, neighbour_road, 0)
                tiebreaker += 1

                # Push neighbour node into open list if fcost is smaller than the existing cost
                if neighbour_fcost < node_fcost[(neighbour_road, 0)]:
                    node_fcost[(neighbour_road, 0)] = neighbour_fcost
                    previous_node[(neighbour_road.roadID, 0)] = (road.roadID, 1)
                    heappush(open_nodes, neighbour_node) # it does not matter whether neighbour is already in open list

        if broken:
            continue
        # Initiate a list that stores the sequence of (next real position, road ID) for the vehicle
        route: list[tuple[tuple[float, float], int]] = [] 

        # Initiate traceback variables
        current_roadID, current_position = vehicle.destinationRoad.roadID, vehicle.destinationPosition

        # Create route using previous_node hashmap
        while (current_roadID, current_position) in previous_node:

            # Unpack previous node information
            previous_roadID, previous_position = previous_node[(current_roadID, current_position)]
            
            # Append new instruction
            newRealPos = getRealPositionOnRoad(landscape.roads[current_roadID], current_position)
            if len(route) > 0 and newRealPos == route[-1][0]:
                pass # skip dupe coords in double intersections
            else:
                route.append((newRealPos, current_roadID))

            # Update traceback variables
            current_roadID, current_position = previous_roadID, previous_position

        # Reverse instructions to obtain chronological order
        route.reverse()
        # print()
        # print(route)

        # Store computed route in routes list            
        routes[vehicle.id] = route
    
    routes = dict(sorted(routes.items()))
    newRoutes = [routes[k] for k in routes.keys()]
    return newRoutes

def sortVehicles(autoflow_vehicles: list[Vehicle]):
    """
    Sorts vehicles based on a priority function.
    """
    return sorted(
        autoflow_vehicles,
        key = lambda vehicle: (
            vehicle.passengerCount * manhattanDistance(
                getRealPositionOnRoad(vehicle.road, vehicle.position),
                getRealPositionOnRoad(vehicle.destinationRoad, vehicle.destinationPosition)
            ),
            vehicle.emissionRate
        ),
        reverse=True
    )

def computeAutoflowVehicleRoutes(autoflow_vehicles: list[Vehicle], landscape: Landscape, MAX_ROAD_SPEED_MPS: float, carPositions = {}) -> list[list[tuple[float, float]]]:
    """
    AutoFlow vehicles perform cooperative A* with awareness of other AutoFlow vehicles.
    Vehicle priorities are determined by pre-trained gradient boosted regression trees.

    A space-time reservation table is used to keeps track of the number of vehicles on each road
    at any timestamp (in seconds). This greatly enhances the accuracy of cost functions when
    evaluating which path to take, as more congested roads would take longer to traverse.    

    Each node is a tuple that stores (fcost, hcost, gcost, tiebreaker, road, position).
    - fcost: sum of gcost and hcost, node with lowest fcost will be evaluated first
    - hcost: optimistic approximate time required to reach destination using MAX_ROAD_SPEED
    - gcost: cost so far i.e. time taken so far, represents the ABSOLUTE time
    - tiebreaker: an unique integer used as a tiebreaker when all costs are equal

    Every node pushed into the Open list will be the start of a road (or the starting position of the vehicle).
    The Closed list contains all visited nodes (including end points of a road as well as the starting position).
    """

    routes: dict[int, list[tuple[tuple[float, float], int]]] = {}

    # Sort the list of vehicles
    autoflow_vehicles = sortVehicles(autoflow_vehicles)


    #delayFactor = len(landscape.roads) // max(landscape.xSize, landscape.ySize)
    # delayFactor = 1.5 # exponential time
    congestionCost = 1
    # print("DF:", delayFactor, "CC:", congestionCost)

    # Set up space-time reservation table 
    reservation_table: dict[int, dict[int, int]] = defaultdict(lambda: defaultdict(int))
    # reservation_table[roadID][timestamp in seconds] => number of vehicles on road at timestamp

    # populate reservation table, since every car needs to get to the end of its spawn road
    for vehicle in autoflow_vehicles:

        positionRemaining = 1 - vehicle.position
        lengthRemaining = vehicle.road.length * positionRemaining
        timeTaken = lengthRemaining / vehicle.road.speedLimit_MPS

        for timestamp in range(ceil(timeTaken)):
            reservation_table[vehicle.road.roadID][timestamp] += congestionCost

    for vehicle in autoflow_vehicles:

        if vehicle.destinationRealPosition == vehicle.startRealPosition:
            routes[vehicle.id] = []
            continue

        tiebreaker = 0 # tiebreaker value for when all costs are equal

        # Hashmap that maps each node to their fcost
        node_fcost: dict[tuple[int, int], float] = defaultdict(lambda: float("inf"))

        # Hashmap that stores the previous node, road index in landscape.roads and ABSOLUTE time cost of each node
        previous_node: dict[tuple[int, float], tuple[int, float, float]] = {}
        # previous_node[(roadID, normalised position)] => (roadID, normalised position, absolute time)
        # NOTE: normalised position is used to handle roads where startPosReal and endPosReal are equal
        # NOTE: ABSOLUTE time is needed to prevent time desync within reservation table
        
        # Calculate real destination position
        destination_position = getRealPositionOnRoad(vehicle.destinationRoad, vehicle.destinationPosition)

        # Nodes are the starting points of each road, can also be the starting point of the vehicle
        open_nodes: list[float, float, float, Road, float] = [] # Open is a priority queue
        closed_nodes: set[tuple(Road, float)] = set() # Closed can just be a set

        # Calculate the cost variables of the starting position
        gcost = 0
        hcost = manhattanDistance(
            getRealPositionOnRoad(vehicle.road, vehicle.position), 
            destination_position
        ) / MAX_ROAD_SPEED_MPS
        fcost = gcost + hcost

        # Add starting position of the vehicle to open_nodes
        start_node = (fcost, hcost, gcost, tiebreaker, vehicle.road, vehicle.position)
        tiebreaker += 1
        heappush(open_nodes, start_node) 

        broken = False

        while True: # loop until target point has been reached

            if len(open_nodes) == 0:
                if carPositions == {}:
                    raise Exception("Path does not exist")
                else:
                    broken = True
                    route = carPositions[vehicle.id]["Routes"]
                    route = [((round(x[0]), round(x[1])), x[2]) for x in route]
                    routes[vehicle.id] = route
                    break

            # Explore the node with the lowest fcost (hcost is tiebreaker)
            fcost, hcost, gcost, tb, road, position = heappop(open_nodes)
            real_position = getRealPositionOnRoad(road, position)

            # Add current to closed_nodes
            closed_nodes.add((road, position))

            # If destination is same as current position (by chance) then skip this vehicle
            if road == vehicle.destinationRoad and position == vehicle.destinationPosition:
                break

            # If destination is on the same road in front of the current position then calculate single instruction
            if road == vehicle.destinationRoad and position < vehicle.destinationPosition:
                time_taken = euclideanDistance(
                    real_position,
                    destination_position
                ) / road.speedLimit_MPS # fastest time estimation from start of the road to the destination
                previous_node[(vehicle.destinationRoad.roadID, vehicle.destinationPosition)] = (
                    road.roadID, position, gcost + time_taken
                )
                break

            if (road, 1) in closed_nodes: # if current road is the starting road, skip
                continue

            # Otherwise, create instruction to move to the end of the road as there is no other choice
            roadEndPosition: tuple[float, float] = road.endPosReal

            # Store references to road intersection for easy reference
            road_start_intersection: Intersection = landscape.intersections[road.start]
            road_end_intersection: Intersection = landscape.intersections[road.end]

            # Initiate time cost of reaching road end node (ignoring traffic lights & any congestion)
            time_taken = euclideanDistance(
                real_position,
                roadEndPosition
            ) / road.speedLimit_MPS

            # Compute waiting time until the next green light
            if len(road_end_intersection.neighbours) >= 3:
                current_modulus_time = gcost % (len(road_end_intersection.neighbours) * road_end_intersection.trafficLightDuration)
                if (
                    (road_end_intersection.trafficLightLookup[road_start_intersection] + 1) * road_end_intersection.trafficLightDuration 
                    > current_modulus_time
                ):
                    waiting_time = (
                        road_end_intersection.trafficLightLookup[road_start_intersection] * road_end_intersection.trafficLightDuration 
                        - current_modulus_time
                    )
                    if waiting_time > 0: # Case 1: current time is earlier in the cycle
                        pass
                    else: # Case 2: current time is within the green light duration, allow vehicle through
                        waiting_time = 0
                else: # Case 3: current time is later in the cycle
                    waiting_time = (
                        len(road_end_intersection.neighbours) * road_end_intersection.trafficLightDuration
                        - current_modulus_time 
                        + road_end_intersection.trafficLightLookup[road_start_intersection] * road_end_intersection.trafficLightDuration
                    )
                time_taken += waiting_time # update time taken to reflect traffic light waiting time

                # Compute cost of reaching road end node (taking congestion into account)
                time_taken += (
                    reservation_table[road.roadID][int(gcost)]
                    // road_end_intersection.trafficPassthroughRate[road_start_intersection]
                    * road_end_intersection.trafficLightDuration * len(road_end_intersection.neighbours)
                ) # update time taken to reflect the number of traffic light cycles waited
                # print(reservation_table[road.roadID][int(gcost)])
                # print(road_end_intersection.trafficPassthroughRate[road_start_intersection])
                # print(road_end_intersection.trafficLightDuration)
                # print(road.speedLimit_MPS)
                # print()
                # print((
                #     reservation_table[road.roadID][int(gcost)]
                #     // road_end_intersection.trafficPassthroughRate[road_start_intersection]
                #     * road_end_intersection.trafficLightDuration
                # ))

            # Set previous node of road end node to road start node
            previous_node[(road.roadID, 1)] = (
                road.roadID, position, gcost + time_taken
            )

            # Add road end to closed nodes
            closed_nodes.add((road, 1))

            # Update variables
            position = 1
            real_position = roadEndPosition
            gcost += time_taken
            hcost = manhattanDistance(
                real_position, 
                destination_position
            ) / MAX_ROAD_SPEED_MPS
            fcost = gcost + hcost

            # Examine neighbours
            for neighbour_intersection in road_end_intersection.neighbours:

                # No U turns allowed
                if neighbour_intersection == road_start_intersection:
                    continue

                # Store reference to neighbour road for easy reference
                neighbour_road = landscape.roadmap[road_end_intersection.coordinates()][neighbour_intersection.coordinates()]
                
                # If neighbour is in closed, skip
                if (neighbour_road, 0) in closed_nodes:
                    continue
                
                # Time cost of reaching neighbour node is the traversal time of virtual pathway
                time_taken = road_end_intersection.intersectionPathways[road_start_intersection][neighbour_intersection].traversalTime     
                # NOTE: traffic light waiting time is already accounted for by the gcost of reaching road end node

                # Compute all cost values
                neighbour_gcost = gcost + time_taken
                neighbour_hcost = manhattanDistance(
                    neighbour_road.startPosReal, 
                    destination_position
                ) / MAX_ROAD_SPEED_MPS
                neighbour_fcost = neighbour_gcost + neighbour_hcost

                neighbour_node = (neighbour_fcost, neighbour_hcost, neighbour_gcost, tiebreaker, neighbour_road, 0)
                tiebreaker += 1

                # Push neighbour node into open list if fcost is smaller than the existing cost
                if neighbour_fcost < node_fcost[(neighbour_road, 0)]:
                    node_fcost[(neighbour_road, 0)] = neighbour_fcost
                    previous_node[(neighbour_road.roadID, 0)] = (
                        road.roadID, 1, -1 # skipped to avoid double marking in reservation table
                    )
                    heappush(open_nodes, neighbour_node) # it does not matter whether neighbour is already in open list

        if broken:
            continue
        # Initiate a list that stores the sequence of (next real position, road ID) for the vehicle
        route: list[tuple[tuple[float, float], int]] = [] 

        # Initiate traceback variables
        current_roadID, current_position = vehicle.destinationRoad.roadID, vehicle.destinationPosition
        previousTimestamp = -1

        # Create route using previous_node hashmap
        while (current_roadID, current_position) in previous_node:

            # Unpack previous node information
            previous_roadID, previous_position, timestamp = previous_node[(current_roadID, current_position)]
            
            # Append new instruction
            newRealPos = getRealPositionOnRoad(landscape.roads[current_roadID], current_position)
            if len(route) > 0 and newRealPos == route[-1][0]:
                pass # skip dupe coords in double intersections
            else:
                route.append((newRealPos, current_roadID))

            # Update congestion status of used road during the usage time period
            if previousTimestamp == -1:
                previousTimestamp = timestamp
            elif timestamp != -1: # skip virtual pathways for reservation table marking
                for i in range(int(timestamp), int(previousTimestamp)):
                    reservation_table[previous_roadID][i] += congestionCost              
                previousTimestamp = timestamp # update previous timestamp

            # Update traceback variables
            current_roadID, current_position = previous_roadID, previous_position

        # Reverse instructions to obtain chronological order
        route.reverse()
        # print()
        # print(route)

        # Store computed route in routes list                
        routes[vehicle.id] = route

    # for route in routes:
    #     print(route)

    # for route in routes:
    #     for i in range(len(route)-1):
    #         if (
    #             route[i][0][0] != route[i+1][0][0] and 
    #             route[i][0][1] != route[i+1][0][1] and 
    #             euclideanDistance(route[i][0], route[i+1][0]) > 30
    #         ):
    #             raise Exception(f"Goals are too far apart: {route[i][0]} and {route[i+1][0]}")
    
    routes = dict(sorted(routes.items()))
    newRoutes = [routes[k] for k in routes.keys()]

    return newRoutes


def recalculateRoutes(carPositions, landscape : Landscape, vehicles : list[Vehicle], MAX_ROAD_SPEED_MPS, update_interval : int):
    """
    Periodically recalculates routes optimally

    Best runtime: 1s
    """
    assert len(carPositions) == len(vehicles)

    specialCases = {}

    # a buffer of n keeps the next n nodes the same
    # the lower the buffer, the better the performance since there's more room to improve
    # automatic setting assuming you can get through 2 goals per second
    buffer = update_interval

    buffers = {}
    for i in range(len(vehicles)):
        buffers[vehicles[i].id] = []
    
    vehicles = {vehicle.id: vehicle for vehicle in vehicles}

    
    for id, data in carPositions.items():
        
        vehicle = vehicles[id]

        route = data["Routes"]
        x, y, roadID = data["Metadata"]

        # if the car reached its desination, we can ignore it
        if id == -1:
            continue
        
        # if the car has <= buffer+1 routes left, there's no need to update routes
        if len(route) <= buffer + 1:
            specialCases[id] = route
            if roadID != -1:
                vehicle.setLocation(landscape.lookupRoad[roadID], landscape.lookupRoad[roadID].get_position(x, y))
            else:
                vehicle.setLocation(landscape.lookupRoad[1], 0)
                vehicle.setDestination(landscape.lookupRoad[1], 0)
        
        # but if a car has more than buffer + 1 entries, it should be updated
        if len(route) > buffer + 1:
            for i in range(buffer):
                if i == buffer - 1:
                    roadID = route[i][2]
                    x, y = route[i][0], route[i][1]
                buffers[id].append(route[i])
            
            vehicle.setLocation(landscape.lookupRoad[roadID], landscape.lookupRoad[roadID].get_position(x, y))
    
    selfish = [vehicles[i] for i in vehicles.keys() if vehicles[i].routingSystem == "Selfish"]
    autoflow = [vehicles[i] for i in vehicles.keys() if vehicles[i].routingSystem == "Autoflow"]

    newRoutes = computeRoutes(selfish, autoflow, landscape, MAX_ROAD_SPEED_MPS, carPositions=carPositions)

    # replace special case routes, otherwise put routes in the right format
    finalRoutes = {}
    for id, route in newRoutes.items():
        if id in specialCases.keys():
            finalRoutes[id] = specialCases[id]
        else:
            temp = buffers[id]
            calculatedRoutes = [(round(x[0][0]), round(x[0][1]), x[1]) for x in route]

            if calculatedRoutes != carPositions[id]["Routes"]:
                if temp[-1] == calculatedRoutes[0]:
                    del calculatedRoutes[0]
                temp += calculatedRoutes
                finalRoutes[id] = temp
            else:
                finalRoutes[id] = calculatedRoutes

        # if finalRoutes[id] != carPositions[id]["Routes"]:
        #     print(finalRoutes[id])
        #     print(carPositions[id]["Routes"])
        #     print()

    

    return finalRoutes