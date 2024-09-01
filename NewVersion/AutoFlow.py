
#================ IMPORTS ================
from NewVersion.LandscapeComponents import *
from NewVersion.VehicleAgents import *

from random import sample
from heapq import *
from math import ceil
import random
from NewVersion.SegmentTree import *
#=========================================
MAX_ROAD_SPEED_MPS = 28

# ===============================================================================================
# Helper Functions
# ===============================================================================================
    
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

def heuristic(int1 : Intersection, int2 : Intersection):
    """
    Very simple heuristic function that returns the euclidean distance between two intersections.

    The heuristic is optimistic and assumes that the vehicle can travel at the maximum speed limit
    on all roads. This is not the case in reality, as the vehicle will have to slow down at traffic
    lights and congested roads.
    """
    return euclideanDistance(int1.coordinates(), int2.coordinates()) / MAX_ROAD_SPEED_MPS


class Node:
    '''
    Node class for the A* algorithm used to simplify the algorithm.
    '''
    def __init__(self, intersection, parent=None):
        self.position = intersection
        self.parent = parent
        self.g = 0  # Cost from start to this node
        self.h = 0  # Heuristic cost from this node to end
        self.f = 0  # Total cost (g + h)

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)


# ===============================================================================================
# Main Functions
# ===============================================================================================

def computeRoutes(selfish_vehicles: list[Vehicle], autoflow_vehicles: list[Vehicle], landscape: Landscape):
    """
    Computes the routes for selfish vehicles first, then AutoFlow vehicles.
    """
    routes = {}
    autoFlowPercentage = len(autoflow_vehicles) / (len(autoflow_vehicles) + len(selfish_vehicles))
    selfish_vehicle_routes = computeSelfishVehicleRoutes(selfish_vehicles, landscape)
    autoflow_vehicle_routes = computeAutoflowVehicleRoutes(autoflow_vehicles, landscape, autoFlowPercentage)

    for vehicle in selfish_vehicles:
        routes[vehicle.id] = selfish_vehicle_routes[vehicle.id]
    
    for vehicle in autoflow_vehicles:
        routes[vehicle.id] = autoflow_vehicle_routes[vehicle.id]

    return routes

def computeSelfishVehicleRoutes(selfish_vehicles: list[Vehicle], landscape: Landscape):
    """
    Selfish routing algorithm of Google Maps, although somewhat simplified as the real algorithm they use is classified.
    Vehicles are not knowledgeable of future traffic and therefore only aware of congestion after it occurs.

    Each node is a packaged intersection that stores:
    - fcost: sum of gcost and hcost, node with lowest fcost will be evaluated first
    - hcost: optimistic approximate time required to reach destination using MAX_ROAD_SPEED
    - gcost: cost so far i.e. time taken so far, represents the ABSOLUTE time

    Every node pushed into the Open list will be the start of a road (or the starting position of the vehicle).
    The Closed list contains all visited nodes (including end points of a road as well as the starting position).
    """
    
    routes: dict[int, list[tuple[tuple[float, float], int]]] = {}

    for vehicle in selfish_vehicles:

        startNode = vehicle.starting
        endNode = vehicle.ending
        
        # A*
        openNodes = []
        closedNodes = set()
        start = Node(startNode)
        end = Node(endNode)
        openDict = {}

        heappush(openNodes, start)
        openDict[start.position] = start
        finalPath = []

        while len(openNodes) > 0:
            currentNode = heappop(openNodes)

            # faster than checking if the node is in the open list
            try:
                del openDict[currentNode.position]
            except:
                pass
            
            closedNodes.add(currentNode.position)

            # rebuild the path
            if end.position.road == currentNode.position.road:
                path = []
                while currentNode:
                    path.append(currentNode.position)
                    currentNode = currentNode.parent
                finalPath = path[::-1]
                finalPath.append(end.position)
                break
                
            for avi in currentNode.position.road.associatedVirtualIntersections:

                nodeNeeded = None
                if currentNode.position.direction == 1:
                    nodeNeeded = currentNode.position.road.int2
                else:
                    nodeNeeded = currentNode.position.road.int1
                
                if avi.correspondingRealIntersection != nodeNeeded or avi.direction != currentNode.position.direction:
                    continue

                for neighbour in avi.connectingVirtualInts:
                
                    if neighbour in closedNodes or neighbour.road == currentNode.position.road:
                        continue
                    
                    # very simple algorithm that solely calculates the time taken to reach the destination
                    intermediary = Node(avi, currentNode)
                    neighNode = Node(neighbour, intermediary)
                    road = currentNode.position.road
                    neighNode.g = currentNode.g + road.traversalTime
                    neighNode.h = heuristic(neighbour, endNode)
                    
                    # more weighting on the heuristic as it is somewhat accurate, and also significantly speeds up the process
                    neighNode.f = neighNode.g + neighNode.h * 2

                    if neighbour in openDict and neighNode.g > openDict[neighbour].g:
                        continue

                    heappush(openNodes, neighNode)
                    openDict[neighbour] = neighNode
        
        routes[vehicle.id] = finalPath
    
    return routes

def sortVehicles(autoflow_vehicles: list[Vehicle]):
    """
    Sorts vehicles based on a custom priority function, prioritising vehicle passenger count, travel distance, and high emission rates.
    """
    return sorted(
        autoflow_vehicles,
        key = lambda vehicle: (
            vehicle.passengerCount * euclideanDistance(
                getRealPositionOnRoad(vehicle.road, vehicle.position, vehicle.direction),
                getRealPositionOnRoad(vehicle.destinationRoad, vehicle.destinationPosition, vehicle.destinationDirection)
            ) * vehicle.emissionRate
        ),
        reverse=True
    )

def computeAutoflowVehicleRoutes(autoflow_vehicles: list[Vehicle], landscape: Landscape, autoFlowPercentage : float):
    """
    AutoFlow vehicles perform cooperative A* with awareness of other AutoFlow vehicles.
    Vehicle priorities are determined by a custom sorting function.

    A space-time reservation table is used to keeps track of the number of vehicles on each road
    at any timestamp (in seconds). This greatly enhances the accuracy of cost functions when
    evaluating which path to take, as more congested roads would take longer to traverse.    

    Each node is a packaged intersection that stores:
    - fcost: sum of gcost and hcost, node with lowest fcost will be evaluated first
    - hcost: optimistic approximate time required to reach destination using MAX_ROAD_SPEED
    - gcost: cost so far i.e. time taken so far, represents the ABSOLUTE time

    TODO: improve speed using math + memoization + planar graph optimisations
    """

    routes = {}

    # Sort the list of vehicles
    autoflowVehicles = sortVehicles(autoflow_vehicles)

    # how many vehicles on each road at each time
    # implemented using a lazy minimum segment tree for O(logn) range queries and updates
    reservationTable : dict[Road, LPSTree] = defaultdict(lambda: LPSTree(10000, value=0, reducef=min))

    # populate the reservation table for each car's initial starting position until they reach the end of the road, as this is unavoidable
    for vehicle in autoflowVehicles:
        traversalTime = ceil(vehicle.road.traversalTime * (1 - vehicle.position))
        reservationTable[vehicle.road].add(0, traversalTime, 1)
        
    for vehicle in autoflowVehicles:

        # determine the starting and ending nodes (ignoring the vehicle's actual starting and ending positions on the road)
        if vehicle.direction == 1:
            startNode = vehicle.road.int2
        else:
            startNode = vehicle.road.int1
        
        if vehicle.destinationDirection == 1:
            endNode = vehicle.destinationRoad.int1
        else:
            endNode = vehicle.destinationRoad.int2
        
        # A*
        openNodes = []
        closedNodes = set()
        start = Node(startNode)
        end = Node(endNode)
        openDict = {}

        heappush(openNodes, start)
        openDict[start.position] = start
        finalPath = []

        while len(openNodes) > 0:
            currentNode = heappop(openNodes)
            try:
                del openDict[currentNode.position]
            except:
                pass
            
            closedNodes.add(currentNode.position)

            # if currentNode == end:
            #     path = []
            #     while currentNode:

            #         path.append(currentNode)
            #         currentNode = currentNode.parent
                
            #     finalPath = path[::-1]
            #     finalPathCopy = finalPath
                
            #     finalPath = [node.position for node in finalPath]
                
            #     # convert everything to virtual ints

            #     vIntPath = [vehicle.starting]
            #     currentLane = vehicle.lane
            #     currentDirection = vehicle.direction
            #     currentRoad = vehicle.road

            #     for i in range(len(finalPath)):
            #         # find all possible ending virtual intersections i.e. end of road, same direction, any lane
            #         possibleEnds : list[VirtualIntersection] = []

            #         assert len(currentRoad.virtualInts) > 0
                    
            #         for vInt in currentRoad.virtualInts:
            #             # print(vInt.direction, currentDirection, vInt.correspondingRealIntersection, finalPath[i])
            #             if vInt.direction == currentDirection and vInt.correspondingRealIntersection == finalPath[i]:
            #                 possibleEnds.append(vInt)
            #                 #print(f"road has {currentRoad.laneCount} lanes, directions {vInt.direction} {currentDirection} lane {vInt.lane} {currentLane} road {vInt.road.id} {currentRoad.id} corres {vInt.correspondingRealIntersection.id} {finalPath[i].id}")
                    
            #         # from these possible endings, we choose the one that's lane is closest to the current lane, and is able to turn on to the next road
            #         bestEnd = None
            #         turningInt : VirtualIntersection = None
            #         minDist = 5
            #         for vInt in possibleEnds:
            #             # there should only be one connectingInt that you can turn to
            #             canTurn = False
            #             tInt : VirtualIntersection = None

            #             assert len(vInt.connectingVirtualInts) > 0

            #             for connectingInt in vInt.connectingVirtualInts:
            #                 if connectingInt.correspondingRealIntersection == finalPath[i]:
            #                     canTurn = True
            #                     tInt = connectingInt
                        
            #             if canTurn:
            #                 dist = abs(vInt.lane - currentLane)
            #                 if dist < minDist:
            #                     bestEnd = vInt
            #                     minDist = dist
            #                     turningInt = tInt
                    
            #         # add both bestEnd and turningInt to the path
            #         vIntPath.append(bestEnd)
            #         vIntPath.append(turningInt)

            #         # update currentLane, currentDirection, currentRoad
            #         currentLane = turningInt.lane
            #         currentDirection = turningInt.direction
            #         currentRoad = turningInt.road
                    
            #         # add the final destination
            #         vIntPath.append(vehicle.ending)

            #         # update the reservation table ONLY IF there were no issues
            #         for i in range(len(finalPathCopy) - 1):
            #             curtime = finalPathCopy[i].g
            #             nextTime = finalPathCopy[i+1].g
            #             currentRoad = landscape.GRAPH[finalPathCopy[i].position][finalPathCopy[i+1].position]
            #             reservationTable[currentRoad].add(curtime, nextTime+1, 1)

            #         finalPath = vIntPath
            #         break
                
            for neighbour in currentNode.position.connectingInts:
                
                if neighbour in closedNodes:
                    continue
                    
                neighNode = Node(neighbour, currentNode)
                road = landscape.GRAPH[currentNode.position][neighbour]

                # initialising the time to the next integer second, and retrieving the congestion on the road
                currentTime = max(ceil(currentNode.g), 0)
                #print(f"Current time: {currentTime}")
                
                roadLeavingTime = currentTime
                congestion = reservationTable[road][currentTime] / autoFlowPercentage

                # binary search on the first index such that the range min is < capacity (log2(x)^2 complexity)
                l = currentTime + 1
                r = 9999
                tree = reservationTable[road]
                r += 1
                while l < r:
                    mid = l + (r - l) // 2
                    query = tree.get(currentTime, mid+1)
                    if query < road.capacity * autoFlowPercentage:
                        r = mid
                    else:
                        l = mid + 1
                roadLeavingTime = r

                if roadLeavingTime >= 10000:
                    roadLeavingTime = currentTime + 1

                #print(f"Road leaving time: {roadLeavingTime}")
            
                
                # calculate traversal time until we reach the last car on the road
                roadLeavingTime += max(0, (road.length - VEHICLE_LENGTH_METRES * congestion) / road.speedLimit)
                #print(f"Road leaving time after congestion: {roadLeavingTime}")

                # calculate time until all the traffic light cycles
                cycleTime = neighbour.trafficLightDuration * neighbour.roadCount
                roadLeavingTime += cycleTime * congestion

                #print(f"Road leaving time after traffic lights: {roadLeavingTime}")

                neighNode.g = ceil(roadLeavingTime)
                neighNode.h = heuristic(neighbour, endNode)
                neighNode.f = neighNode.g + neighNode.h

                if neighbour in openDict and neighNode.g > openDict[neighbour].g:
                    continue

                heappush(openNodes, neighNode)
                openDict[neighbour] = neighNode

        routes[vehicle.id] = finalPath

    return routes