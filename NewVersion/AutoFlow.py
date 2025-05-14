#================ IMPORTS ================
from NewVersion.LandscapeComponents import *
from NewVersion.VehicleAgents import *

from random import sample
from heapq import *
from math import ceil
import random
from NewVersion.SegmentTree import *
import time
import subprocess
import json
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
    

    # Send data to the C++ program
    input_data = ""
    
    # intersections
    input_data += f"{len(landscape.intersections)}\n"
    for intersection in landscape.intersections.values():
        input_data += f"{intersection.id} {intersection.trafficLightDuration} {intersection.roadCount} {intersection.x} {intersection.y}\n"

    
    # roads
    input_data += f"{len(landscape.roads)}\n"
    for road in landscape.roads:
        st = f"{road.id} {road.length} {road.speedLimit} {road.capacity} {road.int1.id} {road.int2.id} {road.traversalTime} {road.laneCount}\n"
        input_data += st
        input_data += f"{len(road.positions)}\n"
        for pos in road.positions:
            input_data += f"{pos[0]} {pos[1]} {pos[2]}\n"

    # graph
    input_data += f"{len(landscape.GRAPH)}\n"
    for key, value in landscape.GRAPH.items():
        input_data += f"{len(value)}\n"
        for v in value:
            input_data += f"{key.id} {v.id} {landscape.GRAPH[key][v].id}\n"

    # vehicles
    input_data += f"{len(selfish_vehicles)}\n"
    for vehicle in selfish_vehicles:
        input_data += f"{vehicle.id} {vehicle.startingRoadId} {vehicle.endingRoadId} {vehicle.starting[0]} {vehicle.starting[1]} {vehicle.starting[2]} {vehicle.ending[0]} {vehicle.ending[1]} {vehicle.ending[2]}\n"
    

    process = subprocess.Popen(["NewVersion/Algorithm/NaiveSelfish"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # process.stdin.write(input_data)
    # process.stdin.flush()
    stdout, stderr = process.communicate(input=input_data)
    
    # Check for errors
    if stderr:
        print(f"Error from C++ program: {stderr}")
    

    actual_output = stdout.split("\n")
    # print(actual_output)
    ind = actual_output.index("---")
    actual_output = actual_output[ind+1:]
    # print(actual_output)
    stdout = "\n".join(stdout.split('\n')[:ind])

    # Read the output from the C++ program
    #output_data = process.stdout.readline()
    #print(f"C++ says: {output_data}")
    print(stdout)

    process.stdin.close()
    process.wait()

    routes = {}
    for vehicle in actual_output:
        if len(vehicle) == 0:
            continue
        i = vehicle.split()
        #print(i)
        routes[int(i[0])] = [landscape.intersections[int(j)] for j in i[1:]]

    return routes

def sortVehicles(autoflow_vehicles: list[Vehicle]):
    """
    Sorts vehicles based on a custom priority function, prioritising vehicle passenger count, travel distance, and high emission rates.
    """
    return sorted(
        autoflow_vehicles,
        key = lambda vehicle: (
            vehicle.passengerCount * vehicle.emissionRate
        )
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

    # Start the C++ program as a subprocess
    autoflowVehicles = autoflow_vehicles
    

    # Send data to the C++ program
    input_data = ""
    
    # intersections
    input_data += f"{len(landscape.intersections)}\n"
    for intersection in landscape.intersections.values():
        input_data += f"{intersection.id} {intersection.trafficLightDuration} {intersection.roadCount} {intersection.x} {intersection.y}\n"

    
    # roads
    input_data += f"{len(landscape.roads)}\n"
    for road in landscape.roads:
        st = f"{road.id} {road.length} {road.speedLimit} {road.capacity} {road.int1.id} {road.int2.id} {road.traversalTime} {road.laneCount}\n"
        input_data += st
        input_data += f"{len(road.positions)}\n"
        for pos in road.positions:
            input_data += f"{pos[0]} {pos[1]} {pos[2]}\n"

    # graph
    input_data += f"{len(landscape.GRAPH)}\n"
    for key, value in landscape.GRAPH.items():
        input_data += f"{len(value)}\n"
        for v in value:
            input_data += f"{key.id} {v.id} {landscape.GRAPH[key][v].id}\n"

    # vehicles
    input_data += f"{len(autoflowVehicles)}\n"
    for vehicle in autoflowVehicles:
        input_data += f"{vehicle.id} {vehicle.startingRoadId} {vehicle.endingRoadId} {vehicle.starting[0]} {vehicle.starting[1]} {vehicle.starting[2]} {vehicle.ending[0]} {vehicle.ending[1]} {vehicle.ending[2]} {vehicle.passengerCount} {vehicle.emissionRate}\n"
    
    for i in range(5):

        process = subprocess.Popen(["NewVersion/Algorithm/NaiveSelfish2"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # process.stdin.write(input_data)
        # process.stdin.flush()
        stdout, stderr = process.communicate(input=input_data)
        
        # Check for errors
        if stderr:
            print(f"Error from C++ program: {stderr}")
        

        actual_output = stdout.split("\n")
        #print(actual_output)
        # if '---' not in actual_output:
        #     print(stdout)
        #     routes = {}
        #     break
        # print(actual_output)
        ind = actual_output.index("---")
        actual_output = actual_output[ind+1:]
        # print(actual_output)
        stdout = "\n".join(stdout.split('\n')[:ind])

        # Read the output from the C++ program
        #output_data = process.stdout.readline()
        #print(f"C++ says: {output_data}")
        print(stdout)

        process.stdin.close()
        process.wait()

        routes = {}
        for vehicle in actual_output:
            if len(vehicle) == 0:
                continue
            i = vehicle.split()
            #print(i)
            routes[int(i[0])] = [landscape.intersections[int(j)] for j in i[1:]]
        
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
                    "route": [intersection.id for intersection in routes.get(vehicle.id, [])]
                } for vehicle in autoflow_vehicles
            ]
        }

        with open("simulation_data.json", "w") as f:
            json.dump(sim_data, f)
        
        subprocess.run(["NewVersion/TrafficSimulatorV2"])

   
    # print(len(landscape.intersections))


    # # Sort the list of vehicles
    

    # # how many vehicles on each road at each time
    # # implemented using a lazy minimum segment tree for O(logn) range queries and updates
    # reservationTable : dict[Road, LPSTree] = defaultdict(lambda: LPSTree(10000, value=0, reducef=min))

    # # populate the reservation table for each car's initial starting position until they reach the end of the road, as this is unavoidable
    # for vehicle in autoflowVehicles:
    #     traversalTime = ceil(vehicle.road.traversalTime * (1 - vehicle.position))
    #     reservationTable[vehicle.road].add(0, traversalTime, 1)
        
    # for vehicle in autoflowVehicles:

    #     starttime = time.time()
        
    #     # A*
    #     openNodes = []
    #     openSet = set()
    #     closedNodes = set()
    #     start = Node(vehicle.starting)
    #     end = Node(vehicle.ending)
    #     openDict = defaultdict(lambda: 9999)

    #     heappush(openNodes, start)
    #     openSet.add(start.position)
    #     openDict[start.position] = 0
    #     finalPath = []
    #     exp = 0

    #     while len(openNodes) > 0:
    #         exp += 1
    #         #print(exp)
    #         currentNode = heappop(openNodes)
    #         openSet.remove(currentNode.position)
    #         closedNodes.add(currentNode.position)

    #         if end.position.road == currentNode.position.road:
    #             path = []
    #             while currentNode:

    #                 path.append(currentNode)
    #                 currentNode = currentNode.parent
                
    #             finalPath = path[::-1]
    #             betterPath = []
                
    #             # update the reservation table ONLY IF there were no issues
    #             for i in range(len(finalPath) - 1):
    #                 curtime = finalPath[i].g
    #                 nextTime = finalPath[i+1].g
    #                 currentRoad = finalPath[i].position.road
    #                 reservationTable[currentRoad].add(curtime, nextTime+1, 1)
    #                 betterPath.append(finalPath[i+1].position)

    #             finalPath = betterPath

    #             break
            
    #         # this is how we check for the next node to visit: first, iterate over all associated virtual intersections on the same road
    #         # these are the nodes that allow you to transition to the next road, so they are an intermediary step
    #         # think of avi being at the end of curr road and neighbour being at start of next road
    #         for avi in currentNode.position.road.associatedVirtualIntersections:
                
    #             # check if this virtual intersection is attached to the correct side of the road
    #             nodeNeeded = None
    #             if currentNode.position.direction == 1:
    #                 nodeNeeded = currentNode.position.road.int2
    #             else:
    #                 nodeNeeded = currentNode.position.road.int1
                
    #             # if its not the correct side of the road, skip this virtual intersection
    #             if avi.correspondingRealIntersection != nodeNeeded or avi.direction != currentNode.position.direction:
    #                 continue
                
    #             # then, we check everything this intermediary node is connected to - hopefully, we find one on another road
    #             for neighbour in avi.connectingVirtualInts:
                
    #                 if neighbour in closedNodes:
    #                     continue
                    
    #                 intermediary = Node(avi, currentNode)
    #                 neighNode = Node(neighbour, intermediary)
    #                 road = currentNode.position.road
    #                 currentTime = max(ceil(currentNode.g), 0)
    #                 #print(f"Current time: {currentTime}")
                    
    #                 roadLeavingTime = currentTime
    #                 congestion = reservationTable[road][currentTime] / autoFlowPercentage

    #                 # binary search on the first index such that the range min is < capacity (log2(x)^2 complexity)
    #                 l = currentTime + 1
    #                 r = 9999
    #                 tree = reservationTable[road]
    #                 r += 1
    #                 while l < r:
    #                     mid = l + (r - l) // 2
    #                     query = tree.get(currentTime, mid+1)
    #                     if query < road.capacity * autoFlowPercentage:
    #                         r = mid
    #                     else:
    #                         l = mid + 1
    #                 roadLeavingTime = r

    #                 if roadLeavingTime >= 10000:
    #                     roadLeavingTime = currentTime + 1

    #                 #print(f"Road leaving time: {roadLeavingTime}")
                
                    
    #                 # calculate traversal time until we reach the last car on the road
    #                 roadLeavingTime += max(0, (road.length - VEHICLE_LENGTH_METRES * congestion) / road.speedLimit)
    #                 #print(f"Road leaving time after congestion: {roadLeavingTime}")

    #                 rInt = neighbour.correspondingRealIntersection

    #                 # calculate time until all the traffic light cycles
    #                 cycleTime = rInt.trafficLightDuration * rInt.roadCount
    #                 roadLeavingTime += cycleTime * congestion

    #                 #print(f"Road leaving time after traffic lights: {roadLeavingTime}")

    #                 neighNode.g = ceil(roadLeavingTime)
    #                 neighNode.h = heuristic(neighbour, vehicle.ending)
    #                 neighNode.f = neighNode.g + neighNode.h
    #                 intermediary.g = neighNode.g
    #                 intermediary.h = heuristic(avi, vehicle.ending)
    #                 intermediary.f = intermediary.g + intermediary.h

    #                 if neighbour not in openSet:
    #                     openSet.add(neighbour)
    #                     heappush(openNodes, neighNode)
    #                 elif neighNode.g > openDict[neighbour]:
    #                     continue
                    
    #                 # push the node into the open list
    #                 openDict[neighbour] = neighNode.g
    #                 openDict[intermediary.position] = intermediary.g

        # routes[vehicle.id] = finalPath

        # print(f"routed in {time.time() - starttime}")

    return routes