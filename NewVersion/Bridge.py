from NewVersion.LandscapeComponents import * 
from NewVersion.AutoFlow import *
import random
import asyncio
import dataclasses
from websockets.exceptions import ConnectionClosedOK
from websockets.server import WebSocketServerProtocol, serve
import json
import websockets

PORT = 8001

@dataclasses.dataclass
class Vector2Message:
    x: float
    y: float

    def __dict__(self):
        return {"x": self.x, "y": self.y}

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return Vector2Message(**d)
    
@dataclasses.dataclass
class Vector3Message:
    x: float
    y: float
    z: float

    def __dict__(self):
        return {"x": self.x, "y": self.y, "z": self.z}

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return Vector3Message(**d)

@dataclasses.dataclass
class VirtualIntMsg:
    id: int
    x: float
    y: float
    lane: int
    direction: int
    position: float
    trafficLightOrder: list[int]
    lightTiming: int
    road: int


    def __dict__(self):
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "lane": self.lane,
            "direction": self.direction,
            "position": self.position,
            "trafficLightOrder": self.trafficLightOrder,
            "lightTiming": self.lightTiming,
            "road": self.road
        }
    
    def serialize(self):
        return json.dumps(self.__dict__())
    
    @staticmethod
    def deserialize(d):
        return VirtualIntMsg(**d)
@dataclasses.dataclass
class RoadInitMsg:
    id: int
    startPos: Vector2Message
    endPos: Vector2Message
    laneCount: int
    speedLimit: int
    capacity: int
    virtualInts: list[VirtualIntMsg]

    def __dict__(self):
        return {
            "id": self.id,
            "startPos": self.startPos.__dict__(),
            "endPos": self.endPos.__dict__(),
            "laneCount": self.laneCount,
            "virtualInts": [vInt.__dict__() for vInt in self.virtualInts],
            "speedLimit": self.speedLimit,
            "capacity": self.capacity
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return RoadInitMsg(**d)
    
@dataclasses.dataclass
class VehicleInitMsg:
    id: int
    type : int
    emissionRate: float
    useAutoFlow: bool
    passengerCount: int
    # x, z, roadID
    position: VirtualIntMsg
    route: list[VirtualIntMsg]

    def __dict__(self):
        return {
            "id": self.id,
            "position": self.position.__dict__(),
            "emissionRate": self.emissionRate,
            "useAutoFlow": self.useAutoFlow,
            "passengerCount": self.passengerCount,
            "route": [v3.__dict__() for v3 in self.route],
            "type": self.type,
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return VehicleInitMsg(**d)
    
@dataclasses.dataclass
class AllRoads:
    
    roads: list[RoadInitMsg]
    vehicles: list[VehicleInitMsg]
    
    def __dict__(self):
        return {
            "roads": [road.__dict__() for road in self.roads],
            "vehicles": [vehicle.__dict__() for vehicle in self.vehicles]
        }
    
    def serialize(self):
        return json.dumps(self.__dict__())
    
    @staticmethod
    def deserialize(d):
        return AllRoads(**d)



async def handleNew(websocket: WebSocketServerProtocol, selectedIndex, vehicleDensity, autoflow_percentage, mapSize, receiveNewDests, roadBlockage):
    cities = ["sydney", "melbourne", "manhattan", "los_angeles", "london", "tokyo"]
    
    # convert map size to length: we set max size to 3k * 3k metres
    mapSize /= 100
    length = int(mapSize * 3000)
    
    # generate the landscape
    landscape = Landscape(length, 
                      length,
                      gridSparseness=0.4,
                      gridCoverage=0.6)
    
    # if the user wants to load a city, do that, otherwise populate the existing landscape
    if selectedIndex == -1:
        landscape.generate()
    else:
        landscape.load(f"NewVersion/{cities[selectedIndex]}.txt")
    
    allRoads = []
    factor = 3
    
    # ========================================= ROAD GENERATION =========================================
    for road in landscape.roads:
        roadTuple = (road.int1.coordinates(), road.int2.coordinates(), road.laneCount)
        virtInts = []
        
        for intersection in road.virtualInts:
            if intersection.position != 1:
                # if the intersection is not a real intersection, we don't need to send the traffic light pattern
                virtInts.append(VirtualIntMsg(
                    intersection.id, 
                    0, 
                    0, 
                    intersection.lane, 
                    intersection.direction, 
                    intersection.position,
                    [],
                    0,
                    road.id))
            else:
                # otherwise, we need to send the traffic light pattern too
                associatedInt = intersection.correspondingRealIntersection
                virtInts.append(VirtualIntMsg(
                    intersection.id, 
                    0, 
                    0, 
                    intersection.lane, 
                    intersection.direction, 
                    intersection.position,
                    associatedInt.trafficLightPattern,
                    associatedInt.trafficLightDuration,
                    road.id))
        
        # create a road init message, and scale the coordinates by a factor of 3 to make it look cohesive on the frontend
        allRoads.append(RoadInitMsg(road.id, 
                                    Vector2Message(roadTuple[0][0]*factor, roadTuple[0][1]*factor), 
                                    Vector2Message(roadTuple[1][0]*factor, roadTuple[1][1]*factor), 
                                    roadTuple[2], 
                                    road.speedLimit,
                                    road.capacity,
                                    virtInts))

    # populate the possible starting positions
    allStartingPositions = []
    allEndingPositions = []
    for road in landscape.roads:
        allStartingPositions += road.availablePositions()
        allEndingPositions += road.availablePositions()

    # ========================================= VEHICLE GENERATION =========================================
    # make sure we have >= 1 vehicle

    totalVehicleCount = max(int(vehicleDensity * len(allStartingPositions) / 100 / 100), 1)
    print("Total vehicle count: ", totalVehicleCount)
    autoFlowVehicleCount = max(1, int(totalVehicleCount * autoflow_percentage / 100))
    selfishVehicleCount = totalVehicleCount - autoFlowVehicleCount

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
    
    allVehicles = autoFlowVehicles + selfishVehicles

    # # ========================================= ROUTE COMPUTATION =========================================

    allRoutes = computeRoutes(selfishVehicles, autoFlowVehicles, landscape)
    for vehicle in allVehicles:
        route = []
        
        # skip it if the algorithm could not find a route
        if allRoutes[vehicle.id] == []:
            continue
        
        # do the same as we did for the roads: if the intersection is not a real intersection, we don't need to send the traffic light pattern
        for intersection in allRoutes[vehicle.id]:
            if intersection.position != 1:
                route.append(VirtualIntMsg(
                    intersection.id, 
                    0, 
                    0, 
                    intersection.lane, 
                    intersection.direction, 
                    intersection.position,
                    [],
                    0,
                    intersection.road.id))
            else:
                associatedInt = intersection.correspondingRealIntersection
                route.append(VirtualIntMsg(
                    intersection.id, 
                    0, 
                    0, 
                    intersection.lane, 
                    intersection.direction, 
                    intersection.position,
                    associatedInt.trafficLightPattern,
                    associatedInt.trafficLightDuration,
                    intersection.road.id))
        
        vehicle.route = route

    # these are the final vehicles that will be sent to the frontend
    dataclassVehicles = []
    for vehicle in allVehicles:
        
        if vehicle.route == []:
            continue

        # set its type - 0 for conventional, 1 for electric, 2 for bus
        type = 0
        if isinstance(vehicle, ElectricVehicle):
            type = 1
        elif isinstance(vehicle, Bus):
            type = 2
        dataclassVehicles.append(VehicleInitMsg(vehicle.id, type, vehicle.emissionRate, vehicle.routingSystem == "Autoflow", vehicle.passengerCount, vehicle.route[0], vehicle.route))

    print("Dataclass vehicles: ", len(dataclassVehicles))
    allRoads = AllRoads(allRoads, dataclassVehicles)
    print("Ready for connection.")
    
    await websocket.send(allRoads.serialize())
    print("Sent")

async def main():
    async with serve(handleNew, "localhost", PORT):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted")