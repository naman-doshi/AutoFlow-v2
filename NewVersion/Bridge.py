from LandscapeComponents import * 
from AutoFlow import *
import random
import asyncio
import dataclasses
from websockets.exceptions import ConnectionClosedOK
from websockets.server import WebSocketServerProtocol, serve
import json
import websockets

PORT = 8001

landscape = Landscape(1000, 
                      1000,
                      gridSparseness=0.4,
                      gridCoverage=0.6)

landscape.load("NewVersion/sydney.txt")
allRoads = []
allIntersections = []

for road in landscape.roads:
  allRoads.append((road.int1.coordinates(), road.int2.coordinates(), road.laneCount))

lengthShortening = {1: 1, 2: 3, 3: 3}
intersectionLargening = {1: 1, 2: 2, 3: 4}

for intersection in landscape.intersections.values():
    try:
        biggestRoad = max(intersection.connectingRoads, key=lambda x: x.laneCount)
        coords = intersection.coordinates()
        allIntersections.append((coords[0], coords[1], intersectionLargening[biggestRoad.laneCount]))
    except ValueError:
        coords = intersection.coordinates()
        allIntersections.append((coords[0], coords[1], 2))


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
class RoadInitMsg:
    startPos: Vector2Message
    endPos: Vector2Message
    laneCount: int

    def __dict__(self):
        return {
            "startPos": self.startPos.__dict__(),
            "endPos": self.endPos.__dict__(),
            "laneCount": self.laneCount,
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return RoadInitMsg(**d)
    
@dataclasses.dataclass
class AllRoads:
    intersections : list[Vector3Message]
    roads: list[RoadInitMsg]
    
    def __dict__(self):
        return {
            "roads": [road.__dict__() for road in self.roads],
            "intersections": [intersection.__dict__() for intersection in self.intersections]
        }
    
    def serialize(self):
        return json.dumps(self.__dict__())
    
    @staticmethod
    def deserialize(d):
        return allRoads([RoadInitMsg.deserialize(road) for road in d["roads"]])
    
lis = []
factor = 0.4
for road in allRoads:
    lis.append(RoadInitMsg(Vector2Message(road[0][0]*factor, road[0][1]*factor), Vector2Message(road[1][0]*factor, road[1][1]*factor), road[2]))
ints = []
for intersection in allIntersections:
    ints.append(Vector3Message(intersection[0]*factor, intersection[1]*factor, intersection[2]))

allRoads = AllRoads(ints, lis)

print("Ready for connection.")

async def handler(websocket: WebSocketServerProtocol):
  await websocket.send(allRoads.serialize())
  print("Sent")

async def main():
    async with serve(handler, "localhost", PORT):
        await asyncio.Future()  # run forever


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Interrupted")