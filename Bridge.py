import asyncio
import dataclasses
from websockets.exceptions import ConnectionClosedOK
from websockets.server import WebSocketServerProtocol, serve
import json

from AutoFlowBridgeCompat import outputToBridge
from LandscapeComponents import Landscape
from VehicleAgents import Vehicle
import websockets
import AutoFlowBridgeCompat
import LandscapeComponents
from LandscapeComponents import Road
from AutoFlow import recalculateRoutes

PORT = 8001


@dataclasses.dataclass
class VehicleInitMessage:
    id: int
    initRoadId: int
    position: tuple[float, float]
    rotation: float
    emissionRate: float
    useAutoFlow: bool
    passengerCount: int

    def __dict__(self):
        return {
            "id": self.id,
            "initRoadId": self.initRoadId,
            "position": self.position,
            "rotation": self.rotation,
            "emissionRate": self.emissionRate,
            "useAutoFlow": self.useAutoFlow,
            "passengerCount": self.passengerCount,
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return VehicleInitMessage(**d)


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
class RoadInitMessage:
    id: int
    speedLimit: float
    startPos: Vector2Message
    endPos: Vector2Message
    neighbors: list[int]

    def __dict__(self):
        return {
            "id": self.id,
            "speedLimit": self.speedLimit,
            "startPos": self.startPos.__dict__(),
            "endPos": self.endPos.__dict__(),
            "neighbors": self.neighbors,
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return RoadInitMessage(**d)


@dataclasses.dataclass
class IntersectionMessage:
    id: Vector2Message
    enterRoadIDs: list[int]
    exitRoadIDs: list[int]
    pattern: list[int]
    greenDuration: float

    def __dict__(self):
        return {
            "id": self.id.__dict__(),
            "enterRoadIDs": self.enterRoadIDs,
            "exitRoadIDs": self.exitRoadIDs,
            "pattern": self.pattern,
            "greenDuration": self.greenDuration,
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return IntersectionMessage(**d)


@dataclasses.dataclass
class InitMessage:
    tiles: list[str]
    rowWidth: int
    vehicles: list[VehicleInitMessage]
    roads: list[RoadInitMessage]
    intersections: list[IntersectionMessage]

    def __dict__(self):
        return {
            "tiles": self.tiles,
            "rowWidth": self.rowWidth,
            "vehicles": [v.__dict__() for v in self.vehicles],
            "roads": [r.__dict__() for r in self.roads],
            "intersections": [i.__dict__() for i in self.intersections],
            "type": "InitMessage",
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return InitMessage(**d)


@dataclasses.dataclass
class VehicleUpdateMessage:
    id: int
    route: list[Vector3Message]

    def __dict__(self):
        return {
            "id": self.id,
            "route": [r.__dict__() for r in self.route],
            "type": "VehicleUpdateMessage",
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return VehicleInitMessage(**d)


@dataclasses.dataclass
class UpdateMessage:
    updates: list[VehicleUpdateMessage]

    def __dict__(self):
        return {
            "updates": [u.__dict__() for u in self.updates],
            "type": "UpdateMessage",
        }

    def serialize(self):
        return json.dumps(self.__dict__())

    @staticmethod
    def deserialize(d):
        return VehicleInitMessage(**d)


class JSONUtils:
    @staticmethod
    def deserialize(text: str):
        d: dict = json.loads(text)
        typ = d.pop("type")

        match typ:
            case "InitMessage":
                return InitMessage.deserialize(d)
            case "VehicleMessage":
                return VehicleInitMessage.deserialize(d)
            case _:
                raise ValueError("Invalid type from JSON")


async def handler(websocket: WebSocketServerProtocol):
    # User input
    message = await websocket.recv()
    message = eval(message)
    vehicleDensity = int(message["vehicleDensity"])
    autoflow_percentage = int(message["autoFlowPercent"])
    mapSize = int(message["mapSize"])
    landscape, MAX_ROAD_SPEED_MPS, TOTAL_VEHICLE_COUNT, allVehicles = AutoFlowBridgeCompat.generateLandscape(mapSize, vehicleDensity)
    update_interval = 1

    inp: tuple[
        dict[int, tuple[float, float, Vehicle]],
        Landscape,
        dict[int, list[tuple[float, float, float]]],
        list[Vehicle],
    ] = outputToBridge(autoflow_percentage, allVehicles, landscape, MAX_ROAD_SPEED_MPS, TOTAL_VEHICLE_COUNT)

    autoflow_vehicles = []
    selfish_vehicles = []
    for vehicle in inp[3]:
        if vehicle.routingSystem == "Autoflow":
            autoflow_vehicles.append(vehicle)
        else:
            selfish_vehicles.append(vehicle)


    # Initial scene
    vehicleInits = []

    # creating a dummy vehicle init for the update interval
    vehicleInits.append(
        VehicleInitMessage(
            update_interval,
            0,
            (0, 0),
            0,
            0,
            False,
            0,
        )
    )


    for id, posAndVeh in inp[0].items():
        vehicleInits.append(
            VehicleInitMessage(
                id,
                posAndVeh[2].road.roadID,
                (posAndVeh[0], posAndVeh[1]),
                0,
                posAndVeh[2].emissionRate,
                posAndVeh[2].routingSystem == "Autoflow",
                posAndVeh[2].passengerCount,
            )
        )

    flatLandscapeMatrix = []
    for row in inp[1].landscapeMatrix:
        flatLandscapeMatrix.extend(row)

    roadMessages = []
    AutoFlowBridgeCompat.populateRoadNeighbors(inp[1])
    print('Populated road neighbors.')
    for road in inp[1].roads:
        roadMessages.append(
            RoadInitMessage(
                road.roadID,
                road.speedLimit_MPS,
                Vector2Message(road.startPosReal[0], road.startPosReal[1]),
                Vector2Message(road.endPosReal[0], road.endPosReal[1]),
                road.neighbors,
            )
        )

    intersections = [
        IntersectionMessage(Vector2Message(i[0][0], i[0][1]), i[1], i[2], i[3], i[4])
        for i in inp[1].unityCache
    ]

    await websocket.send(
        InitMessage(
            flatLandscapeMatrix,
            len(inp[1].landscapeMatrix[0]),
            vehicleInits,
            roadMessages,
            intersections,
        ).serialize()
    )
    print("Finished terrain")
    await asyncio.sleep(0.5)

    # Updates
    updateMessages = []
    for id, route in inp[2].items():
        currentCar = VehicleUpdateMessage(id, [Vector3Message(*r) for r in route])
        updateMessages.append(currentCar)
    await websocket.send(UpdateMessage(updateMessages).serialize())

    print("Finished routing")

    await asyncio.sleep(9.5)

    while True:
        try:
            message = await websocket.recv()
            
            # horrible security
            carPositions = eval(message)
            updateMessages = []

            newRoutes = recalculateRoutes(carPositions, inp[1], autoflow_vehicles + selfish_vehicles, MAX_ROAD_SPEED_MPS, update_interval)

            for k in newRoutes.keys():
                currentCar = VehicleUpdateMessage(k, [Vector3Message(*r) for r in newRoutes[k]])
                updateMessages.append(currentCar)

            await websocket.send(UpdateMessage(updateMessages).serialize())
            
        except websockets.exceptions.ConnectionClosedOK:
            print("Connection closed, stopping reception.")
            break

        await asyncio.sleep(update_interval)



async def main():
    async with serve(handler, "localhost", PORT):
        await asyncio.Future()  # run forever


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Interrupted")
