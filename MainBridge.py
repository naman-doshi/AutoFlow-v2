import asyncio
from websockets.server import WebSocketServerProtocol, serve
import websockets
from LegacyVersion.Bridge import *
import LegacyVersion.AutoFlowBridgeCompat
from NewVersion.Bridge import *

PORT = 8001


async def handler(websocket: WebSocketServerProtocol):
    # User input
    message = await websocket.recv()
    message = eval(message)
    selectedIndex = int(message["selectedIndex"])
    vehicleDensity = int(message["vehicleDensity"])
    autoflow_percentage = int(message["autoFlowPercent"])
    mapSize = int(message["mapSize"])
    receiveNewDests = message["receiveNewDests"]
    roadBlockage = message["roadBlockage"]
    graphics = int(message["graphics"])
    
    # graphics being true means you want 3D
    cities = ["sydney", "melbourne", "manhattan", "los_angeles", "london", "tokyo"]
    if graphics == 0:
        await handleNew(websocket, selectedIndex, vehicleDensity, autoflow_percentage, mapSize, receiveNewDests, roadBlockage)
    else:
        await handleLegacy(websocket, message)

async def main():
    async with serve(handler, "localhost", PORT):
        await asyncio.Future()  # run forever


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Interrupted")
