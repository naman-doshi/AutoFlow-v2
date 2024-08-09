import asyncio
from websockets.server import WebSocketServerProtocol, serve
import websockets


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
    cities = {0: 'sydney', 1: 'melbourne', 2: 'manhattan', 3: 'los_angeles', 4: 'london', 5: 'tokyo'}
    if selectedIndex != -1:
        city = cities[selectedIndex]
        print(city)
    else:
        print("No city selected")


    



async def main():
    async with serve(handler, "localhost", PORT):
        await asyncio.Future()  # run forever


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Interrupted")
