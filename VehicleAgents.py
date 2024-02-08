"""
This script contains all of the vehicle agent object definitions required for the virtual simulation.

Vehicles are either conventional (run on fossil fuel) or electric (EVs) and contain the following fields:
- emissionRate: carbon emission in g/km, visit https://www.ntc.gov.au/light-vehicle-emissions-intensity-australia
- passengerCount: the number of passengers carried by the vehicle
- routingSystem: the routing system used by the vehicle
"""


# ================ IMPORTS ================
from LandscapeComponents import Road

from abc import *
from random import randint

# =========================================


class Vehicle(ABC):

    """
    Virtual representation of a vehicle on the map.
    """

    routingSystems = {0: "Selfish", 1: "Autoflow"}  # expandable
    MAX_EMISSION_RATE = 250
    MAX_PASSENGER_COUNT = 6
    cost = 1

    @abstractmethod
    def __init__(self) -> None:
        self.emissionRate = 0
        self.passengerCount = 1

    def setRoutingSystem(self, systemID: int):
        self.routingSystem = Vehicle.routingSystems[systemID]

    def setLocation(self, road: Road, position: float):
        self.road = road  # the road the vehicle is currently on
        self.position = (
            position  # float between 0 and 1 indicating linear position along a road
        )
        road.vehicleStack.append(self)

        realPositionX = float(road.endPosReal[0] - road.startPosReal[0]) * position + road.startPosReal[0]
        realPositionY = float(road.endPosReal[1] - road.startPosReal[1]) * position + road.startPosReal[1]
        self.startRealPosition = (realPositionX, realPositionY)

    def setDestination(self, road: Road, position: float):
        self.destinationRoad = road
        self.destinationPosition = position
        realPositionX = (road.endPosReal[0] - road.startPosReal[0]) * position + road.startPosReal[0]
        realPositionY = (road.endPosReal[1] - road.startPosReal[1]) * position + road.startPosReal[1]
        self.destinationRealPosition = (realPositionX, realPositionY)

    def __deepcopy__(self, memo):
        agentCopy: Vehicle = self.__class__(self.id)
        agentCopy.setLocation(self.road, self.position)
        agentCopy.setDestination(self.destinationRoad, self.destinationPosition)
        agentCopy.routingSystem = self.routingSystem
        return agentCopy


class ConventionalVehicle(Vehicle):

    """
    Conventional vehicles run on fossil fuel, therefore their emission rate is positive.
    The vehicle's carbon emission per km is represented by its emissionRate field.

    Emission rate ranges from 100g/km to 250g/km for conventional vehicles.
    See https://www.ntc.gov.au/light-vehicle-emissions-intensity-australia for emission rate standards.
    """

    def __init__(
        self, id, useAutoFlow: bool = False
    ) -> None:
        self.emissionRate = randint(100, Vehicle.MAX_EMISSION_RATE)
        #self.emissionRate = 150
        self.passengerCount = randint(1, Vehicle.MAX_PASSENGER_COUNT)
        #self.passengerCount = 1
        self.setRoutingSystem(int(useAutoFlow))
        self.id = id


class ElectricVehicle(Vehicle):

    """
    Electric vehicles run on electricity, therefore their emission rate is zero.
    """

    def __init__(
            self, id, useAutoFlow: bool = False
    ) -> None:
        self.id = id
        self.emissionRate = 0
        self.passengerCount = randint(1, Vehicle.MAX_PASSENGER_COUNT)
        #self.passengerCount = 1
        self.setRoutingSystem(int(useAutoFlow))

class Bus(Vehicle):
    
        """
        Buses are a special type of vehicle that can carry up to 70 passengers.
        Emissions lie in a range around 800g/km.
        https://www.carbonindependent.org/20.html
        """

        cost = 2
    
        def __init__(
                self, id, useAutoFlow: bool = False
        ) -> None:
            self.id = id
            self.emissionRate = randint(600, 1000)
            #self.emissionRate = 150
            self.passengerCount = randint(20, 70)
            self.setRoutingSystem(int(useAutoFlow))