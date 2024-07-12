"""
This script contains all of the vehicle agent object definitions required for the virtual simulation.

Vehicles are either conventional (run on fossil fuel) or electric (EVs) and contain the following fields:
- emissionRate: carbon emission in g/km, visit https://www.ntc.gov.au/light-vehicle-emissions-intensity-australia
- passengerCount: the number of passengers carried by the vehicle
- routingSystem: the routing system used by the vehicle
"""


# ================ IMPORTS ================
from LandscapeComponents import *

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

    # [direction, lane, position]
    def setLocation(self, virtualIntersection : VirtualIntersection):
        self.starting = virtualIntersection
        road = virtualIntersection.road
        self.road = road  # the road the vehicle is currently on
        self.direction = virtualIntersection.direction  # the direction the vehicle is currently facing
        self.lane = virtualIntersection.lane  # the lane the vehicle is currently on
        self.position = virtualIntersection.position

        if self.direction == 1:
            x = road.int1.x + (road.int2.x - road.int1.x) * self.position
            y = road.int1.y + (road.int2.y - road.int1.y) * self.position
        else:
            x = road.int2.x + (road.int1.x - road.int2.x) * self.position
            y = road.int2.y + (road.int1.y - road.int2.y) * self.position

        self.startRealPosition = (x, y)

    def setDestination(self, virtualIntersection : VirtualIntersection):
        self.ending = virtualIntersection
        road = virtualIntersection.road
        self.destinationRoad = road
        self.destinationDirection = virtualIntersection.direction
        self.destinationLane = virtualIntersection.lane
        self.destinationPosition = virtualIntersection.position

        if self.destinationDirection == 1:
            x = road.int1.x + (road.int2.x - road.int1.x) * self.destinationPosition
            y = road.int1.y + (road.int2.y - road.int1.y) * self.destinationPosition
        else:
            x = road.int2.x + (road.int1.x - road.int2.x) * self.destinationPosition
            y = road.int2.y + (road.int1.y - road.int2.y) * self.destinationPosition
        
        self.desinationRealPosition = (x, y)

    def __deepcopy__(self, memo):
        agentCopy: Vehicle = self.__class__(self.id)
        agentCopy.setLocation(self.road, (self.direction, self.lane, self.position))
        agentCopy.setDestination(self.destinationRoad, (self.destinationDirection, self.destinationLane, self.destinationPosition))
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