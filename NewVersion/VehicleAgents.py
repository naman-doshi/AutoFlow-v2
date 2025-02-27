"""
This script contains all of the vehicle agent object definitions required for the virtual simulation.

Vehicles are either conventional (run on fossil fuel) or electric (EVs) and contain the following fields:
- emissionRate: carbon emission in g/km, visit https://www.ntc.gov.au/light-vehicle-emissions-intensity-australia
- passengerCount: the number of passengers carried by the vehicle
- routingSystem: the routing system used by the vehicle
"""


# ================ IMPORTS ================
from NewVersion.LandscapeComponents import *

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
        self.route = []
        self.starting = []
        self.ending = []
        self.startingRoad = 0
        self.endingRoad = 0

    def setRoutingSystem(self, systemID: int):
        self.routingSystem = Vehicle.routingSystems[systemID]

    # [direction, lane, position]

    def __deepcopy__(self, memo):
        agentCopy: Vehicle = self.__class__(self.id)
        agentCopy.emissionRate = self.emissionRate
        agentCopy.passengerCount = self.passengerCount
        agentCopy.route = self.route.copy()
        agentCopy.starting = self.starting.copy()
        agentCopy.ending = self.ending.copy()
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
        self.route = []


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
        self.route = []

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
            self.route = []