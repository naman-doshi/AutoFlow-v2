"""
This script contains all of the environment object definitions required for virtual landscape creation.

Landscape component hierarchy:
- Landscape
    - LandPlots
    - Intersections (including traffic light components)
    - Roads

An original algorithm for generating random landscapes is included. Below is the outline:
1. An empty cell matrix is gradually populated with rectangles that represent land plots
2. Roads and intersections are formed on the perimeter of every distinct land plot
3. All roads and intersections are joined together to form a connected road network

A landscape matrix describing the generated landscape is produced, as well as a directed graph representation
of the road network that the Autoflow algorithm operates on in order to perform its multi-agent path searches.

Three global constants (static variables) are defined:
- CELL_SIZE_METRES: real scaling of each cell in metres
- VEHICLE_LENGTH_METRES: how much space does each vehicle need in their lane
- ACTIVE_LANDSCAPE: global reference to the currently used landscape
"""


# ================ IMPORTS ================
from collections import defaultdict, deque
from random import randint, shuffle
from copy import deepcopy

# =========================================

# =============== CONSTANTS ===============
CELL_SIZE_METRES: int = 20
VEHICLE_LENGTH_METRES: int = 5
ACTIVE_LANDSCAPE: "Landscape" = None
# =========================================


class LandPlot:

    """
    A land plot is a rectangular area where no roads can spawn.
    The size of a land plot is (number of cells taken in x axis) * (number of cells taken in y axis).
    The coordinates of each land plot must be unique.

    NOTE: The properties may become invalid after a new landscape is generated via generate_new_landscape,
    due to the road fitting algorithm giving the roads extra cells (as they have a physical width and height).
    However, this has no implications on the simulation as buildings are static environment components.
    """

    def __init__(self, xSize: int, ySize: int) -> None:
        # Set size of the land plot in number of cells
        self.xSize = xSize
        self.ySize = ySize

    def set_coordinate(self, xPos: int, yPos: int) -> None:
        # Set lower left coordinates of the land plot
        self.xPos = xPos
        self.yPos = yPos

    def area(self) -> int:
        return self.xSize * self.ySize

    def __hash__(self) -> int:
        return hash((self.xPos, self.yPos))


class Intersection:

    """
    An intersection is a point where two or more roads cross.
    If three or more roads cross, a traffic light is spawned with a random signal pattern.

    Spawned at all four corners of a land plot.
    The coordinate (xPos, yPos) must be unique for each intersection.

    Each intersection also contains a neighbours list that points to all its adjacent neighbours (up to four).
    """

    def __init__(self, xPos: float, yPos: float) -> None:
        # Set coordinates of the intersection
        self.xPos = xPos
        self.yPos = yPos

        # Initiate neighbour references
        self.neighbours: list[Intersection] = []

        # Traffic light info
        self.trafficLightPattern: list[int] = None
        self.trafficLightDuration: int = -1  # how long each phase lasts, in seconds

        # Lookup table for quickly accessing the when a particular road gets the green light
        self.trafficLightLookup: dict[Intersection, int] = {}

        # Hashmap for quickly accessing how many vehicles can pass through in a single green light
        self.trafficPassthroughRate: dict[Intersection, int] = {}
        # Maps (intersection where the from-road starts) to number of vehicles that can pass through

        # Pseudo roads for joining roads at an intersection
        self.intersectionPathways: dict[
            Intersection, dict[Intersection, Road]
        ] = defaultdict(dict)
        # Maps (intersection where the from-road starts) to (intersection where the to-road ends)

    def coordinates(self) -> tuple[int, int]:
        return (self.xPos, self.yPos)

    def create_traffic_light(self) -> None:
        """
        Creates a random traffic signal pattern based on self.neighbours.

        The pattern is represented by a list where:
        - the index of the list is the modulus time i.e. current time % number of roads at this intersection
        - the value at that index is the index of the road which starts at self.neighbours[value]

        For example, self.neighbours[self.trafficLightPattern[1]] yields the intersection from where
        the road that gets the green light at every 2nd phase of the traffic light pattern starts.

        A pseudo road between every connectable pair of road joined at the intersection is also created,
        i.e. there will be 6 pseudo roads for a three way intersection, and 12 for a four way intersection.
        NOTE: U turns are NOT supported.

        Pseudo roads are stored via self.intersectionPathways, which maps the intersection from where
        the road that gets the green light starts to the intersection to to where the next road goes/ends.
        This ensures that the mappings are unique.
        """

        roadCount = len(self.neighbours)

        if (
            roadCount >= 3
        ):  # traffic lights are only needed for intersections with 3 or more roads
            # Create random green light order
            self.trafficLightPattern = [i for i in range(roadCount)]
            shuffle(self.trafficLightPattern)  # randomise green light order

            # Create lookup table based on green light order
            for i in range(roadCount):
                self.trafficLightLookup[
                    self.neighbours[self.trafficLightPattern[i]]
                ] = i

            # Randomise phase duration between 3-8 seconds
            self.trafficLightDuration = randint(3, 8)

            # Calculate traffic passthrough rate for every road
            for intersection in self.neighbours:
                self.trafficPassthroughRate[intersection] = (
                    ACTIVE_LANDSCAPE.roadmap[intersection.coordinates()][
                        self.coordinates()
                    ].speedLimit_MPS
                    * self.trafficLightDuration
                    // VEHICLE_LENGTH_METRES
                )

        # Create virtual pathways
        for from_intersection in self.neighbours:
            for to_intersection in self.neighbours:
                if to_intersection.coordinates() != from_intersection.coordinates():
                    pseudoPathway = Road(self.coordinates(), self.coordinates())

                    # Set the starting and ending positions of the virtual pathway
                    pseudoPathway.startPosReal = ACTIVE_LANDSCAPE.roadmap[
                        from_intersection.coordinates()
                    ][self.coordinates()].endPosReal
                    pseudoPathway.endPosReal = ACTIVE_LANDSCAPE.roadmap[
                        self.coordinates()
                    ][to_intersection.coordinates()].startPosReal

                    # Calculate real length and direction of the virtual pathway
                    if (
                        ACTIVE_LANDSCAPE.roadmap[from_intersection.coordinates()][
                            self.coordinates()
                        ].direction
                        != ACTIVE_LANDSCAPE.roadmap[self.coordinates()][
                            to_intersection.coordinates()
                        ].direction
                    ):  # a turn is required
                        pseudoPathway.length = (
                            (
                                pseudoPathway.startPosReal[0]
                                - pseudoPathway.endPosReal[0]
                            )
                            ** 2
                            + (
                                pseudoPathway.startPosReal[1]
                                - pseudoPathway.endPosReal[1]
                            )
                            ** 2
                        ) ** 0.5
                        pseudoPathway.direction = (
                            ACTIVE_LANDSCAPE.roadmap[from_intersection.coordinates()][
                                self.coordinates()
                            ].direction
                            + ACTIVE_LANDSCAPE.roadmap[self.coordinates()][
                                to_intersection.coordinates()
                            ].direction
                        )  # simply join the directions e.g. north road to east road => NE
                    else:  # go straight, no turn required
                        pseudoPathway.length = CELL_SIZE_METRES
                        pseudoPathway.direction = ACTIVE_LANDSCAPE.roadmap[
                            from_intersection.coordinates()
                        ][
                            self.coordinates()
                        ].direction  # same direction

                    # Asign speed limit of the virtual pathway
                    pseudoPathway.set_speed_limit(
                        (
                            ACTIVE_LANDSCAPE.roadmap[from_intersection.coordinates()][
                                self.coordinates()
                            ].speedLimit
                            + ACTIVE_LANDSCAPE.roadmap[self.coordinates()][
                                to_intersection.coordinates()
                            ].speedLimit
                        )
                        / 2
                    )  # set speed limit of pathway to the average of the two connecting roads

                    # Calculate real traversal time and max vehicle count
                    pseudoPathway.calculate_traversal_time()
                    pseudoPathway.calculate_max_vehicle_count()

                    self.intersectionPathways[from_intersection][
                        to_intersection
                    ] = pseudoPathway

    def __hash__(self) -> int:
        return hash((self.xPos, self.yPos))


class Road:

    """
    A road is one way lane between two intersections, or an intersection and one side of the map border.
    By default, two roads going opposite directions are constructed for every group of road tiles.

    Each road has the following properties:
    - Starting coordinate and ending coordinate
    - Length in metres
    - Speed limit in km/h

    The road ALWAYS heads towards the ending coordinate.
    Length is simply the number of tiles the road spans over multiplied by the cell size constant.
    One is subtracted due to the fact that the road starts and ends at the centre of two intersections.
    """

    offset = 0.1  # offsets vehicles towards the middle line of a road tile, ranges from 0 to 0.25
    available_starting_positions = []
    available_ending_positions = []
    neighbors = []

    def __init__(self, start: tuple[int, int], end: tuple[int, int]) -> None:
        self.open = True
        
        self.roadID: int = (
            None  # index within ACTIVELANDSCAPE.roads, assigned by ACTIVELANDSCAPE
        )

        # Set starting intersection and ending intersection of the road
        self.start = start
        self.end = end

        # Set direction as NESW
        self.set_direction()

        # Initiate real starting & ending positions
        self.startPosReal = (
            (start[0] + 0.5) * CELL_SIZE_METRES,
            (start[1] + 0.5) * CELL_SIZE_METRES,
        )
        self.endPosReal = (
            (end[0] + 0.5) * CELL_SIZE_METRES,
            (end[1] + 0.5) * CELL_SIZE_METRES,
        )

        # Adjust real starting & ending positions according to direction
        if self.direction == "N":
            self.startPosReal = (
                self.startPosReal[0]
                + (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
                self.startPosReal[1] + CELL_SIZE_METRES / 2,
            )
            self.endPosReal = (
                self.endPosReal[0]
                + (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
                self.endPosReal[1] - CELL_SIZE_METRES / 2,
            )
        elif self.direction == "S":
            self.startPosReal = (
                self.startPosReal[0]
                - (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
                self.startPosReal[1] - CELL_SIZE_METRES / 2,
            )
            self.endPosReal = (
                self.endPosReal[0]
                - (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
                self.endPosReal[1] + CELL_SIZE_METRES / 2,
            )
        elif self.direction == "E":
            self.startPosReal = (
                self.startPosReal[0] + CELL_SIZE_METRES / 2,
                self.startPosReal[1]
                - (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
            )
            self.endPosReal = (
                self.endPosReal[0] - CELL_SIZE_METRES / 2,
                self.endPosReal[1]
                - (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
            )
        elif self.direction == "W":
            self.startPosReal = (
                self.startPosReal[0] - CELL_SIZE_METRES / 2,
                self.startPosReal[1]
                + (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
            )
            self.endPosReal = (
                self.endPosReal[0] + CELL_SIZE_METRES / 2,
                self.endPosReal[1]
                + (CELL_SIZE_METRES / 4 - CELL_SIZE_METRES * Road.offset),
            )

        # Set real length and speed limit
        self.cellSpan = (abs(end[0] - start[0]) + abs(end[1] - start[1])) - 1
        self.length = self.cellSpan * CELL_SIZE_METRES
        self.set_speed_limit()

        # Calculate traversal time in seconds i.e. how many seconds does it take to traverse this road
        self.calculate_traversal_time()

        # Initialise vehicle stack, which would store a list of Vehicles
        self.vehicleStack = deque()
        # NOTE: At least one vehicle would be allowed on a road, to handle two adjacent intersections

        # Calculate the maximum number of vehicles that can be on this road at the same time
        self.calculate_max_vehicle_count()

        # Create a normalised position table for each coordinate along this road
        self.positionTable: dict[tuple[int, int], float] = {}

        # Compute position table
        i = 0.5  # vehicles cannot start at the beginning or end of any road
        if self.direction == "N":
            for yCoord in range(start[1] + 1, end[1]):
                self.positionTable[(start[0], yCoord)] = 1 / self.cellSpan * i
                i += 1
        elif self.direction == "S":
            for yCoord in range(start[1] - 1, end[1], -1):
                self.positionTable[(start[0], yCoord)] = 1 / self.cellSpan * i
                i += 1
        elif self.direction == "E":
            for xCoord in range(start[0] + 1, end[0]):
                self.positionTable[(xCoord, start[1])] = 1 / self.cellSpan * i
                i += 1
        elif self.direction == "W":
            for xCoord in range(start[0] - 1, end[0], -1):
                self.positionTable[(xCoord, start[1])] = 1 / self.cellSpan * i
                i += 1

    def set_direction(self):
        if self.start[0] == self.end[0]:  # vertical road
            if self.start[1] < self.end[1]:
                self.direction = "N"  # heading north
            else:
                self.direction = "S"  # heading south
        else:  # horizontal road
            if self.start[0] < self.end[0]:
                self.direction = "E"  # heading east
            else:
                self.direction = "W"  # heading west

    # this method should be overwritten by subclasses of road, e.g. highway => 120km/h
    def set_speed_limit(self, speedlimit: float = 60) -> None:
        self.speedLimit = speedlimit
        self.speedLimit_MPS = speedlimit * 1000 / 3600

    def calculate_traversal_time(self) -> None:  # calculates traversal time in seconds
        self.traversalTime = self.length / self.speedLimit_MPS

    def calculate_max_vehicle_count(
        self,
    ) -> None:  # calculates the maximum number of cars that can physically fit
        self.maxVehicleCount = max(1, self.length // VEHICLE_LENGTH_METRES)

    def is_within_bounds(self, x, y) -> bool:
        if self.startPosReal[0] == self.endPosReal[0]:
            # road is going north or south
            return y <= max(self.startPosReal[1], self.endPosReal[1]) or y >= min(self.startPosReal[1], self.endPosReal[1])
        if self.startPosReal[1] == self.endPosReal[1]:
            # road is going east or west
            return x <= max(self.startPosReal[0], self.endPosReal[0]) or x >= min(self.startPosReal[0], self.endPosReal[0])
        
    def get_position(self, x, y) -> float:
        """
        Returns the normalised position of a given coordinate on this road.
        """
        if self.length == 0:
            return 0
        if self.startPosReal[0] == self.endPosReal[0]:
            # road is going north or south
            return round((y - min(self.startPosReal[1], self.endPosReal[1])) / self.length, 3)
        if self.startPosReal[1] == self.endPosReal[1]:
            # road is going east or west
            return round((x - min(self.startPosReal[0], self.endPosReal[0])) / self.length, 3)


class LandPlotDescriptor:

    """
    Used to describe desirable "features" of a virtual landscape.
    Features e.g. residental rows are described through their corresponding land plot sizes.
    A list of LandPlotDescriptor will be passed to the landscape generator algorithm for virtual landscape generation.

    Assuming that the generated landscape faces North:
    - xRange is the horizontal width (west to east) of the land plot
    - yRange is the vertical length (south to north) of the land plot
    - randomOrientation indicates whether x and y can be swapped when generating the specified landplot
    """

    def __init__(
        self,
        xRange: tuple[int, int],
        yRange: tuple[int, int],
        randomOrientation: bool = True,
    ) -> None:
        # Set size range of x and size range of y in number of cells
        self.xRange = xRange
        self.yRange = yRange

        # Whether x and y can be swapped (randomly) when generating landscape
        self.randomOrientation = randomOrientation



class Landscape:

    """
    A landscape is a grid structure that contains straight roads and rectangular land plots.
    The size of the landscape is (number of cells fittable in x axis) * (number of cells fittable in y axis).
    The real distance covered by each (square) cell on either axes is defined by cellSize in metres.
    This object represents an entire virtual landscape, storing references to all components.

    NOTE: x is the horizontal (west to east) coordinate, y is the vertical (south to north) coordinate,
    therefore y is indexed first (row in matrix), then x (column in matrix).

    NOTE: Landscapes generated by the generate_new_landscape method may have up to double the xSize and ySize,
    due to the road fitting algorithm giving the roads extra cells (as they have a physical width and height).
    """

    def __init__(self, xSize: int, ySize: int, cellSize: int = 100) -> None:
        # Set size of the generated landscape in cell count, e.g. 10 x 10 cells with a cellSize of 100 => 1km^2 area
        self.xSize = xSize
        self.ySize = ySize
        cellSize = cellSize
        self.lookupRoad : dict[int, Road] = {}

        # Initiate component references
        self.reset_landscape()

        # Set newly created landscape to global reference
        global ACTIVE_LANDSCAPE
        ACTIVE_LANDSCAPE = self

        self.availableStarts = []
        self.availableEnds = []

    # def get_neighbors(self, node):
    #     """
    #     Get neighboring cells of a given node for pathfinding.
    #     """
    #     x, y = node
    #     neighbors = []

    #     # Define possible movement directions (up, down, left, right)
    #     directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    #     for dx, dy in directions:
    #         new_x, new_y = x + dx, y + dy

    #         # Check if the new coordinates are within the landscape bounds
    #         if 0 <= new_x < self.xSize and 0 <= new_y < self.ySize:
    #             neighbors.append((new_x, new_y))

    #     return neighbors

    # def get_cost(self, current, neighbor):
    #     """
    #     Calculate the cost to move from the current cell to the neighbor cell.
    #     In this example, you can use the Euclidean distance as the cost.
    #     """
    #     x1, y1 = current
    #     x2, y2 = neighbor

    #     # Calculate Euclidean distance
    #     distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    #     # Optionally, you can consider road speed limits or other factors
    #     # For example, you can access road speed limits using self.roadmap

    #     # For now, return the Euclidean distance as the cost
    #     return distance

    


    def reset_landscape(self) -> None:
        # 2D matrix representing the COMPACT land plot allocations i.e. no roads, x and y are cell coordinates
        self.gridMatrix: list[list[tuple[int, int]]] = [
            [None for i in range(self.xSize)] for j in range(self.ySize)
        ]

        # Counter for the amount of available area remaing in gridMatrix
        self.availableArea = self.xSize * self.ySize

        # 2D matrix representing the generated landscape, x and y are cell coordinates
        self.landscapeMatrix: list[list[str]] = [
            [None for i in range(self.xSize)] for j in range(self.ySize)
        ]

        # Component references
        self.landPlots: list[LandPlot] = []
        self.landMap: dict[tuple[int, int], LandPlot] = {}  # coord: land plot
        self.roads: list[Road] = []
        self.intersections: dict[
            tuple[int, int], Intersection
        ] = {}  # coord: intersection

        # Hashmap for accessing roads via intersection coordinates, e.g. self.roadmap[intersection1][intersection2]
        self.roadmap: dict[tuple[int, int], dict[tuple[int, int], Road]] = defaultdict(
            dict
        )

        # Hashmap for accessing the road of a particular coordinate, first Road has smaller coords than second Road
        self.coordToRoad: dict[tuple[int, int], tuple[Road, Road]] = {}
        # i.e. first Road goes west or south, second Road goes east or north

    @staticmethod
    def generate_features(
        desiredFeatures: list[tuple[LandPlotDescriptor, int]]
    ) -> list[LandPlot]:
        """
        Generates and returns a list of feature groups from a list of LandPlotDescriptor and max occurrence counts.
        Each feature group represents a desired feature.
        """

        generatedFeatures: list[LandPlot] = []

        for desiredFeature, maxOccurrenceCount in desiredFeatures:
            if maxOccurrenceCount <= 0:  # must be positive
                continue

            for i in range(maxOccurrenceCount):
                # Generate land plot feature
                if desiredFeature.randomOrientation == True:
                    # Randomly swap the x and y sizes
                    if randint(0, 1):
                        feature = LandPlot(
                            randint(*desiredFeature.xRange),
                            randint(*desiredFeature.yRange),
                        )
                    else:
                        feature = LandPlot(
                            randint(*desiredFeature.yRange),
                            randint(*desiredFeature.xRange),
                        )
                else:
                    feature = LandPlot(
                        randint(*desiredFeature.xRange), randint(*desiredFeature.yRange)
                    )

                # Add feature to generatedFeatures list
                generatedFeatures.append(feature)

        return generatedFeatures

    def get_valid_placements(self, feature: LandPlot) -> list[tuple[int, int]]:
        """
        Returns a list of all possible bottom left coordinates where the feature land plot can fit within self.gridMatrix.
        """

        valid_coordinates: list[tuple[int, int]] = []

        # Check every possibly valid placement coordinate
        for ycoord in range(self.ySize - feature.ySize + 1):
            for xcoord in range(self.xSize - feature.xSize + 1):
                # Assume current placement coordinate is valid
                valid = True

                # Check every cell covered by the feature land plot
                for y in range(ycoord, ycoord + feature.ySize):
                    for x in range(xcoord, xcoord + feature.xSize):
                        # cell has already been taken by another land plot
                        if self.gridMatrix[y][x] != None:
                            valid = False
                            break

                    if not valid:
                        break

                # Append valid coordinates to list
                if valid:
                    valid_coordinates.append((xcoord, ycoord))

        return valid_coordinates

    def place_feature(self, feature: LandPlot, xPos: int, yPos: int) -> None:
        """
        Places a feature land plot within self.gridMatrix.
        """

        feature.set_coordinate(xPos, yPos)

        # Mark every cell covered by the feature land plot as taken
        for y in range(yPos, yPos + feature.ySize):
            for x in range(xPos, xPos + feature.xSize):
                self.gridMatrix[y][x] = (xPos, yPos)
                self.availableArea -= 1

    def connect_intersections(
        self, intersection1: Intersection, intersection2: Intersection
    ) -> None:
        """
        Connects two intersections by creating two roads that go in opposite directions.
        Then updates self.coordToRoad for vehicle agents to easily locate the road they're on.
        """

        # Generate random speed limit for both roads
        random_speed_limit = randint(40, 80)

        # Add intersection to component references
        self.intersections[intersection1.coordinates()] = intersection1
        self.intersections[intersection2.coordinates()] = intersection2

        # Create road from intersection 1 to intersection 2
        intersection1.neighbours.append(intersection2)
        road1 = Road(intersection1.coordinates(), intersection2.coordinates())
        road1.set_speed_limit(random_speed_limit)
        self.roadmap[intersection1.coordinates()][intersection2.coordinates()] = road1
        self.roads.append(road1)

        # Create road from intersection 2 to intersection 1
        intersection2.neighbours.append(intersection1)
        road2 = Road(intersection2.coordinates(), intersection1.coordinates())
        road2.set_speed_limit(random_speed_limit)
        self.roadmap[intersection2.coordinates()][intersection1.coordinates()] = road2
        self.roads.append(road2)

        # Update self.coordToRoad to map every cell covered by road to road
        if intersection1.yPos == intersection2.yPos:  # horizontally alligned
            if (
                intersection1.xPos < intersection2.xPos
            ):  # intersection 1 is to the left of intersection 2
                roadWest, roadEast = road2, road1
            else:  # intersection 2 is to the left of intersection 1
                roadWest, roadEast = road1, road2

            for xCoord in range(intersection1.xPos + 1, intersection2.xPos):
                self.coordToRoad[(xCoord, intersection1.yPos)] = (roadWest, roadEast)

        else:  # vertically alligned
            if (
                intersection1.yPos < intersection2.yPos
            ):  # intersection 1 is to the south of intersection 2
                roadSouth, roadNorth = road2, road1
            else:  # intersection 2 is to the south of intersection 1
                roadSouth, roadNorth = road1, road2

            for yCoord in range(intersection1.yPos + 1, intersection2.yPos):
                self.coordToRoad[(intersection1.xPos, yCoord)] = (roadSouth, roadNorth)

    def label_intersections(self, pos1: tuple[int, int], pos2: tuple[int, int]):
        """
        Labels intersections on self.landscapeMatrix, as well as the roads in between.
        Does NOT create any intersection or road objects.
        """

        # Add intersection labels to self.landscapeMatrix
        self.landscapeMatrix[pos1[1]][pos1[0]] = "IS"
        self.landscapeMatrix[pos2[1]][pos2[0]] = "IS"

        # Add road labels between the two intersections to self.landscapeMatrix
        if pos1[1] == pos2[1]:  # horizontal roads
            for xCoord in range(min(pos1[0], pos2[0]) + 1, max(pos1[0], pos2[0])):
                if self.landscapeMatrix[pos1[1]][xCoord] == None:
                    self.landscapeMatrix[pos1[1]][xCoord] = "HR"
        else:  # vertical roads
            for yCoord in range(min(pos1[1], pos2[1]) + 1, max(pos1[1], pos2[1])):
                if self.landscapeMatrix[yCoord][pos1[0]] == None:
                    self.landscapeMatrix[yCoord][pos1[0]] = "VR"

    def generate_new_landscape(
        self,
        desiredFeatures: list[tuple[LandPlotDescriptor, int]] = [],
        filler: LandPlotDescriptor = LandPlotDescriptor((1, 1), (1, 1), False),
    ) -> None:
        """
        Algorithm for generating a random new landscape based on a list of desired features.

        The following optional parameters may be passed:
        - desiredFeatures: a list of (LandPlotDescriptor, maxCount) describing desired features and maximum occurrence
        - filler: a land plot used to populate the landscape, may occur indefinitely

        If the list of desired features is empty, or when no more desired features can fit,
        filler land plot will be used to populate the landscape.

        The actual map area is (1, 1) to (xSize, ySize) inclusive, i.e. 1-indexed.
        This is because the perimeter of the landscape is generated artificially for the ease of road placements.

        The road filling algorithm may double the xSize and ySize of the generated landscape.
        Therefore, the generated map size will most likely NOT be the same as the initially defined map size.

        More speficially:
        - the final xSize ranges from xSize to xSize*2-1 inclusive
        - the final ySize ranges from ySize to ySize*2-1 inclusive

        NOTE: If filler isn't 1 by 1, the generated landscape may not be connected.
        """

        # Reset component references
        self.reset_landscape()

        # Generate feature land plots from desired features, then sort by decreasing area (place larger features first)
        featuresToAdd = Landscape.generate_features(desiredFeatures=desiredFeatures)
        featuresToAdd.sort(key=lambda feature: feature.area(), reverse=True)

        # Skip impossible placements quickly via temporary optimisation hashmap
        isUnplaceable: dict[tuple[int, int] : bool] = defaultdict(lambda: False)

        # Fit feature land plots into the landscape
        for feature in featuresToAdd:
            if isUnplaceable[(feature.xSize, feature.ySize)]:
                continue
            if feature.area() > self.availableArea:
                isUnplaceable[(feature.xSize, feature.ySize)] = True
                continue
            valid_coordinates = self.get_valid_placements(feature)
            if not valid_coordinates:
                isUnplaceable[(feature.xSize, feature.ySize)] = True
                continue
            self.place_feature(
                feature, *valid_coordinates[randint(0, len(valid_coordinates) - 1)]
            )

        # Generate filler land plots, then sort by decreasing area (place larger features first)
        fillerFeatures = Landscape.generate_features(
            desiredFeatures=[(filler, self.availableArea)]
        )
        fillerFeatures.sort(key=lambda feature: feature.area(), reverse=True)

        # Fill in remaining area with filler, remaining area may not be completely filled if filler isn't 1 by 1
        for feature in fillerFeatures:
            if isUnplaceable[(feature.xSize, feature.ySize)]:
                continue
            valid_coordinates = self.get_valid_placements(feature)
            if not valid_coordinates:
                isUnplaceable[(feature.xSize, feature.ySize)] = True
                continue
            self.place_feature(
                feature, *valid_coordinates[randint(0, len(valid_coordinates) - 1)]
            )

        self.generate_landscape_matrix()

        # Create traffic lights for every intersection with three or more roads
        for intersection in self.intersections.values():
            intersection.create_traffic_light()

        # Assign road IDs
        index = 0
        for road in self.roads:
            road.roadID = index
            self.lookupRoad[index] = road
            index += 1

    def generate_landscape_matrix(self) -> None:
        """
        Generates and updates self.landscapeMatrix from self.gridMatrix.
        Landscape matrix is an unpacked version of grid matrix with road cells.
        """

        # Buffer arrays for road expansions
        xBuffer = [False for i in range(self.xSize)]
        yBuffer = [False for i in range(self.ySize)]

        # Counters for getting the new X and Y coordinates after road insertions
        xExtend = 0
        yExtend = 0

        self.landscapeMatrix = deepcopy(self.gridMatrix)

        # Extend width of landscape for new roads
        for prevX in range(self.xSize):
            for prevY in range(self.ySize):
                # Skip unassigned area
                if self.gridMatrix[prevY][prevX] == None:
                    continue

                # Calculate corresponding coordiantes in landscapeMatrix
                newX, newY = prevX + xExtend, prevY + yExtend

                # Check neighbour cell to the right, if it belongs to a different plot of land then add road
                if (
                    prevX < self.xSize - 1
                    and self.gridMatrix[prevY][prevX]
                    != self.gridMatrix[prevY][prevX + 1]
                ):
                    # Extend width of landscape matrix if not already extended
                    if not xBuffer[prevX]:
                        for y in range(self.ySize + yExtend):
                            self.landscapeMatrix[y].insert(
                                newX + 1,
                                self.landscapeMatrix[y][newX]
                                if self.landscapeMatrix[y][newX]
                                == self.landscapeMatrix[y][newX + 1]
                                else None,
                            )
                        xBuffer[prevX] = True
                        xExtend += 1

        # # Uncomment to see the results of horizontal extension
        # for i in range(len(self.landscapeMatrix)):
        #     print(self.landscapeMatrix[i])
        # print()

        # Extend length of landscape for new roads
        for prevY in range(self.ySize):
            for prevX in range(self.xSize):
                # Skip unassigned area
                if self.gridMatrix[prevY][prevX] == None:
                    continue

                # Calculate corresponding coordiantes in landscapeMatrix
                newX, newY = prevX + xExtend, prevY + yExtend

                # Check neighbour cell below, if it belongs to a different plot of land then add road
                if (
                    prevY < self.ySize - 1
                    and self.gridMatrix[prevY][prevX]
                    != self.gridMatrix[prevY + 1][prevX]
                ):
                    # Extend length of landscape matrix if not already extended
                    if not yBuffer[prevY]:
                        self.landscapeMatrix.insert(
                            newY + 1,
                            [
                                self.landscapeMatrix[newY][x]
                                if self.landscapeMatrix[newY][x]
                                == self.landscapeMatrix[newY + 1][x]
                                else None
                                for x in range(self.xSize + xExtend)
                            ],
                        )
                        yBuffer[prevY] = True
                        yExtend += 1

        # # Uncomment to see the results of vertical extension
        # for i in range(len(self.landscapeMatrix)):
        #     print(self.landscapeMatrix[i])
        # print()

        # Update actual map size
        self.xSize += xExtend
        self.ySize += yExtend

        # Pad landscape for land plot reconstruction and road fitting
        self.landscapeMatrix = (
            [[None for i in range(self.xSize + 2)]]
            + [[None] + self.landscapeMatrix[i] + [None] for i in range(self.ySize)]
            + [[None for i in range(self.xSize + 2)]]
        )

        # Reconstruct all land plots
        for yPos in range(1, self.ySize + 1):
            for xPos in range(1, self.xSize + 1):
                # Skip unassigned area
                if self.landscapeMatrix[yPos][xPos] == None:
                    continue

                landPlotID = self.landscapeMatrix[yPos][xPos]

                # If the current cell is the bottom left corner, create new land plot
                if (
                    landPlotID != self.landscapeMatrix[yPos][xPos - 1]
                    and landPlotID != self.landscapeMatrix[yPos - 1][xPos]
                ):
                    # Initiate size variables of the land plot
                    xSize, ySize = 0, 0

                    for xCoord in range(xPos, self.xSize + 1):
                        if self.landscapeMatrix[yPos][xCoord] == landPlotID:
                            xSize += 1

                    for yCoord in range(yPos, self.ySize + 1):
                        if self.landscapeMatrix[yCoord][xPos] == landPlotID:
                            ySize += 1

                    # Create land plot
                    landPlot = LandPlot(xSize, ySize)
                    landPlot.set_coordinate(xPos, yPos)
                    self.landPlots.append(landPlot)

                    # Add land plot to component references
                    for yCoord in range(yPos, yPos + ySize):
                        for xCoord in range(xPos, xPos + xSize):
                            self.landMap[(yCoord, xCoord)] = landPlot

        # Create roads and intersections
        for landPlot in self.landPlots:
            # Spawn intersections at all four corners of every land plot
            corners = [
                (landPlot.xPos - 1, landPlot.yPos - 1),
                (landPlot.xPos + landPlot.xSize, landPlot.yPos - 1),
                (landPlot.xPos - 1, landPlot.yPos + landPlot.ySize),
                (landPlot.xPos + landPlot.xSize, landPlot.yPos + landPlot.ySize),
            ]
            bottomLeft, bottomRight, topLeft, topRight = [
                Intersection(*corner) for corner in corners
            ]

            # Register intersections
            for possible_intersection in [bottomLeft, bottomRight, topLeft, topRight]:
                self.intersections[
                    possible_intersection.coordinates()
                ] = possible_intersection

            # Label intersections and roads
            self.label_intersections(
                bottomLeft.coordinates(), bottomRight.coordinates()
            )
            self.label_intersections(bottomLeft.coordinates(), topLeft.coordinates())
            self.label_intersections(topRight.coordinates(), bottomRight.coordinates())
            self.label_intersections(topRight.coordinates(), topLeft.coordinates())

            # Update land plot labels
            for yCoord in range(landPlot.yPos, landPlot.yPos + landPlot.ySize):
                for xCoord in range(landPlot.xPos, landPlot.xPos + landPlot.xSize):
                    self.landscapeMatrix[yCoord][xCoord] = "LP"

        # # Unpad landscape to remove the artificialroad perimeter
        # self.landscapeMatrix = [self.landscapeMatrix[i][1:self.xSize+1] for i in range(1, self.ySize+1)]

        # Fill unassigned area with land plots
        for yCoord in range(self.ySize + 2):
            for xCoord in range(self.xSize + 2):
                if self.landscapeMatrix[yCoord][xCoord] == None:
                    self.landscapeMatrix[yCoord][xCoord] = "LP"
                    # NOTE: land plots added here are NOT registered within self.landPlots or self.landMap

        # Connect intersections via DFS and create road objects for graph
        stack = []
        visited = set()

        for (
            intersection_pos
        ) in self.intersections.keys():  # start with a random intersection
            stack.append(intersection_pos)
            break

        while stack:
            xPos, yPos = stack.pop()
            if (xPos, yPos) in visited:
                continue
            visited.add((xPos, yPos))

            for xCoord in range(xPos + 1, self.xSize + 2):  # search for east neighbour
                if self.landscapeMatrix[yPos][xCoord] == "IS":
                    if (xCoord, yPos) not in visited:
                        self.connect_intersections(
                            self.intersections[(xPos, yPos)],
                            self.intersections[(xCoord, yPos)],
                        )
                        stack.append((xCoord, yPos))
                    break
                elif self.landscapeMatrix[yPos][xCoord] == "LP":
                    break

            for xCoord in range(xPos - 1, -1, -1):  # search for west neighbour
                if self.landscapeMatrix[yPos][xCoord] == "IS":
                    if (xCoord, yPos) not in visited:
                        self.connect_intersections(
                            self.intersections[(xPos, yPos)],
                            self.intersections[(xCoord, yPos)],
                        )
                        stack.append((xCoord, yPos))
                    break
                elif self.landscapeMatrix[yPos][xCoord] == "LP":
                    break

            for yCoord in range(yPos + 1, self.ySize + 2):  # search for north neighbour
                if self.landscapeMatrix[yCoord][xPos] == "IS":
                    if (xPos, yCoord) not in visited:
                        self.connect_intersections(
                            self.intersections[(xPos, yPos)],
                            self.intersections[(xPos, yCoord)],
                        )
                        stack.append((xPos, yCoord))
                    break
                elif self.landscapeMatrix[yCoord][xPos] == "LP":
                    break

            for yCoord in range(yPos - 1, -1, -1):  # search for south neighbour
                if self.landscapeMatrix[yCoord][xPos] == "IS" and (xPos, yCoord):
                    if (xPos, yCoord) not in visited:
                        self.connect_intersections(
                            self.intersections[(xPos, yPos)],
                            self.intersections[(xPos, yCoord)],
                        )
                        stack.append((xPos, yCoord))
                    break
                elif self.landscapeMatrix[yCoord][xPos] == "LP":
                    break

    def calculateRoadData(self):
        """
        Calculates all the data regarding the roads that are sent to Unity.
        """

        # Get a list of max vehicle counts for each road
        maxVehicleCounts: list[int] = [
            self.roads[i].maxVehicleCount for i in range(len(self.roads))
        ]

        return maxVehicleCounts

    def calculateIntersectionData(self):
        """
        Calculates all the data regarding the intersections that are sent to Unity.
        """

        # Map every road ID to the neighbours list and traffic light pattern of the matching road's road-end intersection
        roadID_to_intersection_info: dict[
            int, list[list[tuple[int, int]], list[int], int]
        ] = {}
        for intersection in self.intersections.values():
            neighbour_road_start_pos_list: list[tuple[int, int]] = [
                self.roadmap[neighbour_intersection.coordinates()][
                    intersection.coordinates()
                ].startPosReal
                for neighbour_intersection in intersection.neighbours
            ]
            for neighbour_intersection in intersection.neighbours:
                road = self.roadmap[neighbour_intersection.coordinates()][
                    intersection.coordinates()
                ]
                roadID_to_intersection_info[road.roadID] = [
                    neighbour_road_start_pos_list,
                    intersection.trafficLightPattern,  # trafficLightPattern could be None
                    intersection.trafficLightDuration,  # -1 if None
                ]

        return roadID_to_intersection_info

    def precomputeUnityCache(self):
        self.unityCache: list[
            tuple[tuple[float, float], list[int], list[int], list[int], float]
        ] = []
        for inter in self.intersections.values():
            id = inter.coordinates()

            if self.landscapeMatrix[id[1]][id[0]] != "IS":
                raise ValueError("Found invalid intersection")

            enterRoadIDs: list[int] = []
            exitRoadIDs: list[int] = []
            for neighbor in inter.neighbours:
                enterRoadIDs.append(
                    self.roadmap[neighbor.coordinates()][inter.coordinates()].roadID
                )
                exitRoadIDs.append(
                    self.roadmap[inter.coordinates()][neighbor.coordinates()].roadID
                )
            self.unityCache.append(
                (
                    id,
                    enterRoadIDs,
                    exitRoadIDs,
                    inter.trafficLightPattern,
                    inter.trafficLightDuration,
                )
            )
