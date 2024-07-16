from collections import defaultdict
from random import randint, shuffle, uniform, choice
from copy import deepcopy
import math
from scipy.spatial import KDTree, Delaunay
import matplotlib.pyplot as plt
import shapely
import numpy as np
import sys
from DataStructures import *

sys.setrecursionlimit(10**6)

PI = 3.1415926535
allRoads = 0
allIntersections = 0
ACTIVE_LANDSCAPE = None
VEHICLE_LENGTH_METRES = 5

# ========================================= UTILITIES =========================================
def toTuple(c):
    return (c.real, c.imag)

def cis(x):
    return complex(math.cos(x), math.sin(x))

def arg(comp):
    return math.atan2(comp.imag, comp.real)


class Intersection:

    """
    An intersection is a point where two or more roads cross.
    If three or more roads cross, a traffic light is spawned with a random signal pattern.

    The coordinate (x, y) must be unique for each intersection.

    Each intersection also contains a connectingInts list that points to all its adjacent neighbours (up to four).
    """

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

        # Setting the ID of the intersection
        global allIntersections
        self.id = allIntersections
        allIntersections += 1

        # Initialising adjacency lists
        self.connectingInts = []
        self.connectingRoads = []
        self.connectingVirtualInts = []

        # Traffic light information
        self.trafficLightPattern: list[int] = None
        self.trafficLightDuration: int = -1  # how long each phase lasts, in seconds

        # Lookup table for quickly accessing the when a particular road gets the green light
        self.trafficLightLookup: dict[Intersection, int] = {}

        # How many vehicles can pass through the intersection in one phase
        self.trafficPassthroughRate = None

    def coordinates(self):
        return (self.x, self.y)

    def connect(self, secondInt, landscape):

        '''
        Connects the intersection to another intersection or virtual intersection with an automatically generated road.
        '''
        
        if type(secondInt) == VirtualIntersection:
            
            if secondInt in self.connectingVirtualInts:
                return
            
            self.connectingVirtualInts.append(secondInt)
            secondInt.connectingInts.append(self)
        
        elif type(secondInt) == Intersection:
            if secondInt in self.connectingInts:
                return
            
            road = Road(self, secondInt)
            landscape.roads.append(road)
            
            self.connectingRoads.append(road)
            secondInt.connectingRoads.append(road)
            
            self.connectingInts.append(secondInt)
            secondInt.connectingInts.append(self)
            
            landscape.GRAPH[self][secondInt] = road
            landscape.GRAPH[secondInt][self] = road
    
    def disconnect(self, secondInt, landscape):

        '''
        Disconnects the intersection from another intersection or virtual intersection.
        '''
        
        if type(secondInt) == VirtualIntersection:
            
            if secondInt not in self.connectingVirtualInts:
                return
            
            self.connectingVirtualInts.remove(secondInt)
            secondInt.connectingInts.remove(self)
        
        elif type(secondInt) == Intersection:
            
            if secondInt not in self.connectingInts:
                return
            
            # deleting the road that connects the two intersections
            road = landscape.GRAPH[self][secondInt]
            landscape.roads.remove(road)
            
            self.connectingRoads.remove(road)
            secondInt.connectingRoads.remove(road)
            
            self.connectingInts.remove(secondInt)
            secondInt.connectingInts.remove(self)
            
            
            landscape.GRAPH[self][secondInt] = None
            landscape.GRAPH[secondInt][self] = None
    
    def __lt__(self, other):
        return self.id < other.id
        

    def create_traffic_light(self, landscape) -> None:
        """
        Creates a random traffic signal pattern based on self.connectingInts.

        The pattern is represented by a list where:
        - the index of the list is the modulus time i.e. current time % number of roads at this intersection
        - the value at that index is the index of the road which starts at self.connectingInts[value]

        For example, self.connectingInts[self.trafficLightPattern[1]] yields the intersection from where
        the road that gets the green light at every 2nd phase of the traffic light pattern starts.
        """

        self.roadCount = len(self.connectingInts)

        if (self.roadCount >= 3): 
            # Create random green light order
            self.trafficLightPattern = [i for i in range(self.roadCount)]
            shuffle(self.trafficLightPattern)  # randomise green light order

            # Create lookup table based on green light order
            for i in range(self.roadCount):
                self.trafficLightLookup[self.connectingInts[self.trafficLightPattern[i]]] = i

            # Randomise phase duration between 3-8 seconds
            self.trafficLightDuration = randint(3, 8)

            # Calculate traffic passthrough rate for every road
            for intersection in self.connectingInts:
                limit = landscape.GRAPH[self][intersection].speedLimit
                lanes = landscape.GRAPH[self][intersection].laneCount
                self.trafficPassthroughRate = limit * lanes * self.trafficLightDuration // VEHICLE_LENGTH_METRES

    def __hash__(self) -> int:
        return hash((self.x, self.y))
    
# every road is bidirectional, and its lane count depends on its length
class Road:

    '''
    A road is a bidirectional connection between two intersections, and all its parameters are automatically generated.

    Lanes are numbered like so: 012210
    '''

    def __init__(self, int1 : Intersection, int2 : Intersection):
        self.int1 = int1
        self.int2 = int2
        
        # id
        global allRoads
        self.id = allRoads
        allRoads += 1
        
        # length 
        self.length = ((self.int1.x - self.int2.x)**2 + (self.int1.y - self.int2.y)**2)**0.5
        
        # just like in real life, the lane count and speed limit of the road generally depend on its length
        if self.length <= 100:
            self.laneCount = 1
            self.speedLimit = 17
        
        elif self.length <= 200:
            self.laneCount = 2
            self.speedLimit = 22
        
        else:
            self.laneCount = 3
            self.speedLimit = 28
        
        # angle
        self.angle = math.atan2(self.int2.y - self.int1.y, self.int2.x - self.int1.x)

        # constants
        self.traversalTime = self.length / self.speedLimit
        self.capacity = self.laneCount * self.length // VEHICLE_LENGTH_METRES
        
        # virtual intersections on the road
        self.associatedVirtualIntersections = []
        self.virtualInts = []
        
    
    def populatePositions(self, landscape):

        '''
        Populates the road with virtual intersections at regular intervals, and connects them to the real intersections
        at the start and end of the road.

        Virtual intersections are points on the road where lane changes and vehicle spawns can occur.

        Associated virtual intersections lie on a real intersection, and are what enable only certain lanes to be capable of turning onto other roads.
        '''
        
        for direction in [1, -1]:
            lanes = []
            
            for i in range(self.laneCount):
                
                # setting the starting and ending intersections depending on the direction
                if direction == 1:
                    startingInt = self.int1
                    endingInt = self.int2
                else:
                    startingInt = self.int2
                    endingInt = self.int1
                
                
                # lane is a 2D list of virtual intersections, with a each row being a lane
                lane = [VirtualIntersection(self, 0, direction, i, startingInt)]
                
                # creating the first virtual intersection and associating it with the starting intersection
                self.virtualInts.append(lane[0])
                landscape.virtualIntersections.append(lane[0])
                self.associatedVirtualIntersections.append(lane[0])
                lane[0].connect(startingInt)
                
                # creating the rest of the virtual intersections at regular intervals
                # since lane changes/spawns shouldn't occur too close to the endpoints of the road,
                # the interval is at least 20m, or 10% of the road length, whichever is greater
                # - also avoids having too many virtual intersections bloating the landscape
                j = 0.1
                while j < 0.9:

                    # creating the virtual intersection and associating it with the road
                    virtInt = VirtualIntersection(self, j, direction, i)
                    self.virtualInts.append(virtInt)
                    landscape.virtualIntersections.append(virtInt)
                    virtInt.connect(lane[-1])
                    
                    # setting backward and forward connections
                    virtInt.backward = lane[-1]
                    lane[-1].forward = virtInt
                    
                    lane.append(virtInt)
                    
                    j += max(20/self.length, 0.1)
                
                # creating the last virtual intersection and associating it with the ending intersection
                lane.append(VirtualIntersection(self, 1, direction, i, endingInt))
                lane[-1].connect(lane[-2])
                lane[-2].forward = lane[-1]
                lane[-1].backward = lane[-2]
                self.virtualInts.append(lane[-1])
                landscape.virtualIntersections.append(lane[-1])
                self.associatedVirtualIntersections.append(lane[-1])
                lane[-1].connect(endingInt)

                lanes.append(lane)
            
            # connecting the virtual intersections sideways - enabling lane changes
            for i in range(1, len(lanes)):
                for j in range(1, len(lanes[0])-1):
                    lanes[i][j].connect(lanes[i-1][j])
                    lanes[i-1][j].right = lanes[i][j]
                    lanes[i][j].left = lanes[i-1][j]
    
    def findAssociatedVirtualIntersection(self, intersection, lane, dir):
        # helper method to find an associated virtual intersection given a real intersection, lane and direction
        for virtInt in self.associatedVirtualIntersections:
            if virtInt.correspondingRealIntersection == intersection and virtInt.lane == lane and virtInt.direction == dir:
                return virtInt
        return None
    
    def availablePositions(self):
        # returns all the virtual intersections on the road that don't have a corresponding real intersection
        # these are suitable for starting and ending points for vehicles
        pos = []
        for virt in self.virtualInts:
            if virt.correspondingRealIntersection == None:
                pos.append(virt)
        return pos
    
    def __lt__(self, other):
        return self.id < other.id

    def __hash__(self) -> int:
        return hash((self.int1, self.int2))


def getRealPositionOnRoad(road: Road, position: float, direction : int) -> tuple[float, float]:
    """
    Calculates a real 2D position given road, position and direction.

    Parameters:
    - road (Road): The road on which the position is to be calculated.
    - position (float): The position on the road where the real position is to be calculated.
    - direction (int): The direction in which the position is to be calculated. 1 for int1 to int2, -1 for int2 to int1.
    """
    int1 = road.int1.coordinates()
    int2 = road.int2.coordinates()
    if direction == -1:
        int1, int2 = int2, int1
    x = int1[0] + (int2[0] - int1[0]) * position
    y = int1[1] + (int2[1] - int1[1]) * position
    return (x, y)


class VirtualIntersection(Intersection):
    '''
    A virtual intersection (VI) is a point on a road where lane changes and vehicle spawns can occur.
    It inherits from the Intersection class.
    '''
    def __init__(self, road : Road, position : float, direction : int, lane : int, correspondingRealIntersection = None) -> None:
        x, y = getRealPositionOnRoad(road, position, direction)
        super().__init__(x, y)
        
        # it has several unique attributes that describe its position on the road
        # this is all to avoid being reliant on its real position, which could change on the frontend
        self.road = road
        self.lane = lane
        self.direction = direction
        self.position = position
        self.correspondingRealIntersection = correspondingRealIntersection
        self.left = None
        self.right = None
        self.forward = None
        self.backward = None
    
    def connect(self, secondInt):

        '''
        Connects the virtual intersection to another virtual intersection or real intersection.
        '''
        
        if type(secondInt) == VirtualIntersection:
            if secondInt in self.connectingVirtualInts:
                return
            self.connectingVirtualInts.append(secondInt)
            secondInt.connectingVirtualInts.append(self)
            
            # if both virtual intersections have corresponding real intersections, connect the roads that both of them lie on
            if self.correspondingRealIntersection != None and secondInt.correspondingRealIntersection != None:
                self.connectingRoads.append(secondInt.road)
                secondInt.connectingRoads.append(self.road)
        
        elif type(secondInt) == Intersection:
            if secondInt in self.connectingInts:
                return
            self.connectingInts.append(secondInt)
            secondInt.connectingVirtualInts.append(self)
    
    def disconnect(self, secondInt):

        '''
        Disconnects the virtual intersection from another virtual intersection or real intersection.
        '''
        
        if type(secondInt) == VirtualIntersection:
            if secondInt not in self.connectingVirtualInts:
                return
            
            self.connectingVirtualInts.remove(secondInt)
            secondInt.connectingVirtualInts.remove(self)
            
            if self.correspondingRealIntersection != None and secondInt.correspondingRealIntersection != None:
                self.connectingRoads.remove(secondInt.road)
                secondInt.connectingRoads.remove(self.road)
        
        elif type(secondInt) == Intersection:
            if secondInt not in self.connectingInts:
                return
            self.connectingInts.remove(secondInt)
            secondInt.connectingVirtualInts.remove(self)
    
    def delete(self, landscape):
        '''
        Completely removes the virtual intersection from the landscape.

        A VI is deleted when it is no longer needed, meaning its lane can't be used to turn on to any roads.
        Deleting a VI means that cars must change lanes to non-deleted VIs to turn, and avoids the situation of
        cars being stuck in a lane that cannot turn.
        '''
        for intersection in self.connectingInts:
            self.disconnect(intersection)
        for intersection in self.connectingVirtualInts:
            self.disconnect(intersection)
        if self in self.road.associatedVirtualIntersections:
            self.road.associatedVirtualIntersections.remove(self)
        if self in self.road.virtualInts:
            self.road.virtualInts.remove(self)
        if self in landscape.virtualIntersections:
            landscape.virtualIntersections.remove(self)

class Landscape:
    '''
    The landscape is the entire 2D map, containing all intersections, roads, and virtual intersections.
    
    It is automatically populated through several methods:
    1. Grid Generation - randomly oriented grids of intersection are spawned to simulate suburban residences. However,
                         a consequence of the random orientation is that they may often go beyond the map boundaries and increase its size.
    2. Isolated Intersections - random intersections are spawned to simulate non-residential areas

    Then, isolated intersections are connected using Delaunay Triangulation, and hyperconnected intersections are pruned to simplify the graph.

    Finally, all components of the graph are connected to each other, using a DSU, to make every node traversable from every other node.
    '''
    def __init__(self, xSize: int, ySize: int, gridSparseness : float = 0.4, gridCoverage : float = 0.6) -> None:
        '''
        NOTE: Initialise the landscape with smaller dimensions than intended, as the grid generation may increase is size by up to 50%.
        '''

        # initialising dimensions
        self.xSize = xSize
        self.ySize = ySize
        self.area = xSize * ySize

        # gridSparseness (0-1) is a measure of how sparse each grid is - 0 means fully connected, 1 means no grids are spawned at all
        self.gridSparseness = gridSparseness

        # gridCoverage (0-1) is a measure of how much of the landscape is covered by grids - 0 means no grids, 1 means the entire landscape is covered just with grids
        self.gridCoverage = gridCoverage
        
        # all intersections are stored in a dictionary, with their IDs as keys
        self.intersections : dict[int, Intersection] = {}

        # a 'polygon' is a region of the map generated in a gridlike fashion 
        # - used to prevent isolated intersections or roads from overlapping the grids
        self.polygons = []

        # isolated intersections are spawned randomly and not part of any grid
        self.isolatedIntersections = []
        self.isolatedCoords = []

        self.roads = []
        self.virtualIntersections = []

        # lookup table for quickly accessing intersections based on their coordinates
        self.lookupTable : dict[tuple[float, float], Intersection] = {}
       
        # resetting global variables
        global allRoads
        global allIntersections
        global ACTIVE_LANDSCAPE
        ACTIVE_LANDSCAPE = self
        allRoads = 0
        allIntersections = 0
        
        # the graph is represented as a dictionary of dictionaries, where the keys are intersections
        self.GRAPH : dict[Intersection, dict[Intersection, Road]] = defaultdict(dict)

    def reset(self):
        '''
        Resets the landscape to its initial state.
        '''
        self.intersections = {}
        self.roads = []
        self.virtualIntersections = []
        self.lookupTable = {}
        self.GRAPH = defaultdict(dict)
        global allRoads
        global allIntersections
        allRoads = 0
        allIntersections = 0
    
    def resize(self):
        '''
        Resizes the landscape to fit all generated elements, and returns the corners of the landscape.
        '''
        rightmost = max(self.intersections.values(), key = lambda x: x.x)
        leftmost = min(self.intersections.values(), key = lambda x: x.x)
        bottom = min(self.intersections.values(), key = lambda x: x.y)
        top = max(self.intersections.values(), key = lambda x: x.y)
        
        self.xSize = int(rightmost.x - leftmost.x)
        self.ySize = int(top.y - bottom.y)
        self.area = self.xSize * self.ySize
        
        return bottom, leftmost, rightmost, top

    def spawnGridBlock(self, bottomLeft: complex, topRight: complex, rotation: float):
        '''
        Spawns an individual grid block of intersections, with the bottom left and top right corners of the block, and the rotation of the block.
        They are arranged in a grid pattern, and rotated 'rotation' radians anticlockwise relative to the bottom-left corner.

        Parameters:
        - bottomLeft (complex): The bottom left corner of the grid block.
        - topRight (complex): The top right corner of the grid block.
        - rotation (float): The rotation of the grid block in radians.

        '''
        
        horLength = topRight.real - bottomLeft.real
        maxHors = horLength // 20 # maximum number of intersections that can fit in the block
        hors = randint(2, maxHors+1) # number of intersections in the block
        horDist = horLength / (hors-1) # distance between each intersection
        
        vertLength = topRight.imag - bottomLeft.imag
        maxVerts = vertLength // 20 # maximum number of intersections that can fit in the block
        verts = randint(2, maxVerts+1) # number of intersections in the block
        vertDist = vertLength / (verts-1) # distance between each intersection

        
        # creating the grid as a 2D list where each element represents an intersection
        grid = []
        for i in range(verts):
            temp = []
            for j in range(hors):
                temp.append(None)
            grid.append(deepcopy(temp))
            

        for i in range(verts):
            for j in range(hors):
                # position of the intersection
                position = bottomLeft + complex(j * horDist, i * vertDist) * cis(rotation)
                
                # depending on the gridSparseness, the intersection may not be spawned at all - this enhances realism as road networks are rarely perfect grids
                chance = uniform(0, 1)
                if chance <= self.gridSparseness:
                    continue

                intersection = Intersection(position.real, position.imag)
                grid[i][j] = deepcopy(intersection)
                
                # connecting the intersection to its neighbours
                if i > 0 and grid[i-1][j] != None:
                  grid[i][j].connect(grid[i-1][j], self)
                if j > 0 and grid[i][j-1] != None:
                  grid[i][j].connect(grid[i][j-1], self)
                
                self.intersections[intersection.id] = grid[i][j]
                self.lookupTable[toTuple(position)] = grid[i][j]
        

    def pointObstructed(self, point: complex) -> bool:
        '''
        Helper function that checks if the point is 'obstructed' by any other isolated point or polygon.
        
        A point is 'obstructed' if: 
            - its distance from any point or polygon is <= x**0.3 * ln(x), where x is sqrt(area)
        
        This function was custom-made to ensure that road density remains realistic, especially when generating large landscapes.

        Parameters:
        - point (complex): The point to check for obstruction.

        Returns:
        - bool: True if the point is obstructed, False otherwise.
        
        '''
        x = self.area**0.5
        minDist = x**0.3 * math.log(x)
        point = shapely.geometry.Point(point.real, point.imag)
        
        # checking if it's obstructed by any other point
        for i in range(len(self.isolatedCoords)):
            otherPoint = shapely.geometry.Point(self.isolatedCoords[i])
            if point.distance(otherPoint) <= minDist:
                return True
        
        # checking if it's obstructed by any polygon (i.e. grid block)
        for i in range(len(self.polygons)):
            poly = shapely.geometry.Polygon(self.polygons[i])
            if point.distance(poly) <= minDist:
                return True
        
        return False
        
        
    # checks if the distance between any point in this region is <= 30m from any point in another region
    def regionObstructed(self, bottomLeft: complex, topRight : complex, rotation : float) -> bool:
        '''
        Helper function that checks if a polygon intersects another polygon in the landscape.
        Used to prevent grid blocks from overlapping.

        Parameters:
        - bottomLeft (complex): The bottom left corner of the polygon.
        - topRight (complex): The top right corner of the polygon.
        - rotation (float): The rotation of the polygon in radians  (anticlockwise).

        Returns:
        - bool: True if the region is obstructed, False otherwise.
        '''

        # initialising corners
        bl = bottomLeft
        br = bottomLeft + (topRight.real - bottomLeft.real)
        tr = topRight
        tl = bottomLeft + (topRight.imag - bottomLeft.imag) * 1j

        # rotating each corner around bottomLeft
        bl_rot = bl
        br_rot = (br - bottomLeft) * cis(rotation) + bottomLeft
        tr_rot = (tr - bottomLeft) * cis(rotation) + bottomLeft
        tl_rot = (tl - bottomLeft) * cis(rotation) + bottomLeft

        # creating the polygon
        thisPoly = shapely.geometry.Polygon([toTuple(bl_rot), toTuple(br_rot), toTuple(tr_rot), toTuple(tl_rot)])

        # checking if it intersects any other polygon
        for i in range(len(self.polygons)):
            poly = self.polygons[i]
            otherPoly = shapely.geometry.Polygon(poly)
            if thisPoly.intersects(otherPoly):
                return True
        
        return False
            
        
    def spawnGridBlocks(self):
        '''
        Spawns grid blocks of intersections in the landscape, with random orientations and sizes.
        It keeps spawning them until their combined area is more than the landscape's gridCoverage (proportion of area covered by grids).
        '''
        
        consumedArea = 0

        # making sure that actual coverage is not much more than the desired coverage
        while consumedArea < self.area * (self.gridCoverage - 0.05):

            # the upper bound on area is successively constrained, so block sizes keep getting smaller - further enhances realism
            blockArea = randint(900, min(int(self.gridCoverage * self.area - consumedArea), 1000000))
            
            # making sure the block isn't too long nor thin, and it doesn't exceed the landscape boundaries
            horLength = randint(int(blockArea**0.5), min(min(blockArea//20, self.xSize), 1000))
            vertLength = min(blockArea//horLength, self.ySize)
            
            # recalculating the area of the block in case any constraints were violated
            blockArea = horLength * vertLength
            
            # randomly spawning the block
            bottomLeft = complex(randint(0, self.xSize - horLength), randint(0, self.ySize - vertLength))
            topRight = bottomLeft + complex(horLength, vertLength)
            rotation = round(uniform(-PI,PI), 2)
            
            # making sure the block isn't obstructed by another previously spawned block
            while self.regionObstructed(bottomLeft, topRight, rotation):
                bottomLeft = complex(randint(horLength, self.xSize), randint(vertLength, self.ySize))
                topRight = bottomLeft + complex(horLength, vertLength)
                rotation = round(uniform(-PI,PI), 2)
            
            self.spawnGridBlock(bottomLeft, topRight, rotation)
            bl = bottomLeft
            br = bottomLeft + (topRight.real - bottomLeft.real)
            tr = topRight
            tl = bottomLeft + (topRight.imag - bottomLeft.imag) * 1j

            # rotating each corner around bottomLeft
            bl_rot = bl
            br_rot = (br - bottomLeft) * cis(rotation) + bottomLeft
            tr_rot = (tr - bottomLeft) * cis(rotation) + bottomLeft
            tl_rot = (tl - bottomLeft) * cis(rotation) + bottomLeft

            # adding the polygon to the list of polygons
            self.polygons.append([toTuple(bl_rot), toTuple(br_rot), toTuple(tr_rot), toTuple(tl_rot)])
            
            consumedArea += blockArea
        
    def spawnIsolatedIntersections(self):
        '''
        Spawns isolated intersections in the landscape until maximum density is reached.
        '''

        bottom, leftmost, rightmost, top = self.resize()
       
        while True:
            # creating a random point
            point = complex(randint(int(leftmost.x), int(rightmost.x)), randint(int(bottom.y), int(top.y)))
            
            # if the point is obstructed, try again until 100 failed attempts have occurred - this indicates the landscape is very dense already
            failed = 0
            done = False
            while self.pointObstructed(point):
                point = complex(randint(int(leftmost.x), int(rightmost.x)), randint(int(bottom.y), int(top.y)))
                failed += 1
                if failed >= 100:
                    done = True
                    break
            if done:
                break
            
            # creating the intersection
            intersection = Intersection(point.real, point.imag)
            self.intersections[intersection.id] = intersection
            self.isolatedIntersections.append(intersection)
            self.isolatedCoords.append(toTuple(point))
            self.lookupTable[toTuple(point)] = intersection
        
    def connectIsolatedInts(self):
        '''
        Connecting isolated intersections to each other using Delaunay Triangulation.
        
        Delaunay triangulation partitions a set of points in a plane into non-overlapping triangles such 
        that no point is inside the circumcircle of any triangle. It maximizes the minimum angle of the triangles, 
        avoiding excessively thin triangles that are unrealistic for real-life roads.

        Within scipy's implementation (the one used here), it is implemented in O(nlogn) time for maximal efficiency.
        '''
        
        # creating a list of points
        points = np.array([(a.x, a.y) for a in self.isolatedIntersections])
        delaunay = Delaunay(points)
        
        # checking and plotting Delaunay triangles
        for simplex in delaunay.simplices:
            triangle = [points[simplex[0]], points[simplex[1]], points[simplex[2]], points[simplex[0]]]
            line1 = shapely.geometry.LineString([points[simplex[0]], points[simplex[1]]])
            line2 = shapely.geometry.LineString([points[simplex[1]], points[simplex[2]]])
            line3 = shapely.geometry.LineString([points[simplex[2]], points[simplex[0]]])
            
            # using the minDist function again, we make sure that it's not too close to any existing grid block
            intersects = False
            for poly in self.polygons:
                polygon = shapely.geometry.Polygon(poly)
                x = self.area**0.5
                minDist = x**0.3 * math.log(x)
                if line1.distance(polygon) <= minDist or line2.distance(polygon) <= minDist or line3.distance(polygon) <= minDist:
                    intersects = True
                    break
            
            if intersects:
                continue
            
            i1 = self.lookupTable[(triangle[0][0], triangle[0][1])]
            i2 = self.lookupTable[(triangle[1][0], triangle[1][1])]
            i3 = self.lookupTable[(triangle[2][0], triangle[2][1])]
            
            # making sure that each road is >= 50m long and no intersection has more than 4 connections
            if line1.length >= 50 and len(i1.connectingInts) < 4 and len(i2.connectingInts) < 4:
                i1.connect(i2, self)
            if line2.length >= 50 and len(i2.connectingInts) < 4 and len(i3.connectingInts) < 4:
                i2.connect(i3, self)
            if line3.length >= 50 and len(i3.connectingInts) < 4 and len(i1.connectingInts) < 4:
                i3.connect(i1, self)
    

    def pruneEdges(self):
        '''
        Prunes hyperconnected isolated intersections to simplify the graph.

        Reduces the number of multi-lane roads to avoid physically overlapping roads.

        Deletes one road from a pair, if the angle between them is too little
        '''
        for intersection in self.isolatedIntersections:
            edges = len(intersection.connectingRoads)
            
            # chooses a random number of edges to keep - most likely outcome is 3
            toRemove = edges - choice([2, 3, 3, 4])
            
            if toRemove <= 0:
                continue
            
            # removing the edges
            toDelete = []
            shuffle(intersection.connectingInts)
            
            for i in range(toRemove):
                toDelete.append(intersection.connectingInts[i].id)
            
            for i in range(toRemove):
                intersection.disconnect(self.intersections[toDelete[i]], self)

            two_lane, three_lane = [], []
            for road in intersection.connectingRoads:
                if road.laneCount == 2:
                    two_lane.append(road)
                else:
                    three_lane.append(road)
            
            for i in range(len(two_lane)-1):
                two_lane[i].laneCount = 1
            
            for i in range(len(three_lane)-1):
                three_lane[i].laneCount = 1

            multi_lane = two_lane + three_lane
            for r1 in range(len(multi_lane)):
                for r2 in range(r1+1, len(multi_lane)):
                    if abs(multi_lane[r1].angle - multi_lane[r2].angle) < 0.1:
                        multi_lane[r1].laneCount = 1
                        multi_lane[r2].laneCount = 1
                        break
    
    def makeConnected(self):
        '''
        Connects all components of the graph to each other, ensuring completeness and that all nodes are reachable.
        '''

        # creating a dsu to track component connectivity
        dsu = DSU(len(self.intersections))
        
        # mapping intersection IDs to indices and vice versa for ease of use
        idToInd = {}
        indToId = {}
        for i, intersection in enumerate(self.intersections.values()):
            idToInd[intersection.id] = i
            indToId[i] = intersection.id
        
        # creating a kdtree for O(logn) nearest neighbour queries. Building is O(nlogn).
        kdtree = KDTree([(intersection.x, intersection.y) for intersection in self.intersections.values()])
        
        # representing all roads as line segments, and also adding connected edges into the dsu
        lines = []
        intConnections = []
        for road in self.roads:
            lines.append(shapely.geometry.LineString([(road.int1.x, road.int1.y), (road.int2.x, road.int2.y)]))
            intConnections.append((road.int1.id, road.int2.id))
            dsu.union(idToInd[road.int1.id], idToInd[road.int2.id])
        
        # connecting points such that they don't intersect any other road
        for id, intersection in self.intersections.items():
            # querying the 10 closest points to the intersection, as some will be discarded
            closestpoints = kdtree.query((intersection.x, intersection.y), 10)
            
            for i in range(10):
                # if they're already connected, skip
                otherint = self.intersections[indToId[closestpoints[1][i]]]
                if dsu.connected(id, otherint.id):
                    continue

                # check line intersection with any existing road
                line = shapely.geometry.LineString([(intersection.x, intersection.y), (otherint.x, otherint.y)])
                intersects = False
                for j in range(len(lines)):
                    if id != intConnections[j][0] and id != intConnections[j][1] and otherint.id != intConnections[j][0] and otherint.id != intConnections[j][1]:
                        if line.intersects(lines[j]):
                            intersects = True
                            break
                
                # if it doesn't intersect any road, create it
                if not intersects and len(otherint.connectingInts) < 4 and len(intersection.connectingInts) < 4:
                    intersection.connect(otherint, self)
                    dsu.union(id, otherint.id)
                    lines.append(line)
                    intConnections.append((id, otherint.id))
                    break
        
        # theoretically the chance of successful connection is >99%
        if dsu.num_comps() == 1:
            print("Graph is connected!")

    def relativeRoadLookup(self, intersection):
        '''
        Helper function that returns a list of roads connected to an intersection, 
        sorted by lane count.

        Parameters:
        - intersection (Intersection): The intersection to find connected roads for.

        Returns:
        - list: A list of tuples, where each tuple contains the lane count, direction, road, and angle of the road relative to the intersection.
        '''
        roads = []
        for road in intersection.connectingRoads:
            start = complex(*intersection.coordinates())
            if road.int2 == intersection:
                end = complex(*road.int1.coordinates())
                argument = arg(end - start)
                roads.append((road.laneCount, 1, road, argument))
            else:
                end = complex(*road.int2.coordinates())
                argument = arg(end - start)
                roads.append((road.laneCount, -1, road, argument))
        
        roads.sort()
        return roads
    
    
    def connectMultiIntersection(self, intersection : Intersection):
        '''
        For each intersection, this function bridges each lane of each road to the corresponding lane of the other road.
        It also takes into account differing lane counts between the roads, and whether they lie to the left or right of each other:
        - If road2 lies to the left of road1, the leftmost lanes of road1 are connected to road2.
            - If road2 has less lanes than road1, then, the rightmost lanes of road1 are not connected to road2.

        Parameters:
        - intersection (Intersection): The intersection to connect.

        TODO: improve speed
        '''
        roadLookup = self.relativeRoadLookup(intersection)
        roadLookup.reverse()

        # take each road as the reference point, then accordingly create the lane connections for the other roads
        # we go in descending order of lanes, so the reference road will *always* have to discard some lanes

        for i in range(len(roadLookup)):

            l1 = roadLookup[i]
            road1 : Road = l1[2]

            # setting the directions of the left and right lanes, assuming left-hand driving
            if l1[1] == 1:
                leftDir = 1
                rightDir = -1
            else:
                leftDir = -1
                rightDir = 1

            for j in range(i+1, len(roadLookup)):

                l2 = roadLookup[j]
                road2 : Road = l2[2]

                # angle between the roads, the difference in lane count, and the number of lanes to keep
                angle = (math.degrees(l2[3] - l1[3]) + 360) % 360
                laneDifference = l1[0] - l2[0]
                lanesToKeep = l2[0]
                
                # just making sure the roads are in the correct order and some lanes are being kept
                assert l1[0] >= l2[0] and lanesToKeep > 0

                for k in range(lanesToKeep):
                    
                    # road2 lies to the left of road1
                    # i.e. we want to keep the leftmost lanes of road1
                    if angle > 180:
                        road1NodeL = road1.findAssociatedVirtualIntersection(intersection, k, leftDir)
                        road1NodeR = road1.findAssociatedVirtualIntersection(intersection, k+laneDifference, rightDir)
                    
                    # road2 lies to the right of road1
                    # i.e. we want to keep the rightmost lanes of road1
                    else:
                        road1NodeL = road1.findAssociatedVirtualIntersection(intersection, k+laneDifference, leftDir)
                        road1NodeR = road1.findAssociatedVirtualIntersection(intersection, k, rightDir)

                    road2NodeF = road2.findAssociatedVirtualIntersection(intersection, k, 1)
                    road2NodeB = road2.findAssociatedVirtualIntersection(intersection, k, -1)

                    # using a case study, we derive that these are the combinations of connections that need to be made
                    if l2[1] == 1:
                        road1NodeL.connect(road2NodeB)
                        road1NodeR.connect(road2NodeF)
                    else:
                        road1NodeL.connect(road2NodeF)
                        road1NodeR.connect(road2NodeB)

        # deleting virtual intersections that have no connections to any other road
        # this is needed for cars to know to switch lanes to turn
        for i in roadLookup:
            road : Road = i[2]
            for virtual in road.associatedVirtualIntersections:
                if virtual.correspondingRealIntersection != intersection:
                    continue
                toDelete = True
                for connection in virtual.connectingVirtualInts:
                    if connection.correspondingRealIntersection != None:
                        toDelete = False
                        break
                if toDelete:
                    virtual.delete(self)
    
    
    def gen(self):
        '''
        Generates the entire landscape.
        '''
       
        self.spawnGridBlocks()
        self.spawnIsolatedIntersections()
        self.connectIsolatedInts()
        self.pruneEdges()
        self.makeConnected()
        for intersection in self.intersections.values():
            assert len(intersection.connectingInts) <= 4
            intersection.create_traffic_light(self)
        for road in self.roads:
            road.populatePositions(self)
        for id, intersection in self.intersections.items():
            self.connectMultiIntersection(intersection)

    def generate(self):
        '''
        Keeps trying to generate until it's successful.

        This is in case the Delaunay triangulation fails to connect all isolated intersections (a rare occurrence).
        '''

        success = False
        while not success:
            try:
                self.gen()
                success = True
            except:
                print("Failed to generate, trying again...")
                self.reset()
                continue
        
    
    def show(self):
        '''
        Visualises the landscape using matplotlib.

        The commented-out section visualises the virtual intersections as well, which is optional.
        
        plt.show() must be separately called to display the plot.
        '''
        plt.scatter([i.x for i in self.intersections.values()], [i.y for i in self.intersections.values()], c='black', s = 4)
        #plt.scatter([i.x for i in self.virtualIntersections], [i.y for i in self.virtualIntersections], c = 'g', s = 3)
        for i, inter in self.intersections.items():
            for road in inter.connectingRoads:
                plt.plot([road.int1.x, road.int2.x], [road.int1.y, road.int2.y], 'black')
    
    def storeImage(self, path):
        '''
        Stores the visualisation of the landscape in an image file.

        Parameters:
        - path (str): The path to store the image in.
        '''
        self.show()
        plt.savefig(path)
        print(f"Image stored successfully in {path}.")

    def store(self, path):
        '''
        Stores the landscape in a file using a custom file format.

        Parameters:
        - path (str): The path to store the landscape in.
        '''
       
        with open(path, 'w') as f:
            f.write(f"{self.xSize} {self.ySize}\n")
            f.write(f"{len(self.intersections)}\n")
            for id, intersection in self.intersections.items():
                f.write(f"{id} {intersection.x} {intersection.y}\n")
            
            f.write(f"{len(self.roads)}\n")
            for road in self.roads:
                f.write(f"{road.int1.id} {road.int2.id}\n")
        
        print(f"Landscape stored successfully in {path}.")

    def load(self, path):
        '''
        Loads the landscape from a file using a custom file format.

        Parameters:
        - path (str): The path to load the landscape from.
        '''
        with open(path, 'r') as f:
            
            # reset all class variables
            self.reset()
            
            lines = f.readlines()
            
            self.xSize, self.ySize = map(int, lines[0].split())
            
            numIntersections = int(lines[1])
            for i in range(2, 2 + numIntersections):
                id, x, y = map(float, lines[i].split())
                self.intersections[id] = Intersection(x, y)
                self.lookupTable[(x, y)] = self.intersections[id]
            
            numRoads = int(lines[2 + numIntersections])
            for i in range(3 + numIntersections, 3 + numIntersections + numRoads):
                int1, int2 = map(int, lines[i].split())
                int1 = self.intersections[int1]
                int2 = self.intersections[int2]
                int1.connect(int2, self)
            
            for intersection in self.intersections.values():
                assert len(intersection.connectingInts) <= 4
                intersection.create_traffic_light(self)
            
            for road in self.roads:
                road.populatePositions(self)
            
            for id, intersection in self.intersections.items():
                self.connectMultiIntersection(intersection)

        print(f"Landscape loaded successfully from {path}.")
            

                
            
    
