import xml.etree.ElementTree as ET
import math

def parse_osm_roads(osm_file):
    tree = ET.parse(osm_file)
    root = tree.getroot()
    
    # create a dictionary to store node coordinates, and find all nodes
    node_coords = {}
    for node in root.findall('node'):
        node_id = node.get('id')
        lat = node.get('lat')
        lon = node.get('lon')
        node_coords[node_id] = (lat, lon)
    
    roads = []
    # valid highway types
    valid_highways = {
        'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link',
        'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified',
        'unclassified_link', 'residential', 'residential_link', 'service', 'service_link',
        'living_street', 'track', 'path', 'road'
    }
    
    # iterate through all ways (edges)
    for way in root.findall('way'):
        is_road = False
        road_info = {
            'id': way.get('id'),
            'nodes': [],
            'coordinates': [],
            'tags': {}
        }
        
        # iterate through all tags of the way
        for tag in way.findall('tag'):
            key = tag.get('k')
            value = tag.get('v')

            # check if the way is actually a road
            if key == 'highway' and value in valid_highways:
                is_road = True
                road_info['tags'][key] = value
            elif key in {'motor_vehicle', 'motorcar', 'access'}:
                road_info['tags'][key] = value
        
        if is_road:

            highway_type = road_info['tags'].get('highway')
            
            # filter out paths that are not for motor vehicles
            if highway_type == 'path' and road_info['tags'].get('motor_vehicle') != 'yes' and road_info['tags'].get('motorcar') != 'yes':
                continue
            
            # filter out roads with restricted access
            access = road_info['tags'].get('access')
            if access in {'private', 'delivery', 'emergency', 'forestry', 'no'}:
                continue
            for nd in way.findall('nd'):
                node_id = nd.get('ref')
                road_info['nodes'].append(node_id)
                if node_id in node_coords:
                    road_info['coordinates'].append(node_coords[node_id])
            
            roads.append(road_info)
    
    return roads

def haversine_distance(lat1, lon1, lat2, lon2):
    '''
    Calculates the great circle distance between two points
    on the earth (specified in decimal degrees)
    '''
    # Radius of the Earth in meters
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def convert_to_relative_scale(intersections):
    '''
    Converts the lat/lon coordinates of intersections to a relative scale
    with respect to the top-left intersection.
    '''
    if not intersections:
        return []

    # find the top-left intersection
    top_left = min(intersections, key=lambda x: (float(x[1][0]), float(x[1][1])))
    top_left_lat, top_left_lon = float(top_left[1][0]), float(top_left[1][1])
    
    # for each intersection, calculate the relative x and y coordinates
    relative_intersections = []
    for node_id, (lat, lon) in intersections:
        lat, lon = float(lat), float(lon)
        x = haversine_distance(top_left_lat, top_left_lon, top_left_lat, lon)
        y = haversine_distance(top_left_lat, top_left_lon, lat, top_left_lon)
        if lat < top_left_lat:
            y = -y
        if lon < top_left_lon:
            x = -x
        relative_intersections.append((node_id, (x, y)))
    
    return relative_intersections



for city in ['melbourne', 'manhattan', 'los_angeles', 'london', 'tokyo']:
    
    connections = []
    intersections = []
    taken_ints = set()
    
    # parse the OSM file and extract road information
    osm_file_path = 'MapParsing/maps/' + city
    roads = parse_osm_roads(osm_file_path)
    
    # extract intersection coordinates and road connections
    for road in roads:
        nodes = road['nodes']
        coords = road['coordinates']
        if nodes[0] != nodes[-1]:
            connections.append((nodes[0], nodes[-1]))
            if nodes[0] not in taken_ints:
                intersections.append((nodes[0], coords[0]))
                taken_ints.add(nodes[0])
            if nodes[-1] not in taken_ints:
                intersections.append((nodes[-1], coords[-1]))
                taken_ints.add(nodes[-1])\

    # convert the coordinates to a relative scale (coordinate compression)
    intersections = convert_to_relative_scale(intersections)
    maxX = max(intersections, key=lambda x: x[1][0])[1][0]
    minX = min(intersections, key=lambda x: x[1][0])[1][0]
    maxY = max(intersections, key=lambda x: x[1][1])[1][1]
    minY = min(intersections, key=lambda x: x[1][1])[1][1]
    xSize = maxX - minX
    ySize = maxY - minY


    # write the new map to a text file in our custom format
    with open(f'NewVersion/{city}.txt', 'w') as f:
        
        f.write(f"{xSize} {ySize}\n")
        f.write(f"{len(intersections)}\n")
        
        for intersection in intersections:
            f.write(f"{intersection[0]} {intersection[1][0]} {intersection[1][1]}\n")
        
        f.write(f"{len(connections)}\n")
        for road in connections:
            f.write(f"{road[0]} {road[1]}\n")





