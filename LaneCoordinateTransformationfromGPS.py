import xml.etree.ElementTree as ET
import math
from pyproj import CRS, Transformer





def gps_to_utm(lat, lon, azimuth):
    # Define CRS for WGS84 and UTM Zone 32N
    wgs84_crs = CRS.from_epsg(4326)  # WGS84
    utm_crs = CRS.from_epsg(32632)   # UTM Zone 32N

    # Initialize the transformer (transform from WGS84 to UTM)
    transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)

    # Convert to UTM coordinates
    x, y = transformer.transform(lon, lat)

    # Azimuth (heading) information
    att_azimuth_z = math.radians(azimuth)
    h = (att_azimuth_z - math.pi / 2) * -1

    return x, y, h


def parse_opendrive(file_path):
    """Parses the OpenDRIVE file and extracts road, geometry, and lane information."""
    tree = ET.parse(file_path)
    root = tree.getroot()
    roads = []

    # Iterate over each road element in the OpenDRIVE file
    for road in root.findall('road'):
        road_id = road.get('id')
        length = float(road.get('length'))
        geometries = []

        # Extract geometric segments from the <planView> section
        for geometry in road.find('planView').findall('geometry'):
            s = float(geometry.get('s'))
            x = float(geometry.get('x'))
            y = float(geometry.get('y'))
            hdg = float(geometry.get('hdg'))
            length = float(geometry.get('length'))

            geo_type = list(geometry)[0].tag  # line, arc, or spiral
            geo_data = geometry.find(geo_type).attrib if geo_type != 'line' else {}

            geometries.append({
                's': s,
                'x': x,
                'y': y,
                'hdg': hdg,
                'length': length,
                'type': geo_type,
                'data': geo_data
            })
        lane_sections = []

        # Extract lane section information from <lanes>
        for lane_section in road.find('lanes').findall('laneSection'):
            lane_s = float(lane_section.get('s'))
            lanes = []

            # Extract individual lanes within this lane section
            for lane in lane_section.findall('lane'):
                lane_id = int(lane.get('id'))
                widths = []

                # Get lane width polynomial coefficients for different segments
                for width in lane.findall('width'):
                    width_s_offset = float(width.get('sOffset'))
                    a, b, c, d = map(float, [width.get('a'), width.get('b'), width.get('c'), width.get('d')])
                    widths.append({
                        'sOffset': width_s_offset,
                        'a': a, 'b': b, 'c': c, 'd': d
                    })
                lanes.append({'id': lane_id, 'widths': widths})

            lane_sections.append({'s': lane_s, 'lanes': lanes})

        # Collect road data
        roads.append({
            'id': road_id,
            'length': length,
            'geometries': geometries,
            'laneSections': lane_sections
        })

    return roads

def world_to_reference_line(x, y, hdg, roads):
    """Transforms world coordinates (x, y, hdg) into reference line coordinates (s, t) with roadId."""
    best_match = None
    min_heading_diff = float('inf')

    for road in roads:
        road_id = road['id']
        for geometry in road['geometries']:
            x0, y0, geo_hdg, length = geometry['x'], geometry['y'], geometry['hdg'], geometry['length']

            # Project (x, y) onto the current geometry segment
            dx, dy = x - x0, y - y0
            s_local = dx * math.cos(geo_hdg) + dy * math.sin(geo_hdg)  # Along the segment
            t = -dx * math.sin(geo_hdg) + dy * math.cos(geo_hdg)  # Perpendicular to the segment

            if 0 <= s_local <= length:
                s_global = geometry['s'] + s_local

                # Calculate heading difference
                heading_diff = abs((hdg - geo_hdg + math.pi) % (2 * math.pi) - math.pi)

                if heading_diff < min_heading_diff:
                    min_heading_diff = heading_diff
                    best_match = (road_id, s_global, t)

    if best_match:
        return best_match

    raise ValueError("Point is outside the road geometry or heading mismatch.")

def parse_odr_file(file_path, road_id, s_coordinate):
    """
    Parses the OpenDRIVE (.xodr) file to extract the width polynomial coefficients
    for lane ID -1 at a given s-coordinate on a specified road.
    
    Parameters:
        file_path (str): Path to the .xodr file.
        road_id (str): The ID of the road to be searched.
        s_coordinate (float): Longitudinal position along the road (s).
    
    Returns:
        dict or None: Dictionary containing lane width coefficients ('a', 'b', 'c', 'd'), 
                      the distance along the lane section ('ds'), and the width sOffset.
                      Returns None if data not found.
    """

    tree = ET.parse(file_path)
    root = tree.getroot()

    # Search for the road element with the specified road_id
    for road in root.findall('road'):
        if road.get('id') == road_id:
            lane_sections = road.find('lanes').findall('laneSection')
            correct_lane_section = None

            # Find the lane section whose 's' value is less than or equal to the target s_coordinate
            for lane_section in lane_sections:
                lane_section_s = float(lane_section.get('s'))
                if s_coordinate < lane_section_s:
                    break
                correct_lane_section = lane_section

            if correct_lane_section is not None:
                lane_section_s = float(correct_lane_section.get('s'))
                ds = s_coordinate - lane_section_s

                # Look for lane ID -1 
                for lane in correct_lane_section.find('right').findall('lane'):
                    if lane.get('id') == '-1':
                        width_elements = lane.findall('width')
                        chosen_width_element = None

                        # Select the last width entry where sOffset <= ds
                        for width_element in width_elements:
                            sOffset = float(width_element.get('sOffset'))
                            if ds >= sOffset:
                                chosen_width_element = width_element
                            else:
                                break

                        # Extract and return polynomial coefficients if a matching width element is found
                        if chosen_width_element is not None:
                            lane_data = {
                                'a': float(chosen_width_element.get('a')),
                                'b': float(chosen_width_element.get('b')),
                                'c': float(chosen_width_element.get('c')),
                                'd': float(chosen_width_element.get('d')),
                                'ds': ds - float(chosen_width_element.get('sOffset')),
                                'sOffset': sOffset 
                            }
                            return lane_data
    
    # Return None if road or lane or suitable width entry not found
    return None

def calculate_lane_width(lane_data):
    """
    Calculate the width of a lane at a given ds using the polynomial coefficients.
    """
    a = lane_data['a']
    b = lane_data['b']
    c = lane_data['c']
    d = lane_data['d']
    ds = lane_data['ds']
    
    width = a + b * ds + c * ds**2 + d * ds**3
    return width

def world_to_lane_coordinates(x, y, h, opendrive_file):
    """
    Transforms world coordinates (x, y, h) into correct lane coordinates (s, t) with roadId,
    considering the adjustment of the t-coordinate according to the lane offset defined in the A9 ODR file.
    """

    # Parse the OpenDRIVE file
    roads = parse_opendrive(opendrive_file)

    try:
        # Transform world coordinates to reference line coordinates
        road_id, s, t = world_to_reference_line(x, y, h, roads)
        #print(f"World point ({x}, {y}), heading {h} -> Road ID: {road_id}, Reference line (s={s}, t={t})")

        # Parse the ODR file to get lane width data for lane_id -1
        lane_data = parse_odr_file(opendrive_file, road_id, s)
        if lane_data:
            # Calculate the width at the given s_coordinate
            width = calculate_lane_width(lane_data)
            #print(f"The width of lane_id -1 at s_coordinate {s} for road_id {road_id} is {width:.6f} meters")

            # Adjust t with lane offset
            adjusted_t = t + (-0.5 * width)
            #print(f"Adjusted t with lane offset: {adjusted_t}")

            # Return the results
            return s, adjusted_t, road_id
        else:
            print(f"Lane data for lane_id -1 not found for road_id {road_id}")
            return None
    except ValueError as e:
        print(e)
        return None


def gps_to_lane_coordinates( lat, lon, azimuth, opendrive_file):
    """ Transforms GPS-Position (lat, lon, azimuth) into correct lane coordinates (s, t) with roadId """

    # Transform GPS-Position to UTM-Coordinates
    x, y, h = gps_to_utm(lat, lon, azimuth)
    # print(f"Final UTM Coordinates: x={x}, y={y}, h={h}")

    # Transform world coordinates to lane coordinates (s, t) with roadId, considering the adjustment of the t-coordinate according to the lane offset defined in the A9 ODR file.
    result = world_to_lane_coordinates(x, y, h, opendrive_file)
    if result:
        s, adjusted_t, road_id = result
        # print(f"Results -> s: {s}, t: {adjusted_t}, road_id: {road_id}")

    return s, adjusted_t, road_id