from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import math




def _get_line_properties(linestring: LineString):
    """Extracts useful geometric info from a two-point LineString."""
    if len(linestring.coords) != 2:
        raise ValueError("This function only supports LineStrings with exactly two points")

    (x1, y1), (x2, y2) = linestring.coords

    # compute direction vector (cartesian form)
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)

    if length == 0:
        raise ValueError("Cannot operate on a zero-length line")

    # normalize direction vector (i.e. scale it down to length 1) - this way, dx_normalized and dy_normalized represents how much of the length is in x and y direction respectively
    dx_normalized = dx / length
    dy_normalized = dy / length

    return (x1, y1), (x2, y2), dx_normalized, dy_normalized, length


def scale_linestring(linestring: LineString, scale: float) -> LineString:
    (x1, y1), (x2, y2), dx_n, dy_n, length = _get_line_properties(linestring) # (dx_n means "x component of normalized direction vector")

    new_length = length * scale
    extend_length = (new_length - length) / 2

    # move each endpoint along the direction vector
    # (e.g. (dx_normalized * extend_length) is basically saying "how much of the extend_length is in the x direction")
    new_p1 = (x1 - dx_n * extend_length, y1 - dy_n * extend_length) # "-" is used to move point "backwards" along the direction vector
    new_p2 = (x2 + dx_n * extend_length, y2 + dy_n * extend_length) # "+" is used to move point "forwards" along the direction vector

    return LineString([new_p1, new_p2])


def extend_p2_in_linestring(linestring: LineString, extend_length: float) -> LineString:
    (x1, y1), (x2, y2), dx_n, dy_n, _ = _get_line_properties(linestring)

    # move endpoint p2 along the direction vector
    new_p2 = (x2 + dx_n * extend_length, y2 + dy_n * extend_length) # "+" is used to move point "forwards" along the direction vector

    return LineString([(x1, y1), new_p2])


# https://www.matematikbanken.dk/id/158/ 

# handy when dealing with floating point precision comparisons of gps coords
def round_waypoint(coord, decimals=6): # 6 decimal places is roughly ~0.1 meter precision
    return (round(coord[0], decimals), round(coord[1], decimals))

def snap_to_closest_free_spot(snap_line: LineString, coord: Point, grid_res, prealigned_to_aligned_coords_dict) -> Point:
	
    closest_point_on_snap_line = nearest_points(snap_line, coord)[0] # https://shapely.readthedocs.io/en/2.0.4/manual.html#nearest-points 
    direction_line = LineString([closest_point_on_snap_line, coord]) # line from coord to snap line
    direction_line_length = direction_line.length
    if direction_line_length == 0.0:
        return coord # already on the line, no need to snap

    extend_amount = round(direction_line_length / grid_res) * grid_res - direction_line_length # https://stackoverflow.com/questions/7859147/round-in-numpy-to-nearest-step
    extended_direction_line = extend_p2_in_linestring(direction_line, extend_amount) # p2 is the waypoint
    coord_aligned = extended_direction_line.coords[1] # extract p2 from linestring (i.e. the extended coord) (.coords are x,y tuples, not Point object)

    # check if this coord has already been assigned an aligned coord
    dict_search_value = round_waypoint((coord_aligned[1], coord_aligned[0])) # (lat, lon) for searching in dict
    if dict_search_value in {round_waypoint(v) for v in prealigned_to_aligned_coords_dict.values()}:
        # already assigned, try to snap using floor instead of round
        extend_amount = math.floor(direction_line_length / grid_res) * grid_res - direction_line_length
        extended_direction_line = extend_p2_in_linestring(direction_line, extend_amount) 
        coord_aligned = extended_direction_line.coords[1] # 

        # check if this coord has already been assigned an aligned coord
        dict_search_value = round_waypoint((coord_aligned[1], coord_aligned[0])) # (lat, lon) for searching in dict
        if dict_search_value in {round_waypoint(v) for v in prealigned_to_aligned_coords_dict.values()}:
            # already assigned, try to snap using ceil instead of round
            extend_amount = math.ceil(direction_line_length / grid_res) * grid_res - direction_line_length
            extended_direction_line = extend_p2_in_linestring(direction_line, extend_amount) 
            coord_aligned = extended_direction_line.coords[1] 

            dict_search_value = round_waypoint((coord_aligned[1], coord_aligned[0])) # (lat, lon) for searching in dict
            if dict_search_value in {round_waypoint(v) for v in prealigned_to_aligned_coords_dict.values()}:
                #raise ValueError("Could not find a free spot to snap the coord to without overlapping with an already assigned coord")
                extend_amount = round(direction_line_length / (grid_res/2)) * (grid_res/2) - direction_line_length
                extended_direction_line = extend_p2_in_linestring(direction_line, extend_amount) 
                coord_aligned = extended_direction_line.coords[1] 

    return Point(coord_aligned[0], coord_aligned[1]) 


def align_coords_with_centroid_angle(polygon: Polygon, home_gps, x_axis_coords, y_axis_coords, grid_res_x, grid_res_y, centroid_alignment_deviation=0.8):
    
    prealigned_to_aligned_coords_dict = {} # (note, this holds entries for all points, not just the ones inside the polygon - but that's okay)

    # https://en.wikipedia.org/wiki/Centroid#Of_a_polygon 
    # https://stackoverflow.com/questions/75699024/finding-the-centroid-of-a-polygon-in-python 
    # https://shapely.readthedocs.io/en/stable/reference/shapely.Polygon.html#shapely.Polygon.centroid  
    centroid = polygon.centroid  # Point(lon, lat)
    centroid_line = LineString([Point(home_gps[1], home_gps[0]), centroid]) # Point(lon, lat)
    centroid_line_long = scale_linestring(centroid_line, 20) # make it longer in both directions to ensure it crosses the entire polygon. 20 is arbitrary, just needs to be large enough.

    arbitrary_coord = Point(x_axis_coords[5], y_axis_coords[5])
    closest_point_on_centroid_line = nearest_points(centroid_line_long, arbitrary_coord)[0] 
    perpen_line = LineString([closest_point_on_centroid_line, arbitrary_coord]) # this is a line that is perpendicular to the centroid line
    perpen_line_long = scale_linestring(perpen_line, 20) # (same reason as for centroid_line_long)


    for x_coord in x_axis_coords:
        for y_coord in y_axis_coords:

            coord = Point(x_coord, y_coord) # here, coord is a point! (x,y) aka (lon, lat) - NOT the usual (lat, lon)

            #### SNAPPING PERPENDICULAR TO CENTROID LINE ####
            grid_res = grid_res_y*centroid_alignment_deviation + 0.00000001 # TODO FOR DEBUGGING DEN SKAL ÆNDRES!!! UD FRA TRIGONOMETRI OG CENTROID VINKEL. MEN DEN SKAL STADIG GANGES MED CENTROID_ALIGNMENT_DEVIATION
            coord_aligned = snap_to_closest_free_spot(centroid_line_long, coord, grid_res, prealigned_to_aligned_coords_dict)

            #### SNAPPING PARALLEL TO CENTROID LINE ####
            grid_res = grid_res_x*centroid_alignment_deviation + 0.00000001 # TODO FOR DEBUGGING DEN SKAL ÆNDRES!!! UD FRA TRIGONOMETRI OG CENTROID VINKEL
            coord_aligned = snap_to_closest_free_spot(perpen_line_long, coord_aligned, grid_res, prealigned_to_aligned_coords_dict)
            
            #### STORE ALLIGNED COORD IN LOOKUP DICT ####
            coord_aligned = (coord_aligned.x, coord_aligned.y) # convert from Point to tuple (for storage in dict)
            prealigned_to_aligned_coords_dict[(coord.y, coord.x)] = (coord_aligned[1], coord_aligned[0]) # (lon, lat) 

    return prealigned_to_aligned_coords_dict, centroid, centroid_line, centroid_line_long # (centroid and long_centroid_line can be used for plotting)

