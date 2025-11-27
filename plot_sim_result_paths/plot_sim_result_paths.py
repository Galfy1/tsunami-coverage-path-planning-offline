import re
import os
import folium
import pickle
import numpy as np

import matplotlib.cm as cm
import matplotlib.colors as colors

from geopy.distance import geodesic

# SIMLUATON LOGS CAN BE FOUND IN ~/.ros/log/


DIR_TO_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = DIR_TO_THIS_FILE + "/launch.log"

def path_length(coords):
    total = 0.0
    for (lat1, lon1), (lat2, lon2) in zip(coords, coords[1:]):
        total += geodesic((lat1, lon1), (lat2, lon2)).meters # great-circle distance
    return total

def total_turn_degree(coords):
    import numpy as np

    def bearing(pointA, pointB):
        lat1 = np.radians(pointA[0])
        lat2 = np.radians(pointB[0])
        diffLong = np.radians(pointB[1] - pointA[1])
        x = np.sin(diffLong) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(diffLong))
        initial_bearing = np.arctan2(x, y)
        initial_bearing = np.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360 # (normalize to 0-360 degrees)
        return compass_bearing

    total_turn = 0.0
    for i in range(1, len(coords) - 1):
        bearing1 = bearing(coords[i - 1], coords[i])
        bearing2 = bearing(coords[i], coords[i + 1])
        angle_diff = abs(bearing2 - bearing1) 
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        total_turn += angle_diff
    return total_turn

# def turn_count(coords, angle_threshold=30):
#     import numpy as np

#     def bearing(pointA, pointB):
#         lat1 = np.radians(pointA[0])
#         lat2 = np.radians(pointB[0])
#         diffLong = np.radians(pointB[1] - pointA[1])
#         x = np.sin(diffLong) * np.cos(lat2)
#         y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(diffLong))
#         initial_bearing = np.arctan2(x, y)
#         initial_bearing = np.degrees(initial_bearing)
#         compass_bearing = (initial_bearing + 360) % 360
#         return compass_bearing

#     turns = 0
#     for i in range(1, len(coords) - 1):
#         bearing1 = bearing(coords[i - 1], coords[i])
#         bearing2 = bearing(coords[i], coords[i + 1])
#         angle_diff = abs(bearing2 - bearing1)
#         if angle_diff > 180:
#             angle_diff = 360 - angle_diff
#         if angle_diff >= angle_threshold:
#             turns += 1
#     return turns

def main(args=None) -> None:


    ############## Extract flight paths from log file ##############

    # Regex pattern to match the flight path log lines
    # Example line:
    # [application.drone_2]: FLIGHT PATH LOG: [(lat, lon), (lat, lon), ...]
    pattern = re.compile(
        r"\[(?P<namespace>[\w\.]+)\]:\s+FLIGHT PATH LOG:\s+(?P<coords>\[.*\])"
    )
    
    # Dictionary to hold extracted paths
    flight_paths = {}

    with open(LOG_FILE, "r") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                ns = match.group("namespace")
                coords_text = match.group("coords")
                # Safely evaluate list of tuples from the string
                try:
                    coords = eval(coords_text, {"__builtins__": None}, {})
                    flight_paths[ns] = coords
                except Exception as e:
                    print(f"Could not parse coords for {ns}: {e}")

    #print uav count:
    print(f"Number of drones with logged flight paths: {len(flight_paths)}")


    # Regex to capture:
    # Example lines:
    # [application.drone_2]: MISSION START TIME (us): 78500000
    # [application.drone_2]: MISSION END TIME (us): 158500000
    mission_time_pattern = re.compile(
        r"\[(?P<namespace>[\w\.]+)\]:\s+MISSION\s+(?P<type>START|END)\s+TIME\s+\(us\):\s+(?P<value>\d+)"
    )

    mission_times = {}  # { namespace: {"start": int | None, "end": int | None} }

    with open(LOG_FILE, "r") as f:
        for line in f:
            match = mission_time_pattern.search(line)
            if match:
                ns = match.group("namespace")
                ttype = match.group("type").lower()   # "start" or "end"
                value = int(match.group("value"))

                # Create entry once per namespace
                if ns not in mission_times:
                    mission_times[ns] = {"start": None, "end": None}

                # Always update the relevant field
                mission_times[ns][ttype] = value

    # # Print the results
    # for ns, times in mission_times.items():
    #     print(f"{ns}: Start={times['start']}  End={times['end']}")

    # Find the full mission duration across all drones 
    all_start_times = [times["start"] for times in mission_times.values() if times["start"] is not None]
    all_end_times = [times["end"] for times in mission_times.values() if times["end"] is not None]
    if all_start_times and all_end_times:
        overall_start = min(all_start_times)
        overall_end = max(all_end_times)
        overall_duration = overall_end - overall_start
        print(f"Overall Mission Duration: {overall_duration / 1_000_000:.2f} seconds")

    # Compute the Coefficient of Variation (CV) of mission durations https://www.geeksforgeeks.org/data-science/coefficient-of-variation-meaning-formula-and-examples/ 
    mission_durations = []
    for times in mission_times.values():
        if times["start"] is not None and times["end"] is not None:
            duration = times["end"] - times["start"]
            mission_durations.append(duration)
    if mission_durations:
        mean_duration = np.mean(mission_durations)
        std_duration = np.std(mission_durations)
        cv_duration = std_duration / mean_duration if mean_duration > 0 else 0
        cv_duration_pct = cv_duration * 100
        print(f"Mission Duration Coefficient of Variation (CV): {cv_duration_pct:.2f}% (lower is better)")


    # Find UAV utilizaton (ballance between uav missions times)
    # total_uav_time = 0
    # for ns, times in mission_times.items():
    #     if times["start"] is not None and times["end"] is not None:
    #         uav_time = times["end"] - times["start"]
    #         total_uav_time += uav_time
    #         print(f"{ns} Mission Duration: {uav_time / 1_000_000:.2f} seconds")

    # print(f"Total UAV Utilization Time: {total_uav_time / 1_000_000:.2f} seconds")

    # Print total (all drones combined) path lengths and turn degrees metrics
    total_path_length = 0.0
    total_turn_degrees = 0.0
    for drone, path in flight_paths.items():
        pl = path_length(path)
        td = total_turn_degree(path)
        total_path_length += pl
        total_turn_degrees += td
        # print(f"{drone} Path Length: {pl:.2f} meters, Turn Degree: {td:.2f} degrees")
    print(f"Total Path Length (all drones): {total_path_length:.2f} meters")
    print(f"Total Turn Degree (all drones): {total_turn_degrees:.2f} degrees")



    ############## Plot paths using folium ##############

    # Remove the last point from each path to not plot path back to home (makes the plot cleaner)
    for drone in flight_paths:
        if flight_paths[drone]:
            flight_paths[drone] = flight_paths[drone][:-1]

    if flight_paths:
        # Center map at the first coordinate of the first drone
        first_drone = next(iter(flight_paths))
        start_coords = flight_paths[first_drone][0]
        m = folium.Map(location=start_coords, zoom_start=18)

        path_colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'black', 'darkblue', 'gray', 'white']

        for i, (drone, path) in enumerate(flight_paths.items()):
            if not path:
                continue
            folium.PolyLine(path, color=path_colors[i % len(path_colors)], weight=3.0, opacity=1, tooltip=drone).add_to(m)
            # folium.PolyLine(path, color="black", weight=2.5, opacity=1, tooltip=drone).add_to(m)

            # print(f"{drone} path length: {path_length(path):.2f} meters")
            # print(f"{drone} turn count: {turn_count(path)} turns")
            # print(f"{drone} turn degree: {total_turn_degree(path):.2f} degrees")
            # Mark start and end points
            folium.Marker(path[0], popup=f"{drone} Start", icon=folium.Icon(color='green')).add_to(m)
            folium.Marker(path[-1], popup=f"{drone} End", icon=folium.Icon(color='red')).add_to(m)
    else:
        print("No flight paths found in the log.")


    ############## Add bf traversal points to map ##############

    # with open('bf_traversal.pkl', 'rb') as fp:
    #     bf_data = pickle.load(fp)
    #     bf_traversal_gps = bf_data['bf_traversal_gps']

    PLOT_POINT_SOURCE = "tsunami" # "tsunami" or "partition"

    if PLOT_POINT_SOURCE == 'tsunami':
        pkl_path = DIR_TO_THIS_FILE + "/tsunami_offline_data.pkl"
    elif PLOT_POINT_SOURCE == 'partition':
        pkl_path = DIR_TO_THIS_FILE + "/partition_offline_data.pkl"
    with open(pkl_path, 'rb') as fp:
        data_loaded = pickle.load(fp)
    fly_nofly_grid = data_loaded['fly_nofly_grid']
    fly_nofly_grid_gps = data_loaded['fly_nofly_grid_gps'] # used to translate grid cells to gps coords

    # Extract flyable points
    bf_traversal_gps = []
    for (i, j), cell_type in np.ndenumerate(fly_nofly_grid):
        if cell_type == 1: # flyable
            bf_traversal_gps.append(fly_nofly_grid_gps[i][j])
    # Plot BF traversal points
    for i, (lat, lon) in enumerate(bf_traversal_gps):
        # (Add points to the map with colors based on their order)
        folium.CircleMarker(location=[lat, lon], radius=1, color='black', fill=True, fill_opacity=0.7).add_to(m)

    
    ############## Save map to HTML file ##############
    map_file = DIR_TO_THIS_FILE + "/drone_flight_paths.html"
    m.save(map_file)
    print(f"Flight paths and BFT map saved to {map_file}")

    

if __name__ == '__main__':
    main()