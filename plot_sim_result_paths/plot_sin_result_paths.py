import re
import os
import folium
import pickle

import matplotlib.cm as cm
import matplotlib.colors as colors

# SIMLUATON LOGS CAN BE FOUND IN ~/.ros/log/


DIR_TO_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = DIR_TO_THIS_FILE + "/launch.log"

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



    ############## Plot paths using folium ##############

    if flight_paths:
        # Center map at the first coordinate of the first drone
        first_drone = next(iter(flight_paths))
        start_coords = flight_paths[first_drone][0]
        m = folium.Map(location=start_coords, zoom_start=18)

        path_colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'lightred', 'beige', 'darkblue', 'darkgreen']

        for i, (drone, path) in enumerate(flight_paths.items()):
            if not path:
                continue
            folium.PolyLine(path, color=path_colors[i % len(path_colors)], weight=2.5, opacity=1, tooltip=drone).add_to(m)
            # Mark start and end points
            folium.Marker(path[0], popup=f"{drone} Start", icon=folium.Icon(color='green')).add_to(m)
            folium.Marker(path[-1], popup=f"{drone} End", icon=folium.Icon(color='red')).add_to(m)
    else:
        print("No flight paths found in the log.")


    ############## Add bf traversal points to map ##############

    with open('bf_traversal.pkl', 'rb') as fp:
        bf_data = pickle.load(fp)
        bf_traversal_gps = bf_data['bf_traversal_gps']


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