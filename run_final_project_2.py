import pandas as pd
import numpy as np
import requests
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import folium
import polyline


def solve_complete_project_with_viz():
    # ... (All of the data generation, pre-processing, and solving code is IDENTICAL to before) ...
    print("--- Phase 1 & 2: Generating and Pre-processing Data ---")
    DEPOT_COORDINATES = [12.93580, 77.62590];
    NUM_ORDERS = 50;
    VEHICLE_CAPACITY = 8;
    NUM_VEHICLES = 12;
    CUSTOMER_DELIVERY_HOURS = 3.0;
    MAX_ROUTE_TIME_SECONDS = int(10 * 3600)
    order_locations = [{'OriginalOrderID': i + 1,
                        'Latitude': DEPOT_COORDINATES[0] + (4.0 / 111.1) * np.sqrt(np.random.rand()) * np.cos(
                            2 * np.pi * np.random.rand()),
                        'Longitude': DEPOT_COORDINATES[1] + (4.0 / 111.1) * np.sqrt(np.random.rand()) * np.sin(
                            2 * np.pi * np.random.rand())} for i in range(NUM_ORDERS)]
    orders_df = pd.DataFrame(order_locations)
    all_coords = [DEPOT_COORDINATES] + list(zip(orders_df['Latitude'], orders_df['Longitude']))
    try:
        all_coords_lon_lat = [[lon, lat] for lat, lon in all_coords];
        coords_string = ";".join([f"{lon},{lat}" for lon, lat in all_coords_lon_lat]);
        url = f"http://router.project-osrm.org/table/v1/driving/{coords_string}?annotations=duration";
        response = requests.get(url, timeout=60);
        response.raise_for_status();
        time_matrix_raw = response.json()['durations']
    except Exception as e:
        print(f"FATAL ERROR: Could not get data from OSRM API. {e}"); return
    penalty_value = 999999;
    reachable_indices = [i for i, row in enumerate(time_matrix_raw) if
                         time_matrix_raw[i][0] is not None and time_matrix_raw[0][i] is not None];
    unreachable_nodes = [i for i in range(NUM_ORDERS + 1) if i not in reachable_indices and i != 0]
    print(f"Identified {len(unreachable_nodes)} unreachable customer(s): {unreachable_nodes}");
    clean_time_matrix = np.array(time_matrix_raw)[np.ix_(reachable_indices, reachable_indices)];
    clean_time_matrix[clean_time_matrix == None] = penalty_value;
    clean_time_matrix = clean_time_matrix.astype(np.int64);
    reachable_orders_df = orders_df[orders_df['OriginalOrderID'].isin(reachable_indices)]
    print("\n--- Phase 3: Solving the Cleaned Problem with OR-Tools ---")
    num_locations = len(clean_time_matrix);
    manager = pywrapcp.RoutingIndexManager(num_locations, NUM_VEHICLES, 0);
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        return clean_time_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(time_callback);
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    dimension_name = 'Time';
    routing.AddDimension(transit_callback_index, 0, MAX_ROUTE_TIME_SECONDS, True, dimension_name);
    time_dimension = routing.GetDimensionOrDie(dimension_name)
    demands = [1] * num_locations;
    demands[0] = 0

    def demand_callback(from_index):
        return demands[manager.IndexToNode(from_index)]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback);
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, [VEHICLE_CAPACITY] * NUM_VEHICLES, True,
                                            'Capacity')
    time_windows = [(0, int(CUSTOMER_DELIVERY_HOURS * 3600))] * num_locations;
    time_windows[0] = (0, MAX_ROUTE_TIME_SECONDS)
    for i in range(num_locations): time_dimension.CumulVar(manager.NodeToIndex(i)).SetRange(time_windows[i][0],
                                                                                            time_windows[i][1])
    for i in range(NUM_VEHICLES): routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
    search_parameters = pywrapcp.DefaultRoutingSearchParameters();
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC;
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH;
    search_parameters.time_limit.FromSeconds(30)
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        final_routes_dict = print_solution(solution, manager, routing, clean_time_matrix, reachable_indices)
        create_solution_map(final_routes_dict, reachable_orders_df, DEPOT_COORDINATES)
    else:
        print("\n" + "=" * 50 + "\n>>> FAILURE: NO SOLUTION FOUND <<<\n" + "=" * 50)


# ==============================================================================
# >> UPDATED VISUALIZATION FUNCTION <<
# ==============================================================================
def create_solution_map(routes_dict, orders_df, depot_coords):
    """Creates and saves an interactive Folium map with road-snapped routes."""
    print("\n--- Creating Solution Map with Road-Snapped Routes ---")

    m = folium.Map(location=depot_coords, zoom_start=14)
    folium.Marker(depot_coords, popup="Depot (0)", icon=folium.Icon(color="red", icon="home")).add_to(m)
    colors = ['blue', 'green', 'purple', 'orange', 'darkred', 'lightred', 'beige', 'darkblue', 'darkgreen', 'cadetblue',
              'pink', 'lightblue']

    # Create a dictionary to map OriginalOrderID to its coordinates for easy lookup
    coord_map = orders_df.set_index('OriginalOrderID').to_dict('index')
    coord_map[0] = {'Latitude': depot_coords[0], 'Longitude': depot_coords[1]}

    for i, (vehicle_id, route) in enumerate(routes_dict.items()):
        route_color = colors[i % len(colors)]

        # --- NEW: Get detailed route geometry for each leg ---
        full_route_geometry = []
        for j in range(len(route) - 1):
            start_node_id = route[j]
            end_node_id = route[j + 1]

            start_coords = coord_map[start_node_id]
            end_coords = coord_map[end_node_id]

            # Call OSRM route service
            url = f"http://router.project-osrm.org/route/v1/driving/{start_coords['Longitude']},{start_coords['Latitude']};{end_coords['Longitude']},{end_coords['Latitude']}?overview=full&geometries=polyline"
            r = requests.get(url)
            route_geometry = r.json()['routes'][0]['geometry']

            # Decode the polyline and add to our full route list
            decoded_route = polyline.decode(route_geometry)
            full_route_geometry.extend(decoded_route)

        # Add the detailed route line to the map
        folium.PolyLine(full_route_geometry, color=route_color, weight=3, opacity=0.8,
                        popup=f"Vehicle {vehicle_id}").add_to(m)

        # Add markers for each customer on this route
        for node_idx in route[1:-1]:
            customer_coords = [coord_map[node_idx]['Latitude'], coord_map[node_idx]['Longitude']]
            folium.CircleMarker(
                customer_coords, radius=5, color=route_color, fill=True,
                popup=f"Customer {node_idx}<br><b>Vehicle {vehicle_id}</b>"
            ).add_to(m)

    map_filename = "optimized_routes_map.html"
    m.save(map_filename)
    print(f"--- Map saved to '{map_filename}' ---")


# ==============================================================================
# >> The print_solution function and main execution block are unchanged <<
# ==============================================================================
def print_solution(solution, manager, routing, time_matrix, reachable_indices):
    """Prints the solution and returns it as a dictionary."""
    print("\n--- Final Optimized Solution ---")
    total_time, total_routes = 0, 0
    final_routes_dict = {}
    original_indices = np.array(reachable_indices)
    for vehicle_id in range(routing.vehicles()):
        index = routing.Start(vehicle_id)
        if routing.IsEnd(solution.Value(routing.NextVar(index))): continue
        total_routes += 1
        route_nodes, route_time = [], 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_nodes.append(int(original_indices[node_index]))
            previous_index, index = index, solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        route_nodes.append(0)
        final_routes_dict[vehicle_id] = route_nodes
        print(
            f"Route for vehicle {vehicle_id} ({len(route_nodes) - 2} stops): {' -> '.join(map(str, route_nodes))} | Time: {route_time}s")
        total_time += route_time
    print(f"\nTotal Number of Routes: {total_routes}");
    print(f"Total Travel Time: {total_time}s ({total_time / 3600:.2f} hours)")
    if total_routes > 0:
        average_time = total_time / total_routes
        print(f"Average Route Time: {int(average_time)}s ({average_time / 60:.2f} minutes)")
    return final_routes_dict


if __name__ == "__main__":
    solve_complete_project_with_viz()