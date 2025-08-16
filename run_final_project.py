import pandas as pd
import numpy as np
import requests
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import follium

def solve_routing_problem():
    # ==============================================================================
    # PHASE 1: DATA GENERATION
    # ==============================================================================
    print("--- Phase 1: Generating Data ---")
    DEPOT_COORDINATES = [12.93580, 77.62590]
    NUM_ORDERS = 50
    VEHICLE_CAPACITY = 5 # capacity to hold bags i.e 1 vehicle can carry 5 bags
    NUM_VEHICLES = 12  # Start with some slack

    # ===============================================================================
    # ADDITIONAL CONSTRAINTS
    # ===============================================================================
    MAX_ROUTE_TIME_HOURS = 10.0 # max time of vehicle
    MIN_ROUTE_TIME_HOURS = 0.5  # min time of vehicle (this is according to labour law of INDIA)
    MAX_ROUTE_TIME_SECONDS = int(MAX_ROUTE_TIME_HOURS * 3600)
    MIN_ROUTE_TIME_SECONDS = int(MIN_ROUTE_TIME_HOURS * 3600)

    # ... (The rest of the data generation code is the same)
    order_locations = []
    for i in range(NUM_ORDERS):
        r = (4.0 / 111.1) * np.sqrt(np.random.rand())
        theta = 2 * np.pi * np.random.rand()
        order_locations.append({'OriginalOrderID': i + 1, 'Latitude': DEPOT_COORDINATES[0] + r * np.cos(theta),
                                'Longitude': DEPOT_COORDINATES[1] + r * np.sin(theta)})

    orders_df = pd.DataFrame(order_locations)
    all_coords = [DEPOT_COORDINATES] + list(zip(orders_df['Latitude'], orders_df['Longitude']))
    all_coords_lon_lat = [[lon, lat] for lat, lon in all_coords]
    coords_string = ";".join([f"{lon},{lat}" for lon, lat in all_coords_lon_lat])
    url = f"http://router.project-osrm.org/table/v1/driving/{coords_string}?annotations=duration"

    try:
        response = requests.get(url, timeout=60)
        response.raise_for_status()
        time_matrix_raw = response.json()['durations']
    except Exception as e:
        print(f"FATAL ERROR: Could not get data from OSRM API. {e}")
        return

    # ==============================================================================
    # PHASE 1.5: DEFINE TIME
    # ==============================================================================
    # Let's us assume the depot is open all day (eg: 10 hours)
    # and all deliveries must be made within 1 hour (3600 seconds) of the start
    time_windows = [(0, 10800)] * (NUM_ORDERS + 1)
    time_windows[0] = (0, 10800) # Depot window: 10 hours
    print("Defined delivery time windows (0 to 3600 seconds for customers).")


    # ==============================================================================
    # PHASE 2: DATA PRE-PROCESSING (THE NEW, GUARANTEED FIX)
    # ==============================================================================
    print("\n--- Phase 2: Pre-processing Data to Remove Unreachable Nodes ---")

    penalty_value = 999999
    # Node indices are 0 for depot, 1 to 50 for customers
    all_node_indices = list(range(NUM_ORDERS + 1))

    reachable_indices = []
    unreachable_nodes = []

    for i in all_node_indices:
        # A node is reachable if it can get to the depot AND the depot can get to it.
        is_reachable = (time_matrix_raw[i][0] is not None) and (time_matrix_raw[0][i] is not None)
        if is_reachable:
            reachable_indices.append(i)
        elif i != 0:  # Don't list the depot as unreachable
            unreachable_nodes.append(i)

    print(f"Identified {len(unreachable_nodes)} unreachable customer(s): {unreachable_nodes}")
    print(f"Proceeding with {len(reachable_indices) - 1} reachable customers.")

    # Create a new, clean time matrix and orders list
    clean_time_matrix = np.array(time_matrix_raw)[np.ix_(reachable_indices, reachable_indices)]
    # Replace any remaining Nones (e.g. between two island customers)
    clean_time_matrix[clean_time_matrix == None] = penalty_value
    clean_time_matrix = clean_time_matrix.astype(np.int64)

    # Map old indices to new indices (0, 1, 2, 3...)
    new_idx_map = {old_idx: new_idx for new_idx, old_idx in enumerate(reachable_indices)}

    # ==============================================================================
    # PHASE 3: SOLVING THE CLEANED PROBLEM
    # ==============================================================================
    print("\n--- Phase 3: Solving the Cleaned Problem with OR-Tools ---")

    # Create the routing model on the CLEANED data
    num_locations = len(clean_time_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, NUM_VEHICLES, 0)  # Depot is always index 0
    routing = pywrapcp.RoutingModel(manager)

    # --- Add Constraints ---
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return clean_time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # ============================================================================
    # ADDING THE TIME DIMENSION
    # ============================================================================
    dimension_name = 'Time'
    routing.AddDimension(
        transit_callback_index,
        0, # no slack
         MAX_ROUTE_TIME_SECONDS, # Vehicle maximum travel time (10 hours)
        True, # Start cumul to zero
        dimension_name)
    time_dimension = routing.GetDimensionOrDie(dimension_name)

    demands = [1] * num_locations
    demands[0] = 0  # Depot demand is 0

    def demand_callback(from_index):
        return demands[manager.IndexToNode(from_index)]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, [VEHICLE_CAPACITY] * NUM_VEHICLES, True,
                                            'Capacity')

    # ============================================================================
    # APPLY TIME WINDOW CONSTRAINTS
    # ============================================================================
    # this maps our original node indices (e.g., customer 45) to the cleaned list indices (e.g 41)
    original_indices_list = list(reachable_indices)

    # Add time window constraints for each location except the depot
    for original_location_idx, time_window in enumerate(time_windows):
        if original_location_idx == 0:  # Skip depot
            continue
        # We only add constraints for nodes that are reachable
        if original_location_idx in original_indices_list:
            # Find th new, cleaned index for this location
            new_index = original_indices_list.index(original_location_idx)
            index = manager.NodeToIndex(new_index)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add the time window constraints for each vehicles start node (the depot)
    for vehicle_id in range(NUM_VEHICLES):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(time_windows[0][0], time_windows[0][1])

    # ===================================================================================
    # ADD MINIMUM TIME CONSTRAINT
    # ===================================================================================
    # Add a minimum work time for each vehicle that is used
    for vehicle_id in range(NUM_VEHICLES):
        index = routing.End(vehicle_id)
        # This line gets the variable representing the total time of a vehicles route
        route_time_variable = time_dimension.CumulVar(index)\

        # This line gets the variable representing if a vehicle is used (1) or not (0)
        vehicle_active_variable = routing.ActiveVar(vehicle_id)

        # Add the constraints: RouteTime >= MinTime * IsVehicleActive
        routing.solver().Add(route_time_variable >= (MIN_ROUTE_TIME_SECONDS * vehicle_active_variable))


    # --- Set Search Parameters and Solve ---
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(30)

    solution = routing.SolveWithParameters(search_parameters)

    # --- Print Solution ---
    if solution:
        print("\n--- Final Optimized Solution ---")
        total_time = 0
        total_routes = 0
        original_indices = np.array(list(reachable_indices))
        for vehicle_id in range(NUM_VEHICLES):
            index = routing.Start(vehicle_id)
            if routing.IsEnd(solution.Value(routing.NextVar(index))):
                continue  # Skip unused vehicles

            total_routes += 1
            route_nodes = []
            route_time = 0
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_nodes.append(int(original_indices[node_index]))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_time += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

            route_nodes.append(0)  # End at depot
            print(
                f"Route {vehicle_id} ({len(route_nodes) - 2} stops): {' -> '.join(map(str, route_nodes))} | Time: {route_time}s")
            total_time += route_time

        print(f"\nTotal Number of Routes: {total_routes}")
        print(f"Total Travel Time: {total_time}s ({total_time / 3600:.2f} hours)")
    else:
        print("No solution found!")

# Run the entire project
solve_routing_problem()

# Average Route Calculation
''' 
Average Route Time = Total Travel Time / Total Number of Routes
Total Travel Time = 21745 seconds
Total Number of Routes = 12
Average Route Time - 21745 seconds / 12 routes
Average Route Time = 1812 seconds
1812 / 60 = 30.2 minutes
Therefor (Average Route Time = 30.2 Minutes

'''