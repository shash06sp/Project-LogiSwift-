import pandas as pd
import numpy as np
import requests
import os


# ==============================================================================
# PHASE 1: DATA GENERATION LOGIC (WITH ENHANCED ERROR REPORTING)
# ==============================================================================
def generate_routing_data():
    """Generates and returns the orders DataFrame and the time matrix."""
    print("--- Phase 1: Generating Fresh Data ---")

    # --- Constants ---
    Depot_Coordinates = [12.93580, 77.62590]
    NUM_ORDERS = 50
    DELIVERY_RADIUS_KM = 4.0
    RADIUS_IN_DEGREES = DELIVERY_RADIUS_KM / 111.1

    print(f"Using Depot at {Depot_Coordinates}")

    # --- Generate Customer Locations ---
    order_locations = []
    for i in range(NUM_ORDERS):
        r = RADIUS_IN_DEGREES * np.sqrt(np.random.rand())
        theta = 2 * np.pi * np.random.rand()
        lat_offset = r * np.cos(theta)
        lon_offset = r * np.sin(theta)
        customer_lat = DEPOT_COORDINATES[0] + lat_offset
        customer_lon = DEPOT_COORDINATES[1] + lon_offset
        order_locations.append({'OrderID': i + 1, 'Latitude': customer_lat, 'Longitude': customer_lon})

    orders_df = pd.DataFrame(order_locations)
    print(f"Generated {len(orders_df)} customer locations.")

    # --- Build Time Matrix using OSRM API ---
    all_coords = [DEPOT_COORDINATES] + list(zip(orders_df['Latitude'], orders_df['Longitude']))
    all_coords_lon_lat = [[lon, lat] for lat, lon in all_coords]
    coords_string = ";".join([f"{lon},{lat}" for lon, lat in all_coords_lon_lat])
    url = f"http://router.project-osrm.org/table/v1/driving/{coords_string}?annotations=duration"

    print("Making API call to OSRM...")
    try:
        response = requests.get(url, timeout=30)
        response.raise_for_status()  # This will raise an error for bad status codes (like 4xx or 5xx)
        print("API call successful.")
        data = response.json()

        if 'durations' in data:
            durations = data['durations']
            penalty_value = 999999
            none_count = 0
            for i, row in enumerate(durations):
                for j, value in enumerate(row):
                    if value is None:
                        durations[i][j] = penalty_value
                        none_count += 1
            if none_count > 0:
                print(f"WARNING: Replaced {none_count} impossible routes with a penalty value.")

            time_matrix = np.array(durations, dtype=np.int64)
            print("Time matrix successfully created in memory.")
            return orders_df, time_matrix
        else:
            # This block now prints more details if the API response is unusual
            print("ERROR: 'durations' key was not found in the API response.")
            print("The OSRM server might be busy or returned an error message.")
            print("API Response Content:", data)
            return None, None

    except requests.exceptions.RequestException as e:
        # This block now prints the specific error
        print(f"--- FATAL ERROR: An exception occurred during the API request. ---")
        print(f"Error Details: {e}")
        return None, None


# ==============================================================================
# PHASE 2: BASELINE SOLVER LOGIC (No changes needed here)
# ==============================================================================
def solve_with_clarke_wright(orders_df, time_matrix):
    """Takes data as input and prints the baseline solution."""
    print("\n--- Phase 2: Solving with Baseline Algorithm (Clarke & Wright) ---")

    num_orders = len(orders_df)
    depot_idx = 0
    VEHICLE_CAPACITY = 5

    savings = []
    for i in range(1, num_orders + 1):
        for j in range(i + 1, num_orders + 1):
            saving_value = time_matrix[depot_idx, i] + time_matrix[depot_idx, j] - time_matrix[i, j]
            if saving_value > 0:
                savings.append((saving_value, i, j))
    savings.sort(key=lambda x: x[0], reverse=True)

    routes = {i: [depot_idx, i, depot_idx] for i in range(1, num_orders + 1)}

    def find_route(customer_idx, all_routes):
        return all_routes.get(customer_idx)

    for saving, i, j in savings:
        route_i = find_route(i, routes)
        route_j = find_route(j, routes)

        if route_i is not None and route_j is not None and id(route_i) != id(route_j):
            load_i = len(route_i) - 2
            load_j = len(route_j) - 2

            if load_i + load_j <= VEHICLE_CAPACITY:
                new_route = None
                if route_i[-2] == i and route_j[1] == j:
                    new_route = route_i[:-1] + route_j[1:]
                elif route_i[1] == i and route_j[1] == j:
                    route_i.reverse()
                    new_route = route_i[:-1] + route_j[1:]
                elif route_i[-2] == i and route_j[-2] == j:
                    route_j.reverse()
                    new_route = route_i[:-1] + route_j[1:]
                elif route_i[1] == i and route_j[-2] == j:
                    new_route = route_j[:-1] + route_i[1:]

                if new_route:
                    customers_in_new_route = new_route[1:-1]
                    for cust_idx in customers_in_new_route:
                        routes[cust_idx] = new_route

    final_routes = list({id(route): route for route in routes.values()}.values())
    total_time = 0
    print("\n--- Baseline Solution Results ---")
    for i, route in enumerate(final_routes):
        route_time = sum(time_matrix[start, end] for start, end in zip(route, route[1:]))
        print(
            f"Route {i + 1} ({len(route) - 2} stops): {' -> '.join(map(str, route))} | Time: {int(route_time)} seconds")
        total_time += route_time

    print(f"\nTotal Number of Routes: {len(final_routes)}")
    print(f"Total Travel Time: {int(total_time)} seconds ({total_time / 3600:.2f} hours)")


# ==============================================================================
# MAIN EXECUTION BLOCK (Now with a clearer final message)
# ==============================================================================
if __name__ == "__main__":
    orders_data, time_matrix_data = generate_routing_data()

    if orders_data is not None and time_matrix_data is not None:
        solve_with_clarke_wright(orders_data, time_matrix_data)
    else:
        print("\nExecution halted. Could not run the solver because the data generation phase failed.")