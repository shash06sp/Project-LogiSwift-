'''
This file consist code of combined file of generate_data and baseline_solver fiel
The reason to create this file cause in two script file the output is consistent

# BUT FOLLOW AND READ THOSE FILES FOR DETAILED EXPLANATION. IN THOSE FILES THE
LOGIC, STRUCTURE AND TOPIC IS EXPLAINED CLEARLY

'''

import pandas as pd
import numpy as np
import requests


def get_baseline_solution():
    """
    This single function generates data and solves for the Clarke & Wright baseline.
    It does not depend on any external files.
    """
    # ==============================================================================
    # PHASE 1: DATA GENERATION
    # ==============================================================================
    print("--- Phase 1: Generating fresh data in memory ---")

    DEPOT_COORDINATES = [12.93580, 77.62590]  # Correct road-snapped coordinate
    NUM_ORDERS = 50
    VEHICLE_CAPACITY = 8  # Use the same capacity as our final model for a fair comparison

    # --- Generate Customer Locations ---
    order_locations = [{'OrderID': i + 1,
                        'Latitude': DEPOT_COORDINATES[0] + (4.0 / 111.1) * np.sqrt(np.random.rand()) * np.cos(
                            2 * np.pi * np.random.rand()),
                        'Longitude': DEPOT_COORDINATES[1] + (4.0 / 111.1) * np.sqrt(np.random.rand()) * np.sin(
                            2 * np.pi * np.random.rand())} for i in range(NUM_ORDERS)]
    orders_df = pd.DataFrame(order_locations)

    # --- Build Time Matrix using OSRM API ---
    all_coords = [DEPOT_COORDINATES] + list(zip(orders_df['Latitude'], orders_df['Longitude']))
    all_coords_lon_lat = [[lon, lat] for lat, lon in all_coords]
    coords_string = ";".join([f"{lon},{lat}" for lon, lat in all_coords_lon_lat])
    url = f"http://router.project-osrm.org/table/v1/driving/{coords_string}?annotations=duration"

    print("Making API call to OSRM...")
    try:
        response = requests.get(url, timeout=60)
        response.raise_for_status()
        time_matrix_raw = response.json()['durations']
        print("API call successful.")
    except Exception as e:
        print(f"FATAL ERROR: Could not get data from OSRM API. {e}")
        return

    # --- Pre-process and Clean the Time Matrix ---
    penalty_value = 999999
    # Replace any None values from the raw matrix
    for i, row in enumerate(time_matrix_raw):
        for j, value in enumerate(row):
            if value is None:
                time_matrix_raw[i][j] = penalty_value

    time_matrix = np.array(time_matrix_raw, dtype=np.int64)

    # ==============================================================================
    # PHASE 2: SOLVING WITH CLARKE & WRIGHT HEURISTIC
    # ==============================================================================
    print("\n--- Phase 2: Solving with Baseline Algorithm (Clarke & Wright) ---")

    depot_idx = 0
    savings = []
    for i in range(1, NUM_ORDERS + 1):
        for j in range(i + 1, NUM_ORDERS + 1):
            saving_value = time_matrix[depot_idx, i] + time_matrix[depot_idx, j] - time_matrix[i, j]
            if saving_value > 0:
                savings.append((saving_value, i, j))
    savings.sort(key=lambda x: x[0], reverse=True)

    routes = {i: [depot_idx, i, depot_idx] for i in range(1, NUM_ORDERS + 1)}

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

    # --- Calculate and Print Final Results ---
    final_routes = list({id(route): route for route in routes.values()}.values())
    total_time = 0
    print("\n--- True Baseline Solution Results ---")
    for i, route in enumerate(final_routes):
        route_time = sum(time_matrix[start, end] for start, end in zip(route, route[1:]))
        print(
            f"Route {i + 1} ({len(route) - 2} stops): {' -> '.join(map(str, route))} | Time: {int(route_time)} seconds")
        total_time += route_time

    print(f"\nTotal Number of Routes: {len(final_routes)}")
    print(f"Total Travel Time: {int(total_time)} seconds ({total_time / 3600:.2f} hours)")


if __name__ == "__main__":
    get_baseline_solution()