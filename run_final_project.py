import pandas as pd
import numpy as np
import requests
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
import folium
import polyline

# ======================================================================================================================
# THIS FILE IS COMBINATION OF generate_data.py, basline_solver.py, advanced_solver.py
# FOR MORE DETAILED EXPLANATION OF CODE, REFER THE FILES MENTIONED ABOVE
# ======================================================================================================================

def solve_complete_project_with_viz():
    # --- Phase 1 & 2: Prep and Data Creation ---
    print(">>> Starting VRP project (data + pre-processing)...")

    # Depot location (just picking a random point in Bangalore for now)
    DEPOT_LOCATION = [12.93580, 77.62590]

    # Configs (these are pretty arbitrary, I might tune later)
    NUM_CUSTOMERS = 50
    VEH_CAPACITY = 8
    FLEET_SIZE = 12
    DELIVERY_WINDOW_HOURS = 3   # each customer needs to be served in 3 hrs
    MAX_ROUTE_SEC = 10 * 3600   # 10 hours in seconds

    # Generate customer "fake" coordinates near depot
    customer_points = []
    for cid in range(NUM_CUSTOMERS):
        # random radial spread within ~4 km
        dx = (4.0 / 111.1) * np.sqrt(np.random.rand())
        angle = 2 * np.pi * np.random.rand()
        customer_points.append({
            "OriginalOrderID": cid + 1,
            "Latitude": DEPOT_LOCATION[0] + dx * np.cos(angle),
            "Longitude": DEPOT_LOCATION[1] + dx * np.sin(angle)
        })

    ordersTable = pd.DataFrame(customer_points)

    # Combine depot + orders into one big list
    all_nodes = [DEPOT_LOCATION] + list(zip(ordersTable["Latitude"], ordersTable["Longitude"]))

    # --- Step: Fetch OSRM matrix ---
    try:
        # NOTE: OSRM expects lon,lat not lat,lon
        node_pairs = [[lon, lat] for lat, lon in all_nodes]
        coords_str = ";".join([f"{lon},{lat}" for lon, lat in node_pairs])

        req_url = f"http://router.project-osrm.org/table/v1/driving/{coords_str}?annotations=duration"
        resp = requests.get(req_url, timeout=60)
        resp.raise_for_status()

        raw_matrix = resp.json()["durations"]

    except Exception as oops:
        print(f"!!! Big failure while hitting OSRM: {oops}")
        return

    # --- Cleaning OSRM data ---
    # some nodes might be unreachable → handle them gracefully
    penalty_time = 999999
    ok_indices = [i for i, row in enumerate(raw_matrix) if raw_matrix[i][0] is not None and raw_matrix[0][i] is not None]
    bad_nodes = [i for i in range(NUM_CUSTOMERS + 1) if i not in ok_indices and i != 0]

    if bad_nodes:
        print(f"Warning: found {len(bad_nodes)} unreachable customer(s) → {bad_nodes}")

    usable_matrix = np.array(raw_matrix)[np.ix_(ok_indices, ok_indices)]
    usable_matrix[usable_matrix == None] = penalty_time   # clunky but works
    usable_time_matrix = usable_matrix.astype(np.int64)

    usable_orders = ordersTable[ordersTable["OriginalOrderID"].isin(ok_indices)]

    # --- OR-Tools Setup ---
    print("\n>>> Phase 3: Kicking off OR-Tools solver <<<")

    num_nodes = len(usable_time_matrix)
    mgr = pywrapcp.RoutingIndexManager(num_nodes, FLEET_SIZE, 0)
    router = pywrapcp.RoutingModel(mgr)

    # travel time callback
    def travel_time_cb(from_i, to_i):
        return usable_time_matrix[mgr.IndexToNode(from_i)][mgr.IndexToNode(to_i)]

    transit_cb = router.RegisterTransitCallback(travel_time_cb)
    router.SetArcCostEvaluatorOfAllVehicles(transit_cb)

    # add time dimension
    router.AddDimension(transit_cb, 0, MAX_ROUTE_SEC, True, "Time")
    time_dim = router.GetDimensionOrDie("Time")

    # capacity stuff
    load = [1] * num_nodes
    load[0] = 0  # depot has no demand

    def load_cb(from_i):
        return load[mgr.IndexToNode(from_i)]

    load_cb_id = router.RegisterUnaryTransitCallback(load_cb)
    router.AddDimensionWithVehicleCapacity(load_cb_id, 0, [VEH_CAPACITY] * FLEET_SIZE, True, "Capacity")

    # Add time windows (pretty strict for customers, depot gets full horizon)
    win = [(0, int(DELIVERY_WINDOW_HOURS * 3600))] * num_nodes
    win[0] = (0, MAX_ROUTE_SEC)

    for k in range(num_nodes):
        time_dim.CumulVar(mgr.NodeToIndex(k)).SetRange(win[k][0], win[k][1])

    # Force solver to consider minimizing start times too (helps tighten windows)
    for v in range(FLEET_SIZE):
        router.AddVariableMinimizedByFinalizer(time_dim.CumulVar(router.Start(v)))

    # search params
    opts = pywrapcp.DefaultRoutingSearchParameters()
    opts.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    opts.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    opts.time_limit.FromSeconds(30)

    sol = router.SolveWithParameters(opts)

    if sol:
        routes = print_solution(sol, mgr, router, usable_time_matrix, ok_indices)
        create_solution_map(routes, usable_orders, DEPOT_LOCATION)
    else:
        print("\n!!! No solution found :( !!!")


# ------------------------------------------------------------------------------
# Map builder (Folium + OSRM route API)
# ------------------------------------------------------------------------------
def create_solution_map(routes, ordersTable, depot_coords):
    print("\n>>> Drawing solution on a folium map...")

    mymap = folium.Map(location=depot_coords, zoom_start=14)
    folium.Marker(depot_coords, popup="Depot (0)", icon=folium.Icon(color="red", icon="home")).add_to(mymap)

    # cycling through colors... eventually should expand this if fleet is bigger
    palette = ["blue", "green", "purple", "orange", "darkred", "lightred",
               "beige", "darkblue", "darkgreen", "cadetblue", "pink", "lightblue"]

    # map order IDs to coords for easy lookup
    coord_lookup = ordersTable.set_index("OriginalOrderID").to_dict("index")
    coord_lookup[0] = {"Latitude": depot_coords[0], "Longitude": depot_coords[1]}

    for v, (vid, stops) in enumerate(routes.items()):
        col = palette[v % len(palette)]
        path_points = []

        # get each leg geometry
        for i in range(len(stops) - 1):
            a, b = stops[i], stops[i + 1]
            start = coord_lookup[a]
            end = coord_lookup[b]

            # small hack: OSRM wants lon,lat
            url = f"http://router.project-osrm.org/route/v1/driving/{start['Longitude']},{start['Latitude']};{end['Longitude']},{end['Latitude']}?overview=full&geometries=polyline"
            r = requests.get(url)
            geom = r.json()["routes"][0]["geometry"]

            path_points.extend(polyline.decode(geom))

        folium.PolyLine(path_points, color=col, weight=3, opacity=0.8,
                        popup=f"Vehicle {vid}").add_to(mymap)

        # plot customers as circles
        for cust in stops[1:-1]:
            cc = [coord_lookup[cust]["Latitude"], coord_lookup[cust]["Longitude"]]
            folium.CircleMarker(cc, radius=5, color=col, fill=True,
                                popup=f"Customer {cust}<br><b>Vehicle {vid}</b>").add_to(mymap)

    fname = "optimized_routes_map.html"
    mymap.save(fname)
    print(f">>> Map written to {fname}")


# ------------------------------------------------------------------------------
# Just printing the solver solution in a readable way
# ------------------------------------------------------------------------------
def print_solution(sol, mgr, router, time_matrix, ok_indices):
    print("\n>>> Optimized Solution:")

    total_time, active_routes = 0, 0
    final_routes = {}
    index_lookup = np.array(ok_indices)

    for v in range(router.vehicles()):
        idx = router.Start(v)
        if router.IsEnd(sol.Value(router.NextVar(idx))):
            continue  # skip unused vehicle

        active_routes += 1
        path, rtime = [], 0

        while not router.IsEnd(idx):
            nid = mgr.IndexToNode(idx)
            path.append(int(index_lookup[nid]))
            prev, idx = idx, sol.Value(router.NextVar(idx))
            rtime += router.GetArcCostForVehicle(prev, idx, v)

        path.append(0)
        final_routes[v] = path
        print(f"Veh {v}: {' -> '.join(map(str, path))} | Time {rtime}s")

        total_time += rtime

    print(f"\nRoutes used: {active_routes}")
    print(f"Total time: {total_time}s ({total_time/3600:.2f} hrs)")
    if active_routes > 0:
        avg_t = total_time / active_routes
        print(f"Avg route time: {int(avg_t)}s ({avg_t/60:.2f} min)")
    return final_routes


if __name__ == "__main__":
    solve_complete_project_with_viz()
