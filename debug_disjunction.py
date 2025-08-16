from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def run_toy_problem():
    """A simple, self-contained test for node dropping."""
    print("--- Running Toy Problem to Debug Node Dropping ---")

    # Create the routing index manager for 1 vehicle and 4 locations.
    # Locations: 0=Depot, 1=Good_Cust_1, 2=Good_Cust_2, 3=Bad_Cust
    manager = pywrapcp.RoutingIndexManager(4, 1, 0)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Hardcoded time matrix.
    # Note that all routes to/from node 3 are penalized.
    time_matrix = [
        [0, 150, 180, 999999],  # Depot
        [145, 0, 100, 999999],  # Good_Cust_1
        [175, 95, 0, 999999],  # Good_Cust_2
        [999999, 999999, 999999, 0],  # Bad_Cust (unreachable)
    ]

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # --- The Disjunctions (Allowing nodes to be dropped) ---
    # We set the penalty for dropping a node to be LESS than the penalty
    # for an impossible route.
    drop_penalty = 800000
    for node in range(1, 4):  # For nodes 1, 2, and 3
        routing.AddDisjunction([manager.NodeToIndex(node)], drop_penalty)

    # Setting search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # --- Print the solution ---
    if solution:
        print("\nSolution Found:")
        index = routing.Start(0)
        route = []
        route_time = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(previous_index, index, 0)
        route.append(manager.IndexToNode(index))
        print(f"Route: {' -> '.join(map(str, route))}")
        print(f"Total Time: {route_time}")

        dropped_nodes = 'Dropped nodes:'
        for node in range(1, 4):
            if solution.Value(routing.NextVar(manager.NodeToIndex(node))) == manager.NodeToIndex(node):
                dropped_nodes += f' {node}'
        print(dropped_nodes)
    else:
        print('No solution found!')


# Run the toy problem
run_toy_problem()