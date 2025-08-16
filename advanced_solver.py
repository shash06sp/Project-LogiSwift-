''' Building a Advanced Optimization Model Using Capacitated Vehicle Routing
Problem (CVRP) which is more efficient and less complexity than
Clark Wilson Model, which we have done previously in run_project file'''

import numpy as np
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

print("--- Starting Advanced Solver (Using Google OR - Tools) ---")

# Load Data
try:
    time_matrix = np.load('time_matrix.npy')
    orders_df = pd.read_csv('orders.csv')
    print("Time Matrix and CSV files loaded Successfully. ")
except FileNotFoundError:
    print("ERROR: Time Matrix and CSV files not found. Please run the data generation script first. ")
    exit()

# Constraints
Num_Vehicles = 12
Depot_Idx = 0
Vehicle_Capacity = 5

# Creating the data model for the solver i.e a class that the solver can easily use

def create_data_model():
    # Stores the data for the problem.
    data = {}
    data['time_matrix'] = time_matrix.tolist() # OR Tools prefers lists
    data['num_vehicles'] = Num_Vehicles
    data['depot'] = Depot_Idx

    # For CVRP, we need demands for each location
    # Demand is 1 for each customer, 0 for the depot
    # Refer Algorithm file for detailed explanation of CVRP
    data['demands'] = [0] + [1] * len(orders_df)
    data['vehicle_capacities'] =  [Vehicle_Capacity] * Num_Vehicles
    return data

data = create_data_model()

# Create the routing model
# Here we initialize the main components of the OR Tools Solver

# Create the routing index manager
manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                       data['num_vehicles'], data['depot'])

# Create Routing Model
routing = pywrapcp.RoutingModel(manager)

# Define CallBacks and Add Constraints
# ---- Time CallBack ----
def time_callback(from_index, to_index):
    # Returns the travel time between two nodes
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['time_matrix'][from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(time_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# ---- Demand CallBack & Capacity Constraint ----

def demand_callback(from_index):
    # Returns the demand of a node
    from_node = manager.IndexToNode(from_index)
    return data['demands'][from_node]

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0, # nul capacity slack
    data['vehicle_capacities'], # Vehicle Maximum Capacities
    True, # Start cumul to zero
    'Capacity')


# --- Demand Callback & Capacity Constraint ---
# ... (your existing capacity code) ...

# --- Allow Nodes to be Dropped (Penalties) ---
# Set a large penalty for dropping a node. This value should be less than
# our impossible route penalty (999999) so the solver prefers to drop an
# unreachable node rather than use a penalized route.
# --- Allow Nodes to be Dropped (Penalties) ---
# MAKE SURE THIS CODE IS ACTIVE (NOT COMMENTED OUT)
penalty = 800000
for node in range(1, len(data['time_matrix'])):
    if manager.NodeToIndex(node) != -1:
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)
# --- Set Search Parameters and Solve ---

# Set Search Parameters and Solve
# Selecting firest solution heuristic
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
search_parameters.time_limit.FromSeconds(30)

print("Solving the CVRP...")
solution = routing.SolveWithParameters(search_parameters)

# Print the Solution
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"\n--- Advanced OR-Tools Solution ---")
    total_time = 0
    total_routes = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        route_time = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += f' {node_index} ->'
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        node_index = manager.IndexToNode(index)
        plan_output += f' {node_index}\n'

        if route_time > 0:  # Only print used routes
            total_routes += 1
            plan_output += f'Time of the route: {route_time}s\n'
            plan_output += f'Load of the route: {route_load}\n'
            print(plan_output)
            total_time += route_time

    print(f"Total Number of Routes: {total_routes}")
    print(f'Total Travel Time: {total_time}s ({total_time / 3600:.2f} hours)')

# Replace the old block with THIS new block:
if solution:
    print_solution(data, manager, routing, solution)
else:
    # This new 'else' part prints a much clearer failure message
    print("\n" + "="*50)
    print(">>> FAILURE: NO SOLUTION FOUND <<<")
    print("This likely means the problem is over-constrained.")
    print("="*50)


    ''' 
     else:
        print("No solution found!")'''