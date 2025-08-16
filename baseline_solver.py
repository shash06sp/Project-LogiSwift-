# Implementing The Baseline Heuristic
'''
Today's objective is to implement the Clarke and Wright Savings Algorithm.
This is a classic and highly respected heuristic for solving the Vehicle Routing Problem.
By the end of the day, you will have a script that generates a complete, feasible
set of delivery routes and calculates the total time required.
'''

# Conceputual Overview: Clarke & Wright Savings
''' 
Think of the algorithm like this:

1. Worst Case Scenario: Imagine you send out a separate delivery rider for 
every single one of your 50 customers. Each trip looks like Depot -> Customer i -> Depot. 
This is valid, but incredibly inefficient.

2. Finding "Savings": The algorithm cleverly asks:
"How much time can I save if I link two customer trips together?" For any two customers, 
i and j, the saving is calculated as:

Saving(i, j) = (Time from Depot to i) + (Time from Depot to j) - (Time from i to j)
This forumula is inspired from trignometry for finding hypotenuse of a right angle trianle
The path follow of this forumula: Depot -> customer i -> customer j -> Depot

This saving represents the travel time you eliminate by not having to return to the 
depot between serving customers i and j.

3. Greedy Merging: The algorithm calculates this saving for every possible pair of customers.
It then sorts this list from the highest saving to the lowest. It greedily iterates down 
the list, merging routes together as long as the merge doesn't violate the vehicle's capacity.
'''

import numpy as np
import pandas as pd

# Load Data
time_matrix = np.load('time_matrix.npy')
order_df = pd.read_csv('orders.csv')
num_orders = len(order_df)
depot_idx = 0

# Constranits
Vehicle_Capacity = 5

# Create a list to hold the savings for every pair of customers (i,j)
# Remember thet customer indices in our matrix rum from 1 to 50

savings = []
# Iterate over all unique pairs of customers
for i in range(1, num_orders + 1):
    for j in range(i + 1, num_orders + 1):
        # C(0,i) + C(0,j) - C(i,j)
        savings_value = time_matrix[depot_idx, i] + time_matrix[depot_idx, j] - time_matrix[i,j]
        savings.append((savings_value, i, j))

# Sort the savings list in descending order
savings.sort(key=lambda x: x[0], reverse=True)

# Worst Case: Each route is a list of none indices, eg: for Depot -> C1 -> Depot
routes = {i: [depot_idx, i, depot_idx] for i in range(1, num_orders + 1)}

# The Main Merging Loop
# This is the core logic. Iterate through your sorted savings list and try to merge routes.
# A merge is only valid if the customers i and j are at the ends of their current routes and
# the combined load does not exceed capacity.

def find_route(customer_idx, all_routes):
    for route_list in all_routes.values():
        if customer_idx in route_list:
            return route_list
        return None

for saving, i, j in savings:
    route_i = find_route(i, routes)
    route_j = find_route(j, routes)

    # Proceed only if i and j are in differrent routes
    if route_i is not None and route_j is not None and route_i is not route_j:
        # Calculate Load of each route (number of custoners)
        load_i = len(route_i) - 2
        load_j = len(route_j) - 2
        if load_i + load_j <= Vehicle_Capacity:
            # Check the 4 merge cases

            # Case 1: End route_i connects to Start of route_j (i -> j)
            # Ex: i=C12, j=C3, Route A: D -> C5 -> C12 -> D, Route B: D -> C3 -> C8 -> D
            # Result: D -> C5 -> C12 -> C3 -> C8 -> D
            if route_i[2] == i and route_j[1] == j:
                new_route = route_i[: -1] + route_j[1:]

            # Case 2: Start of route_i connects to Start of route_j (reverse i, then i -> j)
            # Ex: i = C5, j=C3, Route A: D -> C5 -> C12 -> D, Route B: D -> C3 -> C8 -> D
            # Result: D -> C12 -> C5 -> C3 -> C8 -> D
            elif route_i[1] == i and route_j[1] == j:
                route_i.reverse() # Reverse route_i in place
                new_route = route_i[:-1] + route_j[1:] # Reverses postion of i from 1 to -1 and j remains same

            # Case 3: End of route_i connects to End of route_j (reverse j, then i- > j)
            # Ex: i=C12, j=C8, Route A: D -> C5 -> C12 -> D, Route B: D -> C3 -> C8 -> D
            # Result: D -> C5 -> C12 -> C8 -> C3 -> D
            elif route_i[-2] == i and route_j[-2] ==j:
                route_j.reverse() # Reverse route_j in place
                new_route = route_j[:-1] + route_i[1:]

            # Case 4: Start of route_i connects to End of route_j (j -> i)
            # Ex: i=C5, j=C8, Route A: D -> C5 -> C12, Route B: D -> C3 -> C8 -> D
            # Result: D -> C3 -> C8 -> C5 -> C12 -. D
            elif route_i[1] == i and route_j[-2] == j:
                # This route is equivalent to connecting j to i
                new_route = route_j[:-1] + route_i[1:]

            else:
                # No merge possible for this pair (i or j is not at an endpoint)
                new_route = None

            # If a valid merge was found, update the routes dictionary
            if new_route:
                for customer in new_route:
                    if customer != depot_idx:
                        routes[customer] = new_route

    final_routes = list({id(route): route for route in routes.values()}.values())

    total_time = 0
    print("\n--- Baseline Solution (Clarke & Wright Savings) ---")
    for i, route in enumerate(final_routes):
        route_time = 0
        for j in range(len(route) - 1):
            start_node = route[j]
            end_node = route[j + 1]
            route_time += time_matrix[start_node, end_node]

        print(f"Route {i + 1}: {' -> '.join(map(str, route))} | Time: {route_time} seconds")
        total_time += route_time

    print(f"\nTotal Number of Routes: {len(final_routes)}")
    print(f"Total Travel Time: {total_time} seconds ({total_time / 3600:.2f} hours)")