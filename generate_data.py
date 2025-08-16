import pandas as pd
import numpy as np
import requests
import json

# Constants
Depot_Name = "Zepto Kormangala 5th block"
# Coordinats of zepto using google maps: 12.937771 N, 77.618625 E
Depot_Coordinates = [12.937771, 77.618625]
Num_Orders_Max = 50
Delivery_Radius_Km = 4.0

# Conversion factor: 1 degreee of latitude is approximately 111.1 km
# Reason for conversion is to determine the actual distane of the earth's
# surface represented by a specific latitude angle
Radius_In_Degrees = Delivery_Radius_Km / 111.1

print(f"Depot set to '{Depot_Name}' at {Depot_Coordinates}.")
print(f"Generating {Num_Orders_Max} orders within a {Delivery_Radius_Km} km radius")

# Generate Customer Locations
order_locations = []
# A loop to generate Num_Orders_Max random points
for i in range(Num_Orders_Max):
    #Generate  a random radius and angle
    # we use np.sqrt(np.random.rand()) to ensure a uniform distribution in the circle
    # np.random.rand(): Generates a uniform random number between 0 and 1
    # np.sqrt(): Takes the square root to convert from uniform area sampling to
    # uniform radius sampling. Without this points would cluster near the center
    # (since area ∝ radius²).
    r = Radius_In_Degrees * np.sqrt(np.random.rand())

    # Polar Coordinates: Typically used with a random angle
    # theta = 2pi * np.random.rand() [classical formula = 2pi * r]
    theta = 2 * np.pi * np.random.rand() #

    # Convert Polar Coordinats to Cartesian Offsets
    # lat_cor = x, lon_cor = y, cartesian formula = r * theta
    # we use cos and sin, since there are two coordiantes x & y
    lat_offset = r * np.cos(theta)
    lon_offset = r * np.sin(theta)

    # Apply offsets to depot coordinates to get customer location
    customer_lat = Depot_Coordinates[0] + lat_offset
    customer_lon = Depot_Coordinates[0] + lon_offset

    order_locations.append({'OrderID': i + 1, 'Latitude': customer_lat, 'Longitude': customer_lon})

# Create a Panda DataFrame
orders_df = pd.DataFrame(order_locations)

# Save to CSV
orders_df.to_csv('orders.csv', index=False)

print(f"Successfully generated and saved {len(orders_df)} orders to 'orders.csv'.")

# Building Travel Time Matrix using OSRM API
# First, Create a list of all coordinates: Depot is at index 0
all_coords = [Depot_Coordinates] + list(zip(orders_df['Latitude'], orders_df['Longitude']))

# OSRM requires coordinates in longitude, latitude format
all_coords_lon_lat = [[lon, lat] for lat, lon in all_coords]

# Format coordinates for the API call URL
coords_string = ";".join([f"{lon},{lat}" for lon, lat in all_coords_lon_lat])

# Build the OSRM API request URL
url = f"http://router.project-osrm.org/table/v1/driving/{coords_string}?annotations=duration"
print("Making API call to OSRM... This may take a moment")

# --- Step 4 (REVISED & ROBUST): Save and Verify the Time Matrix ---

try:
    # Make the API call
    response = requests.get(url)
    response.raise_for_status()
    print("API call successful.")

    data = response.json()

    if 'durations' in data:
        durations = data['durations']

        # --- DATA CLEANING STEP ---
        # Iterate through the matrix and replace any 'None' values with a large penalty number.
        print("Cleaning the data: Checking for impossible routes...")
        penalty_value = 999999  # A very large number representing an impossible route
        none_count = 0
        for i, row in enumerate(durations):
            for j, value in enumerate(row):
                if value is None:
                    durations[i][j] = penalty_value
                    none_count += 1

        if none_count > 0:
            print(f"WARNING: Found and replaced {none_count} impossible routes with a penalty value.")
        else:
            print("Data is clean. No impossible routes found.")

        # Create and save the matrix
        time_matrix = np.array(durations, dtype=np.int64)  # Use the cleaned 'durations' list
        file_path = 'time_matrix.npy'
        np.save(file_path, time_matrix)
        print(f"--- SUCCESS: Data saved to '{file_path}' ---")

        # --- VERIFICATION STEP ---
        print("\n--- Starting Verification ---")
        try:
            loaded_matrix = np.load(file_path)
            print("VERIFICATION SUCCESSFUL: File loaded correctly.")
            print(f"Shape of loaded matrix: {loaded_matrix.shape}")
            print(f"Data type of loaded matrix: {loaded_matrix.dtype}")
            print("First 5x5 corner of the matrix:")
            print(loaded_matrix[:5, :5])
            print("--- Verification Complete ---")

        except Exception as e:
            print(f"--- VERIFICATION FAILED: The file was saved, but could not be re-loaded. Error: {e} ---")

    else:
        print("--- ERROR: 'durations' not found in API response. ---")

except Exception as e:
    print(f"--- FATAL ERROR during API request or file processing: {e} ---")