import numpy as np
import os

# This script is designed to load and verify your .npy file.
file_path = 'time_matrix.npy'

print(f"Attempting to load and verify '{file_path}'...")

if os.path.exists(file_path):
    print("SUCCESS: Found the file.")
    try:
        # Use NumPy to "decode" and load the binary data
        loaded_matrix = np.load(file_path)

        # Print its properties in a human-readable format
        print(f"Data type: {loaded_matrix.dtype}")
        print(f"Shape: {loaded_matrix.shape}")

        # Print a sample of the data
        print("\nTop-left 5x5 corner of the matrix:")
        print(loaded_matrix[:5, :5])

    except Exception as e:
        print(f"ERROR: The file was found, but could not be loaded. Error: {e}")
else:
    print(f"FAILURE: The file '{file_path}' does not exist.")