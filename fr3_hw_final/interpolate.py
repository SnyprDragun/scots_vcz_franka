import pandas as pd
import sys
import numpy as np
from scipy.interpolate import interp1d

def interpolate_trajectory(input_filename: str, output_points: int) -> pd.DataFrame:
    """
    Loads a 3D trajectory from a CSV file (x, y, z columns) and interpolates it
    to a specified number of output points using cubic spline interpolation.

    Args:
        input_filename (str): The path to the input CSV file.
        output_points (int): The desired number of points in the output trajectory.

    Returns:
        pd.DataFrame: A new DataFrame with the interpolated x, y, z trajectory.
    """
    # 1. Load the original data
    try:
        # Assuming the CSV file has headers 'x', 'y', 'z'
        df = pd.read_csv(input_filename)
        original_data = df[['x', 'y', 'z']].values
    except FileNotFoundError:
        print(f"Error: Input file '{input_filename}' not found.")
        return pd.DataFrame()
    except KeyError:
        print("Error: CSV must contain 'x', 'y', and 'z' columns.")
        return pd.DataFrame()

    num_original_points = original_data.shape[0]
    
    if num_original_points < 4:
        # Cubic interpolation requires at least 4 points.
        print(f"Error: Need at least 4 points for cubic interpolation, found {num_original_points}.")
        return pd.DataFrame()
    
    # 2. Define the 'time' or index axis for the original points
    # This assumes the points are uniformly sampled in time/sequence space.
    original_time = np.arange(num_original_points)

    # 3. Define the new, densely sampled time axis
    # The new points span the exact same duration/sequence as the original.
    new_time = np.linspace(original_time.min(), original_time.max(), output_points)

    # Prepare array for interpolated trajectory
    interpolated_trajectory = np.zeros((output_points, 3))

    # 4. Perform interpolation for each dimension (x, y, z) independently
    for i, dim_name in enumerate(['x', 'y', 'z']):
        # Create the cubic spline interpolator object
        # kind='cubic' provides C2 continuity (smoother motion).
        # fill_value='extrapolate' allows calculation slightly outside the range if needed.
        spline_interp = interp1d(original_time, original_data[:, i], kind='cubic', fill_value="extrapolate")
        
        # Calculate the interpolated values on the new time axis
        interpolated_trajectory[:, i] = spline_interp(new_time)

    # 5. Convert the result back to a DataFrame
    df_interpolated = pd.DataFrame(interpolated_trajectory, columns=['x', 'y', 'z'])
    
    return df_interpolated



def invert_and_duplicate_csv(input_filename: str, output_filename: str):
    """
    Reads a CSV file, reverses the order of the data rows, and then
    concatenates the original data with the reversed data.

    The final output file will have: [Header] + [Original Rows] + [Reversed Rows].

    Args:
        input_filename (str): The path to the input CSV file (e.g., 'simulated_trajectory.csv').
        output_filename (str): The path where the new, combined CSV will be saved.
    """
    try:
        # 1. Read the original CSV file into a DataFrame
        df_original = pd.read_csv(input_filename)
        print(f"Successfully loaded {input_filename} with {len(df_original)} rows.")

        # 2. Invert the row order to create the return path
        # [::-1] is a standard Python slicing technique to reverse the order of the rows.
        # We use .copy() to ensure it's a new, independent DataFrame.
        df_reversed = df_original[::-1].copy()
        
        # Optional: Reset the index of the reversed DataFrame for clean output
        df_reversed.reset_index(drop=True, inplace=True)

        # 3. Concatenate the original and the reversed DataFrame
        # The result is the complete out-and-back trajectory.
        df_combined = pd.concat([df_original, df_reversed], ignore_index=True)
        print(f"Total rows in combined trajectory: {len(df_combined)}")

        # 4. Save the combined DataFrame to the new CSV file
        # index=False ensures we don't write the DataFrame index as a column.
        df_combined.to_csv(output_filename, index=False)
        print(f"Successfully saved combined trajectory to {output_filename}.")

    except FileNotFoundError:
        print(f"Error: Input file '{input_filename}' not found.", file=sys.stderr)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)

# --- Example Usage ---
if __name__ == '__main__':
    # Define the file names
    # Assuming 'simulated_trajectory.csv' is the file you want to use
    # INPUT_FILE = "simulated_trajectory.csv"
    # OUTPUT_FILE = "inverted_trajectory_full.csv"

    # # Execute the function
    # invert_and_duplicate_csv(INPUT_FILE, OUTPUT_FILE)

    # Define file names and the desired number of points
    input_file = "simulated_trajectory.csv"
    output_file = "first_20000.csv"
    desired_points = 20000

    print(f"Starting interpolation from {input_file} to {desired_points} points...")

    # Run the interpolation function
    high_res_trajectory = interpolate_trajectory(input_file, desired_points)

    if not high_res_trajectory.empty:
        # Save the result
        high_res_trajectory.to_csv(output_file, index=False, float_format='%.8f')
        
        print(f"Interpolation complete.")
        print(f"Original points: {pd.read_csv(input_file).shape[0]}")
        print(f"Interpolated points saved to: {output_file} ({high_res_trajectory.shape[0]})")
        
        # Display the first few rows of the new, dense trajectory
        print("\nFirst 5 interpolated steps:")
        print(high_res_trajectory.head())
    else:
        print("Interpolation failed. Check console for error messages.")
        