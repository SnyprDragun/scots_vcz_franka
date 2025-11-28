import sys
import numpy as np
import pandas as pd
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

    try:
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
        print(f"Error: Need at least 4 points for cubic interpolation, found {num_original_points}.")
        return pd.DataFrame()
    original_time = np.arange(num_original_points)
    new_time = np.linspace(original_time.min(), original_time.max(), output_points)
    interpolated_trajectory = np.zeros((output_points, 3))

    for i, dim_name in enumerate(['x', 'y', 'z']):
        spline_interp = interp1d(original_time, original_data[:, i], kind='cubic', fill_value="extrapolate")
        interpolated_trajectory[:, i] = spline_interp(new_time)
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
        df_original = pd.read_csv(input_filename)
        print(f"Successfully loaded {input_filename} with {len(df_original)} rows.")
        df_reversed = df_original[::-1].copy()
        df_reversed.reset_index(drop=True, inplace=True)
        df_combined = pd.concat([df_original, df_reversed], ignore_index=True)
        print(f"Total rows in combined trajectory: {len(df_combined)}")
        df_combined.to_csv(output_filename, index=False)
        print(f"Successfully saved combined trajectory to {output_filename}.")

    except FileNotFoundError:
        print(f"Error: Input file '{input_filename}' not found.", file=sys.stderr)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)

if __name__ == '__main__':
    INPUT_FILE = "example_1_trajectory.csv"
    OUTPUT_FILE = "example_1_trajectory_inverted.csv"
    invert_and_duplicate_csv(INPUT_FILE, OUTPUT_FILE)

    input_file = "example_1_trajectory_inverted.csv"
    output_file = "example_1_final_trajectory.csv"
    # input_file = "test.csv"
    # output_file = "test.csv"
    desired_points = 10000

    print(f"Starting interpolation from {input_file} to {desired_points} points...")
    high_res_trajectory = interpolate_trajectory(input_file, desired_points)

    if not high_res_trajectory.empty:
        high_res_trajectory.to_csv(output_file, index=False, float_format='%.8f')
        print(f"Interpolation complete.")
        print(f"Original points: {pd.read_csv(input_file).shape[0]}")
        print(f"Interpolated points saved to: {output_file} ({high_res_trajectory.shape[0]})")
        print("\nFirst 5 interpolated steps:")
        print(high_res_trajectory.head())
    else:
        print("Interpolation failed. Check console for error messages.")
