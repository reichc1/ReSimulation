import numpy as np
import pandas as pd
from numpy.polynomial.legendre import legval
import os

# Constants
FREQUENCY = 10  # in Hz
LATERAL_ACTIVITIES = ["follow", "evasion", "right", "left", "IncompleteStartLeft", "IncompleteStartRight", "IncompleteEndLeft", "IncompleteEndRight"]
LONGITUDINAL_ACTIVITIES = ["cruise", "brake", "accelerate"]

def generate_timestamps(start, end, freq=FREQUENCY):
    """
    Generate timestamps between start and end at the given frequency.
    Ensures the exact start and end values are included.
    """
    timestamps = np.arange(start, end, 1 / freq)
    if timestamps[-1] < end:
        timestamps = np.append(timestamps, end)
    timestamps[0] = start
    timestamps[-1] = end
    return timestamps

def process_vehicle(vehicle_df, vehicle_id, resimulation_dataframe_path_input_dir):
    all_rows = []

    # Separate lateral and longitudinal activities 
    lateral_activities = vehicle_df[vehicle_df["activity_type"].isin(LATERAL_ACTIVITIES)]
    longitudinal_activities = vehicle_df[vehicle_df["activity_type"].isin(LONGITUDINAL_ACTIVITIES)]

    # Process each lateral activity (lane changes)
    for _, activity in lateral_activities.iterrows():
        start_time = activity["activity_starttime"]
        end_time = activity["activity_endtime"]
        timestamps = generate_timestamps(start_time, end_time)
        
        # Round timestamps to match precision from sampling frequency
        decimal_places = max(0, int(np.ceil(-np.log10(1 / FREQUENCY))))
        timestamps = np.round(timestamps, decimal_places)
        
        # Parse activity parameters (stored as string or array-like)
        params = activity["activity_parameters"]
        if isinstance(params, str):
            params = [float(x) for x in params.split(",")]
        elif isinstance(params, (float, int, np.number)) or np.array(params).ndim == 0:
            params = [float(params)]
        else:
            params = [float(p) for p in params]

        activity_type = activity["activity_type"]

        # Handle lane change activities, which split into two trajectory parts
        if activity_type in ["right", "left", "IncompleteStartLeft", "IncompleteStartRight", "IncompleteEndLeft", "IncompleteEndRight"]:
            lane_change_time = params[0] + start_time
            params_first = params[1:8]
            params_second = params[8:15]

            ts_first = timestamps[timestamps <= lane_change_time]
            ts_second = timestamps[timestamps >= lane_change_time]

            # Evaluate t_values for first half using Legendre polynomial
            if len(ts_first) > 1 and ts_first[-1] != ts_first[0]:
                scaled_ts_first = 2 * (ts_first - ts_first[0]) / (ts_first[-1] - ts_first[0]) - 1
                t_values_first = legval(scaled_ts_first, params_first)
            else:
                t_values_first = [None] * len(ts_first)

            # Evaluate t_values for second half using Legendre polynomial
            if len(ts_second) > 1 and ts_second[-1] != ts_second[0]:
                scaled_ts_second = 2 * (ts_second - ts_second[0]) / (ts_second[-1] - ts_second[0]) - 1
                t_values_second = legval(scaled_ts_second, params_second)
            else:
                t_values_second = [None] * len(ts_second)

            # Determine new lane ID after lane change
            original_lane_id = activity["lane_id"]
            if activity_type in ["right", "IncompleteStartRight", "IncompleteEndRight"]:
                new_lane_id = original_lane_id + 1
            elif activity_type in ["left", "IncompleteStartLeft", "IncompleteEndLeft"]:
                new_lane_id = original_lane_id - 1

            # Collect rows for first half (before lane change)
            for i, ts in enumerate(ts_first):
                all_rows.append({
                    "timestamp": ts,
                    "velocity": None,
                    "t_coordinate": t_values_first[i],
                    "lane_id": original_lane_id,
                    "initial_GPS_position": activity["initial_GPS_position"] if i == 0 else None
                })

            # Collect rows for second half (after lane change)
            for i, ts in enumerate(ts_second):
                all_rows.append({
                    "timestamp": ts,
                    "velocity": None,
                    "t_coordinate": t_values_second[i],
                    "lane_id": new_lane_id,
                    "initial_GPS_position": activity["initial_GPS_position"]
                })

        # Handle follow/evasion activities, which use a single polynomial
        elif activity_type in ["follow", "evasion"]:
            leg_params = params[:7]
            scaled_ts = 2 * (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0]) - 1
            t_values = legval(scaled_ts, leg_params)
            for i, ts in enumerate(timestamps):
                all_rows.append({
                    "timestamp": ts,
                    "velocity": None,
                    "t_coordinate": t_values[i],
                    "lane_id": activity["lane_id"],
                    "initial_GPS_position": activity["initial_GPS_position"] if i == 0 else None
                })        

    # Process each longitudinal activity
    for _, activity in longitudinal_activities.iterrows():
        timestamps = generate_timestamps(activity["activity_starttime"], activity["activity_endtime"])
        decimal_places = max(0, int(np.ceil(-np.log10(1 / FREQUENCY))))
        timestamps = np.round(timestamps, decimal_places)
        
        # Parse longitudinal activity parameters
        params = activity["activity_parameters"]
        if isinstance(params, str):
            params = [float(x) for x in params.split(",")]
        elif isinstance(params, (float, int, np.number)) or np.array(params).ndim == 0:
            params = [float(params)]
        else:
            params = [float(p) for p in params]

        # Evaluate velocity values using Legendre polynomial
        scaled_ts = 2 * (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0]) - 1
        v_values = legval(scaled_ts, params)
        
        # Append velocity data rows
        for i, ts in enumerate(timestamps):
            all_rows.append({
                "timestamp": ts,
                "velocity": v_values[i],
                "t_coordinate": None,
                "lane_id": None,
                "initial_GPS_position": activity["initial_GPS_position"] if i == 0 else None
            })

    # Convert to DataFrame
    df = pd.DataFrame(all_rows)
    # Sort by timestamp before filling
    df = df.sort_values(by="timestamp").reset_index(drop=True)
    # Merge duplicate timestamps (both lat + long)
    df = df.groupby("timestamp").first().reset_index()

    # Fill missing t_coordinate forward only for lateral activities
    df["t_coordinate"] = df["t_coordinate"].ffill()
    # Forward-fill lane_id and velocity if necessary
    df["lane_id"] = df["lane_id"].ffill()
    df["velocity"] = df["velocity"].interpolate()

    # Compute delta_s
    dt = 1 / FREQUENCY # seconds
    df["delta_s"] = df["velocity"] / 3.6 * dt

    # Set initial_GPS_position only at the first timestamp row
    first_idx = df["timestamp"].idxmin()
    df.loc[:, "initial_GPS_position"] = None
    df.at[first_idx, "initial_GPS_position"] = next(
        (row["initial_GPS_position"] for row in all_rows if row["initial_GPS_position"] is not None),
        None
    )

    # Save to parquet in a temporary folder
    output_path = f"{vehicle_id}_ReSimulationInput.parquet"
    save_path = os.path.join(resimulation_dataframe_path_input_dir, output_path)
    df.to_parquet(save_path, index=False)
    print(f"Saved: {save_path}")
    

# Main function to process all vehicles
def process_all_vehicles(input_df, resimulation_dataframe_path_input_dir):
    for vehicle_id in input_df["vehicle_id"].unique():
        vehicle_df = input_df[input_df["vehicle_id"] == vehicle_id].copy()
        process_vehicle(vehicle_df, vehicle_id, resimulation_dataframe_path_input_dir)