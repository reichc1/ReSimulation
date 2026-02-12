import os
import shutil
import pandas as pd
from GenerateTrajectoryCatalogAndScenario import GenerateScenario
from ProcessVehicleDataframes import process_all_vehicles
from RunSimulationInEsmini import run_esmini, run_esmini_without_saving_video

# Configuration Parameters
username = "oguisse"
PATH_ESMINI = R"C:\Users\OGUISSE\esmini-demo\bin\esmini.exe" # Path to esmini executable
road_network = r"C:\Users\OGUISSE\esmini-demo\2017-04-04_Testfeld_A9_Sued.xodr" # local path to the road network file 
vehicle_catalog_path = r"C:\Users\OGUISSE\esmini-demo\resources\xosc\Catalogs\Vehicles" # Path to the vehicle catalog

# List of directories containing ReSimulation data
drive_list = [r"C:\Users\OGUISSE\Desktop\test_modul"] # local path to the "ReSimulation_dataframe.parquet" folder

# Simulation control flags
RUN_ESMINI = True # Set to True to run the simulation using esmini.
SAVE_SIMULATION_VIDEO = False # Set to True if you also want to save a video recording of the simulation.


if __name__ == "__main__":

    # Iterate through each drive file folder
    for drive_file in drive_list:
    
        # Step 1: Process Re-Simulation input data for each vehicle in the drive and save data in a temporary folder
        print("Creating ReSimulation input dataframe for each vehicle in drive file:", drive_file)
        resimulation_dataframe_path = os.path.join(drive_file, "ReSimulation_dataframe.parquet")
        temp_folder = os.path.join(drive_file, "ReSimulation_Data_Buffer")
        os.makedirs(temp_folder, exist_ok=True)
        input_df = pd.read_parquet(resimulation_dataframe_path)
        process_all_vehicles(input_df, temp_folder)

        # Step 2: Generate Scenario and catalog openscenario-files  
        print("Performing ReSimulation for the file:", drive_file)
        generate_scenario = GenerateScenario()
        generate_scenario.process_file(temp_folder, road_network, username, vehicle_catalog_path, drive_file)
        
        # Step 3: Remove temporary folder after scenario generation
        temp_folder = os.path.join(drive_file, "ReSimulation_Data_Buffer")
        if os.path.exists(temp_folder):
            shutil.rmtree(temp_folder)
            print(f"Deleted temporary folder: {temp_folder}")

        print("Finished processing the file: ", drive_file)

        # Step 4: Run the scenario simulation using esmini
        if RUN_ESMINI and SAVE_SIMULATION_VIDEO:
            print("Running simulation in esmini and saving simulation-video for the file:", drive_file)
            run_esmini(RUN_ESMINI, PATH_ESMINI, drive_file) # Run esmini and save video 

        if RUN_ESMINI and not SAVE_SIMULATION_VIDEO:
            print("Running simulation in esmini for the file:", drive_file)
            run_esmini_without_saving_video(RUN_ESMINI, PATH_ESMINI, drive_file) # Run esmini without saving video