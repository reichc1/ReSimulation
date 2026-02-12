# ReSimulation Guide

This project enables you to resimulate a driving scenario locally using the `ReSimulation.py` script.

## 🛠️ Requirements

Before running the script, make sure you install and configure the following:

### 1. esmini (Simulation Engine)

- Download esmini-demo from: [esmini v2.48.1](https://github.com/esmini/esmini/releases/tag/v2.48.1)
- Add the path to `esmini.exe` to your **Environment Variables → Path**  
  Example path:  
  `C:\Users\YourName\esmini-demo\bin`

### 2. FFmpeg (Optional - for video recording)

- Download from: [FFmpeg](https://www.videohelp.com/software/ffmpeg)
- Extract the `.7z` file
- Add the folder containing `ffmpeg.exe` to your **Environment Variables → Path**  
  Example path:  
  `C:\Users\YourName\Downloads\ffmpeg\bin`
  
Note: If you encounter a configuration error (e.g., "Path not found"), it likely requires restarting your programming software (e.g., VS Code) or your computer.

### 3. A9 Road Network (OpenDRIVE file)

- Download from: [A9 Road Network](https://github.com/tum-gis/opendrive-testfeld-a9)
- Use the file: `2017-04-04_Testfeld_A9_Sued.xodr`

### 4. Python Packages

Install the following packages in your local environment terminal before running the code:
```python
pip install scenariogeneration pyproj
```
---

## 📂 Input File

- Prepare a file named `ReSimulation_dataframe.parquet` containing your drive activity data.
- Place it in a dedicated folder.  
  This folder will be used to store the generated `.xosc` files.

---

## ⚙️ Script Configuration

In `ReSimulation.py`, adjust the following variables:

```python
username = "your_username"
PATH_ESMINI = r"Path\to\esmini.exe"
road_network = r"Path\to\2017-04-04_Testfeld_A9_Sued.xodr"
vehicle_catalog_path = r"Path\to\Vehicle\Catalog"  # e.g., esmini-demo\resources\xosc\Catalogs\Vehicles
drive_list = [r"Path\to\folder\with\ReSimulation_dataframe.parquet"]
RUN_ESMINI = True  # Set to True to run the simulation using esmini.
SAVE_SIMULATION_VIDEO = True # Set to True if you also want to save a video recording of the simulation.
```
---

## ▶️ Run the Script

Run the `ReSimulation.py` script locally (e.g., in Visual Studio Code):

After running the script, the following `.xosc` files will be generated in the **same folder** as your input `ReSimulation_dataframe.parquet` file:

- `drive_scenario.xosc`
- `TrajectoryCatalog.xosc`

If `SAVE_SIMULATION_VIDEO = True` is set in the script, a simulation video will also be saved in the same folder:

- `simulated_drive.mp4`


