import os
import subprocess
import glob
import xml.etree.ElementTree as ET

def strip_namespace(tag):
    return tag.split('}', 1)[-1] if '}' in tag else tag

def get_ego_entity_index(xosc_path, ego_name="Ego"):
    """
    Parses the .xosc OpenSCENARIO file and returns the index of the ego vehicle
    in the <Init><Actions><Private> section, used for camera tracking.
    """

    tree = ET.parse(xosc_path)
    root = tree.getroot()

    # Find the <Init> element
    init_element = None
    for elem in root.iter():
        if strip_namespace(elem.tag) == "Init":
            init_element = elem
            break

    if init_element is None:
        raise ValueError("No <Init> section found in the .xosc file.")

    # Inside <Init>, locate the <Actions> element
    actions_element = next((e for e in init_element if strip_namespace(e.tag) == "Actions"), None)
    if actions_element is None:
        raise ValueError("No <Actions> element found inside <Init>.")

    # Extract all entityRef names from <Private> elements
    entity_refs = []
    for private in actions_element:
        if strip_namespace(private.tag) == "Private":
            entity_name = private.attrib.get("entityRef")
            if entity_name:
                entity_refs.append(entity_name)

    # Return the index of the ego entity name
    try:
        return entity_refs.index(ego_name)
    except ValueError:
        raise ValueError(f"Ego entity '{ego_name}' not found in Init actions. Found: {entity_refs}")
    

def run_esmini(RUN_ESMINI, PATH_ESMINI, OUTPUT_DIR):
    """
    Run esmini with screen capture enabled and create a video output using ffmpeg.
    Also cleans up intermediate screenshot files.
    """
    if RUN_ESMINI:
        esmini_exe = os.path.abspath(PATH_ESMINI)
        xosc_path = os.path.join(OUTPUT_DIR, "drive_scenario.xosc")

        # Get the index of Ego for camera_follow
        ego_index = get_ego_entity_index(xosc_path)

        print("✅ Launching esmini...")
        result = subprocess.run([
            esmini_exe,
            "--osc", xosc_path,
            "--window", "60", "60", "800", "400",
            "--camera_mode", "driver",
            "--follow_object", str(ego_index),
            "--capture_screen"
        ], cwd=OUTPUT_DIR)

        print("✅ esmini exited with code:", result.returncode)

        if result.returncode == 0:
            print("✅ Generating video with ffmpeg...")
            ffmpeg_cmd = [
                "ffmpeg", "-framerate", "30",
                "-i", "screen_shot_%05d.tga",
                "-pix_fmt", "yuv420p",
                "simulated_drive.mp4"
            ]
            subprocess.run(ffmpeg_cmd, cwd=OUTPUT_DIR)

            # Clean up .tga screenshot images
            print("🧹 Cleaning up .tga screenshots...")
            tga_files = glob.glob(os.path.join(OUTPUT_DIR, "screen_shot_*.tga"))
            for tga in tga_files:
                try:
                    os.remove(tga)
                except Exception as e:
                    print(f"⚠️ Failed to delete {tga}: {e}")

            print("✅ Cleanup complete.")
        else:
            print("❌ esmini failed. Skipping video generation.")


def run_esmini_without_saving_video(RUN_ESMINI, PATH_ESMINI, OUTPUT_DIR):
    """
    Run esmini without capturing screenshots or generating a video.
    """
    if RUN_ESMINI:
        #print("✅ Working directory:", os.getcwd())

        esmini_exe = os.path.abspath(PATH_ESMINI)
        xosc_path = os.path.join(OUTPUT_DIR, "drive_scenario.xosc")
        # Get the index of Ego for camera_follow
        ego_index = get_ego_entity_index(xosc_path)

        print("✅ Launching esmini...")
        result = subprocess.run([
            esmini_exe,
            "--osc", xosc_path,
            "--window", "60", "60", "800", "400",
            "--camera_mode", "driver",
            "--follow_object", str(ego_index)
        ], cwd=OUTPUT_DIR)

        print("✅ esmini exited with code:", result.returncode)            