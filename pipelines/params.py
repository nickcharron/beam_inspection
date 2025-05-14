import rospkg
import os
from pathlib import Path

# Helper functions for getting paths
def get_pipelines_path() -> str:
    current_file = os.path.abspath(__file__)
    return Path(current_file).parent

def get_pipeline_inputs_path() -> str:
    return os.path.join(get_pipelines_path(), "inputs")


def get_calibration_results_path() -> str:
    return os.path.join(get_pipelines_path(), "calibrations", "ig2")

def get_beam_inspection_path() -> str:
    return rospkg.RosPack().get_path("beam_inspection")

def get_bag_processing_path() -> str:
    beam_inspection_path = get_beam_inspection_path()
    return os.path.join(beam_inspection_path, "tools", "bag_processing")

# File and Directory Paths 
BEAM_SLAM_LAUNCH_PATH = rospkg.RosPack().get_path("beam_slam_launch")
BS_LAUNCH_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "launch")
BS_CONFIG_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "config")
CATKIN_WS = "/userhome/catkin_ws_inspection"
PIPELINES_PATH = get_pipelines_path()
PIPELINE_INPUTS = get_pipeline_inputs_path()
BEAM_ROBOTICS_PATH = Path(PIPELINES_PATH).parent
CALIBRATION_PATH = get_calibration_results_path()
EXTRINSICS_JSON_PATH = os.path.join(
    CALIBRATION_PATH, "extrinsics_reorg.json")
INTRINSICS_PATH = CALIBRATION_PATH
SAM_CHECKPOINT = os.path.join(
        Path.home(), "beam_data", "sam", "sam_vit_h_4b8939.pth")

# Folder names
RESULTS_FOLDER = "results"
SLAM_OUTPUT_FOLDER = "slam"
MAP_BUILDER_FOLDER = "map_builder"
GLOBAL_MAPPER_RESULTS = "global_mapper_results"
GLOBAL_MAP_REFINEMENT_RESULTS = "global_map_refined_results"
IMAGE_EXTRACTOR_FOLDER = "image_extractor"
MAP_LABELER_FOLDER = "map_labeler"

# Filenames
RAW_BAG_FILE = "raw.bag"
SLAM_BAG_FILE = "data.bag"
INSPECTION_BAG_FILE = "inspection.bag"
LOCAL_MAPPER_BAG_FILE = "local_mapper_results.bag"
