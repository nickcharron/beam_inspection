import argparse
import sys
import os
import json
import logging
import shutil

from utils import load_datasets_config
from run_helpers import run_slam, run_map_refinement, run_map_builder, preprocess_bag
from run_helpers import run_image_extractor, run_image_selection
from run_helpers import run_quantify_defects, run_map_labeler, run_sam_labeling
from params import *

logger = logging.getLogger("RUN_ALL")

trajectory_checksum = ""
map_checksum = ""
map_builder_trajectory_checksum = ""


def setup_logger(output_file: str):
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger.addHandler(handler1)
    if output_file:
        handler2 = logging.FileHandler(output_file)
        handler2.setLevel(logging.DEBUG)
        handler2.setFormatter(formatter)
        logger.addHandler(handler2)


def parse_args(args):
    parser = argparse.ArgumentParser(
        description='Run full inspection pipeline')
    parser.add_argument('-d', type=int, help='dataset number')
    args = parser.parse_args()
    return args


def load_run_all_config():
    config_path = os.path.join(PIPELINE_INPUTS, "run_all_config.json")
    logger.info(f"loading pipeline config json from {config_path}")
    f = open(config_path)
    config = json.load(f)
    f.close()
    return config


def run(dataset_number: int):
    datasets_config = load_datasets_config()
    if dataset_number > (len(datasets_config["datasets"]) - 1):
        logger.error("Invalid dataset_number")
        exit()

    run_all_config = load_run_all_config()
    dataset_path = datasets_config["datasets"][dataset_number]["path"]
    if not os.path.exists(dataset_path):
        err = f"dataset path does not exist: {dataset_path}"
        logger.error(err)
        raise Exception(err)

    output_path = os.path.join(dataset_path, RESULTS_FOLDER)

    if not os.path.exists(output_path):
        logger.info("creating output directory: %s", output_path)
        os.mkdir(output_path)

    log_path = os.path.join(output_path, "run_all_pipeline.log")
    setup_logger(log_path)


    # preprocess
    if run_all_config["preprocess_bag"]:
        preprocess_bag(datasets_config, dataset_number)   

    # slam
    slam_config = run_all_config["slam"]
    if slam_config["enable"]:
        run_slam(datasets_config, output_path, dataset_number, slam_config["rosbag_play_rate"],
                 slam_config["local_mapper_config"], slam_config["global_mapper_config"])
    else:
        logger.info("skipping slam")

    # refinement
    ref_config = run_all_config["map_refinement"]
    run_refinement = ref_config["run_submap_refinement"] or ref_config[
        "run_submap_alignment"] or ref_config["run_posegraph_optimization"] or ref_config["run_batch_optimizer"]
    if run_refinement:
        run_map_refinement(output_path, ref_config["run_submap_refinement"], ref_config[
            "run_submap_alignment"], ref_config["run_posegraph_optimization"], ref_config["run_batch_optimizer"])
    else:
        logger.info("skipping map refinement")

    # map builder
    if run_all_config["run_map_builder"]:
        run_map_builder(datasets_config, output_path,
                        dataset_number, run_all_config["use_refined_results_for_map"])
    else:
        logger.info("skipping map builder")

    # image extractor
    if run_all_config["run_image_extractor"]:
        run_image_extractor(datasets_config, output_path, dataset_number)
    else:
        logger.info("skipping image extractor")

    # image selection
    if run_all_config["run_image_selection"]:
        run_image_selection(output_path)
    else:
        logger.info("skipping image extractor")
        img_extractor_output = os.path.join(
            output_path, IMAGE_EXTRACTOR_FOLDER)
        camera_list = os.path.join(img_extractor_output, "CameraList.json")
        new_list_file = os.path.join(
            img_extractor_output, "CameraListNew.json")
        if not os.path.exists(new_list_file) and os.path.exists(camera_list):
            logger.info(
                f"no CameraListNew file, copying from {camera_list} to {new_list_file}")
            shutil.copyfile(camera_list, new_list_file)

    # SAM image labeling
    if run_all_config["run_sam_labeling"]:
        run_sam_labeling(output_path)

    # map labeler
    color_map = run_all_config["map_labeler"]["color_map"]
    label_defects = run_all_config["map_labeler"]["label_defects"]
    if color_map or label_defects:
        run_map_labeler(output_path, color_map, label_defects)
    else:
        logger.info("skipping map labeler")

    # quantify defects
    if run_all_config["run_quantify_defects"]:
        run_quantify_defects(output_path)
    else:
        logger.info("skipping map defect quantification")

    logger.info("run_all.py pipeline completed successfully")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    run(args.d)
