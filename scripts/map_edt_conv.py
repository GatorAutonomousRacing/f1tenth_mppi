import scipy
from scipy.ndimage import distance_transform_edt
from PIL import Image
import os
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import yaml

MAPS_DIR_PATH = "/sim_ws/src/f1tenth_mppi/maps"
MAP_PNG_PATH = str(Path(MAPS_DIR_PATH, "Spielberg_map.png"))
MAP_YAML_PATH = str(Path(MAPS_DIR_PATH, "Spielberg_map.yaml"))



def parse_yaml(path : str) -> dict:
    with open(path, "r") as yaml_file:
        occ_grid_params : dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)
    return occ_grid_params

    

def open_map(path : str) -> None:
    map_image = Image.open(path).convert("L")
    map_img_arr = np.array(map_image)

    occ_grid_params = parse_yaml(MAP_YAML_PATH)
    free_threshold = occ_grid_params["free_thesh"]
    occupied_threshold  = occ_grid_params["occupied_thresh"]

    map_img_arr_normalized = map_img_arr.astype(np.float64)/255
    binary_occupancy_grid = map_img_arr_normalized > free_threshold
    plt.imsave("map_img.png", binary_occupancy_grid)
    

open_map(MAP_PNG_PATH)
parse_yaml(MAP_YAML_PATH)
    