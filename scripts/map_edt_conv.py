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




def parse_yaml(path : str) -> None:
    with open(path, "r") as yaml_file:
        occ_grid_params : dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)
        print(occ_grid_params)
        


def open_map(path : str) -> None:
    map_image = Image.open(path).convert("L")
    print(f"Format: {map_image.format}")
    map_img_arr = np.array(map_image)
    print(map_img_arr.shape)
    

open_map(MAP_PNG_PATH)
parse_yaml(MAP_YAML_PATH)
    