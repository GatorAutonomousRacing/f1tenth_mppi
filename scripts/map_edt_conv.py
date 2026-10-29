import scipy
from PIL import Image
import os
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import yaml
from scipy.ndimage import distance_transform_edt, label

START_X, START_Y = 0.0, 0.0

MAPS_DIR_PATH = "/sim_ws/src/f1tenth_mppi/maps"
MAP_PNG_PATH = str(Path(MAPS_DIR_PATH, "Spielberg_map.png"))
MAP_YAML_PATH = str(Path(MAPS_DIR_PATH, "Spielberg_map.yaml"))
IMAGE_PATH = "/sim_ws/src/f1tenth_mppi/images"
PROJECT_PATH = "/sim_ws/src/f1tenth_mppi"


def parse_yaml(path : str) -> dict:
    with open(path, "r") as yaml_file:
        occ_grid_params : dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)
    return occ_grid_params

def save_occ_grid(map_img_path):
    params = parse_yaml(MAP_YAML_PATH)
    img = np.array(Image.open(map_img_path).convert("L")).astype(np.float64) / 255
    # ROS: occupied iff (1 - p) > occupied_thresh, so free iff p >= 1 - occupied_thresh
    return img >= (1.0 - params["occupied_thresh"])

def main():
    params = parse_yaml(MAP_YAML_PATH)
    res, ox, oy = params["resolution"], params["origin"][0], params["origin"][1]

    free = save_occ_grid(MAP_PNG_PATH)
    H = free.shape[0]

    labels, _ = label(free)   # default 4-connectivity: 1-px diagonal walls can't leak
    row = H - 1 - int((START_Y - oy) / res)
    col = int((START_X - ox) / res)
    assert labels[row, col] != 0, "seed landed on a wall pixel"
    drivable = labels == labels[row, col]

    plt.imsave(str(Path(IMAGE_PATH, "drivable.png")), drivable, cmap="gray")  # only the band should be white

    edt_m = distance_transform_edt(drivable) * res   # meters; everything off-track is 0
    edt_m.astype("<f4").tofile(str(Path(PROJECT_PATH, "scripts", "track_sdf.f32")))

    
if __name__ == "__main__":
    main()





