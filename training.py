import warnings
warnings.filterwarnings('ignore')
import os
import yaml
import glob
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_theme(style="darkgrid", font_scale=1.5, font="SimHei", rc={"axes.unicode_minus":False})

import torch
from ultralytics import YOLO

from PIL import Image

seed = 1
random.seed(seed)
np.random.seed(seed)
torch.manual_seed(seed)
torch.cuda.manual_seed(seed)
model = YOLO("yolov8n.pt")
config = {
    "path": "/kaggle/input/yolo-drone-detection-dataset/drone_dataset",
    "train": "/kaggle/input/yolo-drone-detection-dataset/drone_dataset/train",
    "val": "/kaggle/input/yolo-drone-detection-dataset/drone_dataset/valid",
    "nc": 1,
    "names": ["drone"],
}

with open("data.yaml", "w") as file:
    yaml.dump(config, file, default_flow_style=False)