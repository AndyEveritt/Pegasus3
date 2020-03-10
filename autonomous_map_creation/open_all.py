import cv2
from imutils import paths
import numpy as np

for imagePath in paths.list_files("./pegasus-3.0/autonomous_map_creation/Soton/numpy"):
    image = np.load(imagePath)
    cv2.imshow(imagePath, image[100:-100, 100:-100])

    cv2.waitKey(0)