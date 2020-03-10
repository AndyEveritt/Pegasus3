# -*- coding: utf-8 -*-
"""
Created on Mon Oct 29 11:15:22 2018

@author: Andy Everitt (aje2g15@soton.ac.uk)
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy.interpolate as interp
import utm

from asc2img import get_data
import map_analysis as ma


if __name__ == "__main__":
	# Load images
	SAerial = cv2.imread('./Soton/Aerial.jpg')
	SDEM = np.load('./Soton/su4115_DSM_1M.npy')

	SDEM[SDEM==-9999] = 0



	# Image meta data (TFW)
	SAerial_tfw = [1.0, 0.0, 0.0, -1.0, 441000.0, 116000.0]
	SDEM_tfw = [1.0, 0.0, 0.0, -1.0, 441000.0, 116000.0]

	SAerial_map = ma.map_data("Soton Aerial", SAerial, SAerial_tfw)
	SDEM_map = ma.map_data("Soton DEM", SDEM, SDEM_tfw)


	SAerial_map.overview = SAerial_map.data.copy()
	SDEM_map.overview = ma.normalise(SDEM_map.data.copy(), 255)

	ma.display_image(SAerial_map.overview, "Original", "Aerial")
	ma.display_image(SDEM_map.overview, "Original", "DEM")



	SAerial_map.corners, SAerial_map.contours = ma.analyse_satellite(SAerial_map.data, "Aerial ROI", 0.001, 0.5)
	SDEM_map.gradient = ma.image_gradient(SDEM_map.data, "5m ROI")




#	SDEM_map.gradient = ma.image_gradient(SDEM_map.data, "1m Soton")
#	SDEM_map.gradient[SDEM_map.gradient < 0.5] = 0
#	SDEM_map.gradient[SDEM_map.gradient < 1] = 0
#	SDEM_map.gradient[SDEM_map.gradient > 1] = 1
#	ma.display_image(SDEM_map.gradient, "after",".")



	combined_map = ma.create_combined_map(SAerial_map, SDEM_map)
#	display_image(combined_map.data, "Harris, Contours, & Gradient", "Drivable map")

	drivable_map = ma.create_drivable_map(SAerial_map, SDEM_map, weight=[1e5,1,1],
									thresh_corner=5e-7, thresh_gradient=0.5)

	# apply the overlay
	output_overlay = SAerial_map.data.copy()
	alpha = 0.5
	cv2.addWeighted(combined_map.data, alpha, output_overlay, 1 - alpha, 0, output_overlay)
	#display_image(output_overlay, "Overlay", "Drivable map")

	h1, w1 = combined_map.data.shape[:2]
	h2, w2 = output_overlay.shape[:2]
	h3, w3 = SAerial_map.data.shape[:2]

	#create empty matrix
	vis = np.zeros((max(h1, h2), w1+w2+w3,3), np.uint8)

	#combine 2 images
	vis[:h1, :w1,:3] = combined_map.data
	vis[:h2, w1:w1+w2,:3] = output_overlay
	vis[:h2, w1+w2:w1+w2+w3,:3] = SAerial_map.data
	ma.display_image(vis, "Combined", "Drivable map")

	# Create a window and add two trackbars for controlling the thresholds.
	#cv2.namedWindow('Canny Edge Detection')
	#cv2.createTrackbar('Threshold1', 'Canny Edge Detection', 0, 1200, nothing)
	#cv2.createTrackbar('Threshold2', 'Canny Edge Detection', 0, 1200, nothing)

	while True:

		if cv2.waitKey(50) & 0xFF == ord('q'):
			break

	cv2.destroyAllWindows()

