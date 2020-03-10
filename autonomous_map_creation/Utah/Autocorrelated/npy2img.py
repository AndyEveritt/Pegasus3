# -*- coding: utf-8 -*-

import numpy as np
import cv2


def normalise(img, scale=1):
	img = img.astype(np.float64)
	norm_img = scale * (img-np.min(img)) / np.max(img-np.min(img))
	norm_img = norm_img.astype(np.uint8)
	return norm_img


DEM_1 = np.load('./12SWH000400.npy')
DEM_2 = np.load('./12SWH200400.npy')

cv2.imwrite("12SWH000400.png", normalise(DEM_1, 255))
cv2.imwrite("12SWH200400.png", normalise(DEM_2, 255))