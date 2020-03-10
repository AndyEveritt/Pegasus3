# -*- coding: utf-8 -*-
"""
Created on Mon Oct 29 11:15:22 2018

@author: ajeve
"""

import cv2
import numpy as np
from scipy import signal
import scipy.sparse as sp
import matplotlib.image as mpimg
from skimage import color


def nothing(x):
    # We need a callback for the createTrackbar function.
    # It doesn't need to do anything, however.
    pass

 
def gaussian_kernel(k, s=0.5):
    # generate a (2k+1)x(2k+1) gaussian kernel with mean=0 and sigma = s
    probs = [np.exp(-z*z/(2*s*s))/np.sqrt(2*np.pi*s*s) for z in range(-k, k+1)]
    return np.outer(probs, probs)


# =============================================================================
# def create_graph(imfile, k=1., sigma=0.8, sz=1):
#     # create the pixel graph with edge weights as dissimilarities
#     rgb = mpimg.imread(imfile)[:,:,:3]
#     gauss_kernel = gaussian_kernel(sz, sigma)
#     for i in range(3):
#         rgb[:,:,i] = signal.convolve2d(rgb[:,:,i], gauss_kernel, boundary='symm', mode='same')
#     yuv = color.rgb2yiq(rgb)
#     (w, h) = yuv.shape[:2]
#     edges = {}
#     for i in range(yuv.shape[0]):
#         for j in range(yuv.shape[1]):
#             # compute edge weight for nbd pixel nodes for the node i,j
#             for i1 in range(i-1, i+2):
#                 for j1 in range(j-1, j+2):
#                     if (i1 == i and j1 == j):
#                         continue
#                     if (i1 >= 0 and j1 < h):
#                         wt = np.abs(yuv[i,j,0]-yuv[i1,j1,0])
#                         n1, n2 = ij2id(i,j,w,h), ij2id(i1,j1,w,h)
#                         edges[n1, n2] = edges[n2, n1] = wt
#     return edges
# =============================================================================

# create the pixel graph with edge weights as dissimilarities
rgb = mpimg.imread('satellite_local.png')[:,:,:3]
gauss_kernel = gaussian_kernel(1, 0.8)
for i in range(3):
    rgb[:,:,i] = signal.convolve2d(rgb[:,:,i], gauss_kernel, boundary='symm', mode='same')
yuv = color.rgb2yiq(rgb)
(w, h) = yuv.shape[:2]
edges = {}
for i in range(yuv.shape[0]):
    for j in range(yuv.shape[1]):
        # compute edge weight for nbd pixel nodes for the node i,j
        for i1 in range(i-1, i+2):
            for j1 in range(j-1, j+2):
                if (i1 == i and j1 == j):
                    continue
                if (i1 >= 0 and j1 < h):
                    wt = np.abs(yuv[i,j,0]-yuv[i1,j1,0])
#                    n1, n2 = ij2id(i,j,w,h), ij2id(i1,j1,w,h)
#                    edges[n1, n2] = edges[n2, n1] = wt


# =============================================================================
# img_original = cv2.imread('satellite_local.png')
# 
# 
# # Create a window and add two trackbars for controlling the thresholds.
# cv2.namedWindow('Canny Edge Detection')
# cv2.createTrackbar('Threshold1', 'Canny Edge Detection', 0, 1200, nothing)
# cv2.createTrackbar('Threshold2', 'Canny Edge Detection', 0, 1200, nothing)
# 
# while True:
#     # Get the latest threshold values.
#     threshold1 = cv2.getTrackbarPos('Threshold1', 'Canny Edge Detection')
#     threshold2 = cv2.getTrackbarPos('Threshold2', 'Canny Edge Detection')
#     
#     # Copy original img
#     img_harris = img_original.copy()
#     img_contour = img_original.copy()
#     
#     # The Harris corner detector operates on a grayscale image.
#     gray = cv2.cvtColor(img_harris, cv2.COLOR_BGR2GRAY)
#     corners = cv2.cornerHarris(gray,2,3,0.04)
#     # Dialate the detected corners to make them clearer in the output image.
#     corners = cv2.dilate(corners,None)
# 
#     # Perform thresholding on the corners to throw away some false positives.
#     img_harris[corners > 0.0002 * corners.max()] = [0,0,255]
# 
#     cv2.imshow("Harris", img_harris)
#     
# 
#     # Update the image using the latest threshold.
#     edges = cv2.Canny(img_original, threshold1, threshold2)
#     cv2.imshow('Canny Edge Detection', edges)
#     
#     
#     _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     cv2.drawContours(img_contour, contours, -1, (0,0,65535), 1)   # red contours
#     cv2.imshow('Contours', img_contour)
#     
# 
#     if cv2.waitKey(50) & 0xFF == ord('q'):
#         break
# 
# cv2.destroyAllWindows()
# =============================================================================

