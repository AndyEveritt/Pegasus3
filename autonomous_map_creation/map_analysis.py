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


class map_data:
    def __init__(self, name, data, meta):
        self.name = name
        self.data = data
        self.meta = meta
        self.corners = []
        self.contours = []
        self.gradient = []

    def find_ROI(self, utm_tl, utm_br):
        self.ROI_utm_tl = utm_tl
        self.pixel_tl = utm_2_pixel(self.meta, [utm_tl[0], utm_tl[1]])
        self.pixel_br = utm_2_pixel(self.meta, [utm_br[0], utm_br[1]])
        print("[INFO] ROI pixels, {}: [ tl={}, br={} ]".format(
            self.name, self.pixel_tl, self.pixel_br))
        self.ROI = self.data[self.pixel_tl[1]
            :self.pixel_br[1], self.pixel_tl[0]:self.pixel_br[0]]


def nothing(x):
    # We need a callback for the createTrackbar function.
    # It doesn't need to do anything, however.
    pass


def display_image(img, img_name, window_name):
    # scale windows to display images
    window_width, window_height = scale_window(img)
    # Create window
    cv2.namedWindow(window_name + ": " + img_name, cv2.WINDOW_NORMAL)
    # Scale window
    cv2.resizeWindow(window_name + ": " + img_name,
                     window_width, window_height)
    # Show image
    cv2.imshow(window_name + ": " + img_name, img)


def normalise(img, scale=1):
    img = img.astype(np.float64)
    norm_img = scale * (img-np.min(img)) / np.max(img-np.min(img))
    norm_img = norm_img.astype(np.uint8)
    return norm_img


def otsu_canny(image, lowrate=0.1):
    if len(image.shape) > 2:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Otsu's thresholding
    ret, _ = cv2.threshold(image, thresh=0, maxval=255,
                           type=(cv2.THRESH_BINARY + cv2.THRESH_OTSU))
    edged = cv2.Canny(image, threshold1=(ret * lowrate), threshold2=ret)

    # return the edged image
    return edged


def scale_window(img, scale=0.5, screen_res=[1920, 1080]):
    # define the screen resulation
    scale_width = scale * screen_res[0] / img.shape[1]
    scale_height = scale * screen_res[1] / img.shape[0]
    scale = min(scale_width, scale_height)

    # resized window width and height
    window_width = int(img.shape[1] * scale)
    window_height = int(img.shape[0] * scale)
    return window_width, window_height


def analyse_satellite(img, name, harris_threshold=0.001, otsu_threshold=0.5):
    # scale windows to display images
    window_width, window_height = scale_window(img)

    # Copy original img
    img_harris = img.copy()
    img_contour = img.copy()

    # The Harris corner detector operates on a grayscale image.
    gray = cv2.cvtColor(img_harris, cv2.COLOR_BGR2GRAY)
    corners = cv2.cornerHarris(gray, 2, 3, 0.04)
    # Dialate the detected corners to make them clearer in the output image.
    corners = cv2.dilate(corners, None)
#	corners[corners > harris_threshold * corners.max()] = 255

    # Perform thresholding on the corners to throw away some false positives.
    img_harris[corners > harris_threshold * corners.max()] = [0, 0, 255]

    display_image(img_harris, name, "Satellite Harris")
    #display_image(normalise(corners, 255), name, "Satellite Corners")

    # Update the image using the latest threshold.
    edges = otsu_canny(img, otsu_threshold)
    #display_image(edges, name, "Satellite Canny Edge")

    _, contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_contour, contours, -1,
                     (0, 0, 65535), 1)   # red contours
    display_image(img_contour, name, "Satellite Contours")

    # Draw contours onto blank image
    img_blank = np.zeros([img_contour.shape[0], img_contour.shape[1]])
    cv2.drawContours(img_blank, contours, -1, 255, 1)   # contours
    #display_image(img_blank, name, "Contours")

    return corners, img_blank


def image_gradient(img, name):
    # scale windows to display images
    window_width, window_height = scale_window(img)

    sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=1)
    sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=1)

    magnitude = (sobelx**2 + sobely**2)**0.5
    #magnitude = normalise(magnitude, 255)

    #display_image(img, name, "DEM Original")
    #display_image(sobelx, name, "DEM Gradient x")
    #display_image(sobely, name, "DEM Gradient y")
    display_image(normalise(magnitude, 255), name, "DEM Gradient Magnitude")

    return magnitude


def utm_2_pixel(meta, utm):
    """
    Takes an input image and meta data (describing UTM position and pixel size).
    Returns the pixel in the image relating to an input UTM coordinate.
    """
    x_origin = meta[4]
    y_origin = meta[5]
    x_step = meta[0]
    y_step = meta[3]
    x_skew = meta[2]
    y_skew = meta[1]

    x_utm = utm[0]
    y_utm = utm[1]

    x_pixel = ((y_step*x_utm - x_skew*y_utm + x_skew*y_origin - y_step*x_origin) /
               (x_step*y_step - x_skew*y_skew))
    y_pixel = ((-y_skew*x_utm + x_step*y_utm + y_skew*x_origin - x_step*y_origin) /
               (x_step*y_step - x_skew*y_skew))

    return [int(x_pixel), int(y_pixel)]


def create_combined_map(Aerial_map, DEM_map):
    map_corners = Aerial_map.corners.copy()
    map_contours = Aerial_map.contours.copy()
    map_gradient = DEM_map.gradient.copy()

    # Interpolate map 2 & 3 to be the same size as map 1
    map_contours = cv2.resize(
        map_contours, (map_corners.shape[1], map_corners.shape[0]))
    map_gradient = cv2.resize(
        map_gradient, (map_corners.shape[1], map_corners.shape[0]))
    map_gradient *= Aerial_map.meta[0] / DEM_map.meta[0]

    map_corners[map_corners > 0.001 * map_corners.max()] = 255

    combined = np.zeros(
        [map_corners.shape[0], map_corners.shape[1], 3]).astype(np.uint8)
    combined[:, :, 0] = map_corners
    combined[:, :, 1] = map_contours
    combined[:, :, 2] = normalise(map_gradient, 255)

    # Create meta data
    meta = Aerial_map.meta
    meta[4:6] = Aerial_map.ROI_utm_tl

    combined_map = map_data("Combined", combined, meta)

    return combined_map


def create_drivable_map(name, Aerial_map, DEM_map, weight=[1e5, 0.8, 1.5], thresh_corner=5e-7, thresh_gradient=0.5, thresh=1):
    map_corners = Aerial_map.corners.copy()
    map_contours = Aerial_map.contours.copy()
    map_gradient = DEM_map.gradient.copy()

    # Interpolate map 2 & 3 to be the same size as map 1
    map_contours = cv2.resize(
        map_contours, (map_corners.shape[1], map_corners.shape[0]))
    map_gradient = cv2.resize(
        map_gradient, (map_corners.shape[1], map_corners.shape[0]))
    map_gradient *= Aerial_map.meta[0] / DEM_map.meta[0]
    DEM_map.gradient_interp = map_gradient
    map_contours = (normalise(map_contours, 1)).astype(np.float64)

    display_image(map_gradient, "...", "...")

    data = np.zeros_like(map_corners)

    # Create meta data
    meta = Aerial_map.meta
    meta[4:6] = Aerial_map.ROI_utm_tl

    # Probabilistic Map
    probability = weight[0]*map_corners + weight[1] * \
        map_contours + weight[2]*map_gradient
    probability_map = map_data("Obstacle Probability", probability, meta)
    display_image(probability, "Probability Obstacle Map", name)

    # Filter input maps
    map_corners[abs(map_corners) < thresh_corner] = 0
    #map_contours[map_contours == 255] = 1
    map_gradient[map_gradient < thresh_gradient] = 0

    data = weight[0]*map_corners + weight[1] * \
        map_contours + weight[2]*map_gradient
    obstacle_map = map_data("Obstacle", data, meta)
    display_image(obstacle_map.data, "Obstacle Map", name)

    binary = data.copy()
    binary[binary < thresh] = 0
    binary[binary > thresh] = 1
    obstacle_map.binary = binary
    display_image(obstacle_map.binary, "Thresholded Obstacle Map", name)

    return(obstacle_map, probability_map)


if __name__ == "__main__":
    # Load images
    print("[INFO] loading data")

    # Utah
    DEM_ac_1 = np.load('./Utah/Autocorrelated/12SWH000400.npy')
    DEM_ac_2 = np.load('./Utah/Autocorrelated/12SWH200400.npy')
    DEM_ac = np.concatenate((DEM_ac_1, DEM_ac_2), axis=1)
    DEM_n39 = np.load('./Utah/USGS/n39w111_10m.npy')
    satellite_local = cv2.imread('./Utah/satellite_local.png')
    satellite_global = cv2.imread('./Utah/satellite_global.png')
    Aerial = cv2.imread('./Utah/Aerial/q3128_se_NAIP2016_RGB.tif')
    USGS = cv2.imread('./Utah/USGS/n39w111_10m.tif')

    # Soton
    SH4015 = np.load('./Soton/numpy/su4015_DSM_1M.npy')
    SH4016 = np.load('./Soton/numpy/su4016_DSM_1M.npy')
    SH4017 = np.load('./Soton/numpy/su4017_DSM_1M.npy')
    SH4018 = np.load('./Soton/numpy/su4018_DSM_1M.npy')
    SH4019 = np.load('./Soton/numpy/su4019_DSM_1M.npy')
    SH4115 = np.load('./Soton/numpy/su4115_DSM_1M.npy')
    SH4116 = np.load('./Soton/numpy/su4116_DSM_1M.npy')
    SH4117 = np.load('./Soton/numpy/su4117_DSM_1M.npy')
    SH4118 = np.load('./Soton/numpy/su4118_DSM_1M.npy')
    SH4119 = np.load('./Soton/numpy/su4119_DSM_1M.npy')
    SH4215 = np.load('./Soton/numpy/su4215_DSM_1M.npy')
    SH4216 = np.load('./Soton/numpy/su4216_DSM_1M.npy')
    SH4217 = np.load('./Soton/numpy/su4217_DSM_1M.npy')
    SH4218 = np.load('./Soton/numpy/su4218_DSM_1M.npy')
    SH4219 = np.load('./Soton/numpy/su4219_DSM_1M.npy')
    SH4315 = np.load('./Soton/numpy/su4315_DSM_1M.npy')
    SH4316 = np.load('./Soton/numpy/su4316_DSM_1M.npy')
    SH4317 = np.load('./Soton/numpy/su4317_DSM_1M.npy')
    SH4318 = np.load('./Soton/numpy/su4318_DSM_1M.npy')
    SH4319 = np.load('./Soton/numpy/su4319_DSM_1M.npy')
    SH4415 = np.load('./Soton/numpy/su4415_DSM_1M.npy')
    SH4416 = np.load('./Soton/numpy/su4416_DSM_1M.npy')
    SH4417 = np.load('./Soton/numpy/su4417_DSM_1M.npy')
    SH4418 = np.load('./Soton/numpy/su4418_DSM_1M.npy')
    SH4419 = np.load('./Soton/numpy/su4419_DSM_1M.npy')

    SH41 = np.concatenate((SH4119, SH4118, SH4117, SH4116, SH4115), axis=0)
    SH42 = np.concatenate((SH4219, SH4218, SH4217, SH4216, SH4215), axis=0)
    SH43 = np.concatenate((SH4319, SH4318, SH4317, SH4316, SH4315), axis=0)
    SH44 = np.concatenate((SH4419, SH4418, SH4417, SH4416, SH4415), axis=0)
    SH = np.concatenate((SH41, SH42, SH43, SH44), axis=1)

    SH[SH == -9999] = SH[SH != -9999].mean()
    display_image(normalise(SH, 255), ".", ".")

    # Image meta data (TFW)
    Aerial_tfw = [1.0, 0.0, 0.0, -1.0, 516050.5, 4254699.5]
    DEM_ac_tfw = [5.0, 0.0, 0.0, -5.0, 500000.000000, 4260005.000000]
    DEM_n39_tfw = [9.158750350857, 0.0, 0.0, -9.158750350857,
                   499951.223334768729, 4317314.345700392]
    SH_tfw = [1.0, 0.0, 0.0, -1.0, 441000.0, 116000.0]

    Aerial_map = map_data("Aerial", Aerial, Aerial_tfw)
    DEM_ac_map = map_data("DEM_ac", DEM_ac, DEM_ac_tfw)
    DEM_n39_map = map_data("DEM_n39", DEM_n39, DEM_n39_tfw)
    SH_map = map_data("Soton", SH, SH_tfw)

    Aerial_map.overview = Aerial_map.data.copy()
    DEM_ac_map.overview = normalise(DEM_ac_map.data.copy(), 255)
    DEM_n39_map.overview = normalise(DEM_n39_map.data.copy(), 255)
    SH_map.overview = normalise(SH_map.data.copy(), 255)

    # Bounding Coordinates
#	tl = [38.419210, -110.799344]	# top left lat long coordinate
#	br = [38.392979, -110.765011]	# bottom right lat long coordinate
    tl = [38.417427, -110.795872]  # top left lat long coordinate
    br = [38.394543, -110.770687]  # bottom right lat long coordinate

    utm_tl = utm.from_latlon(tl[0], tl[1])
    utm_br = utm.from_latlon(br[0], br[1])

    # NYC Bounding Coordinates
    tl_NYC = [40.747921, -73.998345]
    br_NYC = [40.740671, -73.993153]

    # Find pixel values for bounding coordinates
    print("[INFO] finding ROI")
    Aerial_map.find_ROI(utm_tl, utm_br)
    DEM_ac_map.find_ROI(utm_tl, utm_br)
    DEM_n39_map.find_ROI(utm_tl, utm_br)

    cv2.rectangle(Aerial_map.overview, (Aerial_map.pixel_tl[0], Aerial_map.pixel_tl[1]), (
        Aerial_map.pixel_br[0], Aerial_map.pixel_br[1]), (255, 0, 0), 50)
    cv2.rectangle(DEM_ac_map.overview, (DEM_ac_map.pixel_tl[0], DEM_ac_map.pixel_tl[1]), (
        DEM_ac_map.pixel_br[0], DEM_ac_map.pixel_br[1]), 1, 20)
    cv2.rectangle(DEM_n39_map.overview, (DEM_n39_map.pixel_tl[0], DEM_n39_map.pixel_tl[1]), (
        DEM_n39_map.pixel_br[0], DEM_n39_map.pixel_br[1]), 1, 20)

    display_image(Aerial_map.overview, "Original", "Aerial")
    display_image(DEM_ac_map.overview, "Original", "DEM ac")
    display_image(DEM_n39_map.overview, "Original", "DEM n39")

    print("[INFO] analysing data")
    #analyse_satellite(Aerial, "Aerial")
    Aerial_map.corners, Aerial_map.contours = analyse_satellite(
        Aerial_map.ROI, "Aerial ROI", 0.001, 0.5)
    #analyse_DEM(DEM, "5m")
    #image_gradient(DEM, "5m")
    DEM_ac_map.gradient = image_gradient(DEM_ac_map.ROI, "5m ROI")
    DEM_n39_map.gradient = image_gradient(DEM_n39_map.ROI, "10m ROI")

    SH_map.gradient = image_gradient(SH_map.data, "1m Soton")
    SH_map.gradient[SH_map.gradient < 0.5] = 0
    SH_map.gradient[SH_map.gradient < 1] = 0
    SH_map.gradient[SH_map.gradient > 1] = 1
    display_image(SH_map.gradient, "after", ".")

    print("[INFO] creating obstacle map")
    combined_map = create_combined_map(Aerial_map, DEM_ac_map)
#	display_image(combined_map.data, "Harris, Contours, & Gradient", "Drivable map")

    obstacle_ac_map, probability_ac_map = create_drivable_map("ac", Aerial_map, DEM_ac_map, weight=[1e5, 1, 1],
                                                              thresh_corner=5e-7, thresh_gradient=0.5)

    obstacle_n39_map, probability_n39_map = create_drivable_map("n39", Aerial_map, DEM_n39_map, weight=[1e5, 1, 1.5],
                                                                thresh_corner=5e-7, thresh_gradient=0.5)

    # apply the overlay
    output_overlay = Aerial_map.ROI.copy()
    alpha = 0.5
    cv2.addWeighted(combined_map.data, alpha, output_overlay,
                    1 - alpha, 0, output_overlay)
    #display_image(output_overlay, "Overlay", "Drivable map")

    h1, w1 = combined_map.data.shape[:2]
    h2, w2 = output_overlay.shape[:2]
    h3, w3 = Aerial_map.ROI.shape[:2]

    # create empty matrix
    vis = np.zeros((max(h1, h2), w1+w2+w3, 3), np.uint8)

    # combine 2 images
    vis[:h1, :w1, :3] = combined_map.data
    vis[:h2, w1:w1+w2, :3] = output_overlay
    vis[:h2, w1+w2:w1+w2+w3, :3] = Aerial_map.ROI
    display_image(vis, "Combined", "Drivable map")

    # Create a window and add two trackbars for controlling the thresholds.
    #cv2.namedWindow('Canny Edge Detection')
    #cv2.createTrackbar('Threshold1', 'Canny Edge Detection', 0, 1200, nothing)
    #cv2.createTrackbar('Threshold2', 'Canny Edge Detection', 0, 1200, nothing)

    while True:

        if cv2.waitKey(50) & 0xFF == ord('q'):
            cv2.imwrite("./maps/tmp_obstacle_map_binary.png",
                        (255*obstacle_ac_map.binary).astype(int))
            cv2.imwrite("./maps/tmp_obstacle_map.png",
                        (255*obstacle_ac_map.data).astype(int))
            cv2.imwrite("./maps/tmp_probability_ac_map.png",
                        (255*probability_ac_map.data).astype(int))
            cv2.imwrite("./maps/tmp_probability_n39_map.png",
                        (255*probability_n39_map.data).astype(int))
            cv2.imwrite("./maps/tmp_Soton_map.png",
                        (255*SH_map.gradient).astype(int))
            break

    cv2.destroyAllWindows()
