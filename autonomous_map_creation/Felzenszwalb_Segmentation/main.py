from scipy import ndimage
import matplotlib.pyplot as plt
from filter import smooth
from segment_graph import *
import time
import cv2
import numpy as np


# --------------------------------------------------------------------------------
# Segment an image:
# Returns a color image representing the segmentation.
#
# Inputs:
#           in_image: image to segment.
#           sigma: to smooth the image.
#           k: constant for threshold function.
#           min_size: minimum component size (enforced by post-processing stage).
#
# Returns:
#           num_ccs: number of connected components in the segmentation.
# --------------------------------------------------------------------------------
def segment(in_image, sigma, k, min_size):
    start_time = time.time()
    height, width, band = in_image.shape
    print("Height:  " + str(height))
    print("Width:   " + str(width))

    # gaussian blur
    smooth_red_band = smooth(in_image[:, :, 0], sigma)
    smooth_green_band = smooth(in_image[:, :, 1], sigma)
    smooth_blue_band = smooth(in_image[:, :, 2], sigma)

    # build graph
    edges_size = width * height * 4
    edges = np.zeros(shape=(edges_size, 3), dtype=object)
    num = 0
    for y in range(height):
        for x in range(width):
            if x < width - 1:
                edges[num, 0] = int(y * width + x)
                edges[num, 1] = int(y * width + (x + 1))
                edges[num, 2] = diff(smooth_red_band, smooth_green_band, smooth_blue_band, x, y, x + 1, y)
                num += 1
            if y < height - 1:
                edges[num, 0] = int(y * width + x)
                edges[num, 1] = int((y + 1) * width + x)
                edges[num, 2] = diff(smooth_red_band, smooth_green_band, smooth_blue_band, x, y, x, y + 1)
                num += 1

            if (x < width - 1) and (y < height - 2):
                edges[num, 0] = int(y * width + x)
                edges[num, 1] = int((y + 1) * width + (x + 1))
                edges[num, 2] = diff(smooth_red_band, smooth_green_band, smooth_blue_band, x, y, x + 1, y + 1)
                num += 1

            if (x < width - 1) and (y > 0):
                edges[num, 0] = int(y * width + x)
                edges[num, 1] = int((y - 1) * width + (x + 1))
                edges[num, 2] = diff(smooth_red_band, smooth_green_band, smooth_blue_band, x, y, x + 1, y - 1)
                num += 1
    # Segment
    u = segment_graph(width * height, num, edges, k)

    # post process small components
    for i in range(num):
        a = u.find(edges[i, 0])
        b = u.find(edges[i, 1])
        if (a != b) and ((u.size(a) < min_size) or (u.size(b) < min_size)):
            u.join(a, b)

    num_cc = u.num_sets()
    output = np.zeros(shape=(height, width, 3))

    # pick random colors for each component
    colors = np.zeros(shape=(height * width, 3))
    for i in range(height * width):
        colors[i, :] = random_rgb()

    for y in range(height):
        for x in range(width):
            comp = u.find(y * width + x)
            output[y, x, :] = colors[comp, :]

    # creating overlayed output and input image
    output_overlay = in_image.copy()
    overlay = np.uint8(output.copy())

    # set largest segment to green (driveable) and everywhere else to 0
    median = np.median(overlay[:,:,1])
    overlay[:,:,1] = 255*(overlay[:,:,1] == median).astype(float)
    overlay[:,:,0] = overlay[:,:,2] = 0

    # apply the overlay
    alpha = 0.3
    cv2.addWeighted(overlay, alpha, output_overlay, 1 - alpha, 0, output_overlay)

    # calculate time taken
    elapsed_time = time.time() - start_time
    print(
        "Execution time: " + str(int(elapsed_time / 60)) + " minute(s) and " + str(
            int(elapsed_time % 60)) + " seconds")    

    # displaying the result
    fig = plt.figure()
    a = fig.add_subplot(1, 3, 1)
    plt.imshow(in_image)
    a.set_title('Original Image')
    a = fig.add_subplot(1, 3, 2)
    plt.imshow(output)
    a.set_title('Segmented Image')
    plt.savefig('./results/satellite_local.png')
    a = fig.add_subplot(1, 3, 3)
    plt.imshow(output_overlay)
    a.set_title('Passable Terrain')
    plt.show()
    plt.imsave('./results/satellite_local_segmented.png', output_overlay)
#    cv2.imwrite("results/satellite_local_segmented.png".format(input_path), output_overlay)
    
    return(output)


if __name__ == "__main__":
    sigma = 0.5
    k = 500
    min = 50
    input_path = "../Utah/satellite_local.png"

    # Loading the image
    input_image = ndimage.imread(input_path, flatten=False, mode=None)
    print("Loading is done.")
    print("processing...")
    output = segment(input_image, sigma, k, min)
