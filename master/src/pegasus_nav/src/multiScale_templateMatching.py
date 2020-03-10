# import the necessary packages
import numpy as np
import imutils
import cv2
import sys


class ballLocations:
    """[summary]
    Stores the location of the bounding rectangles, centre, radius and the scale
    space they were found in for all possible ball locations
    """

    def __init__(self):
        self.location = []
        self.centre = []
        self.radius = []
        self.scale = []
        self.value = []


class Accumulator:
    """[summary]
    Used to accumulate predicted ball locations
    """

    def createAccum(self, size):
        """[summary]
        create empty accumulator 

        Arguments:
            size {[type]} -- desired accumulator size
        """
        self.accum = np.zeros(size)

    def vote(self, position, radius, weight):
        """[summary]
        vote using a gaussian weigthing around position

        Arguments:
            position {[type]} -- position of vote (ball location)
            radius {[type]} -- radius of vote (ball radius)
            weight {[type]} -- weight of vote (inverse match result)
        """
        mask = np.zeros_like(self.accum)
        cv2.circle(mask, position[::-1], radius, float(weight), -1)
        # mask = cv2.GaussianBlur(mask, (0,0), radius/8)
        self.accum[mask != 0] += radius * weight * mask[mask != 0]

    def errodeAccum(self, kernel=(5, 5), errosion_iters=2):
        self.accum = cv2.erode(self.accum, kernel=kernel,
                               iterations=errosion_iters)

    def normaliseAccum(self):
        cv2.normalize(self.accum, self.accum, 0, 1, cv2.NORM_MINMAX)

    def convert8Bit(self):
        return (255*self.accum).astype('uint8')

    def convertFloat(self):
        self.accum = (self.accum / 255.).astype('float64')

    def inverseAccum(self):
        return 255 - self.accum

    def findMax(self):
        """ find the max response in the accumulator """
        (_, self.maxVal, _, self.maxLoc) = cv2.minMaxLoc(self.accum)

    def findBlob(self, minThreshold=0, maxThreshold=255):
        accum8bit = self.convert8Bit()
        # inverse_accum = self.inverseAccum()
        accum8bit = cv2.GaussianBlur(accum8bit, (0,0), 8)
        
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        params.filterByColor = True
        params.blobColor = 255

        # Change thresholds
        params.minThreshold = minThreshold
        params.maxThreshold = maxThreshold

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87

        

        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        self.keypoints = detector.detect(accum8bit)

        # Draw keypoints
        try:
            inverse_accum = cv2.drawKeypoints(accum8bit, self.keypoints, np.array(
                []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        except:
            print "No keypoints"
            pass
        
        # cv2.imshow("inv accum", inverse_accum)

        self.convertFloat()        


def checkEndKey(key):
    """[summary] 
    Checks if key 'q' has been pressed

    Arguments:
        key {int} -- key pressed

    Returns:
        [bool] -- returns True is key is 'q'
    """
    if key == ord("q"):
        cv2.destroyAllWindows()
        cam.release()
        return True


def getTemplate(filename, width=24):
    """[summary]
    Retrieves a template for template matching.
    Resizes template.
    Converts template into HSV color space.

    Arguments:
        filename {str} -- directory of template
        width {int} -- width that the template should be resized to

    Returns:
        [tH] -- template height
        [tW] -- template width
        [hsv_template] -- template in HSV color space
    """
    template = cv2.imread(filename)
    template = imutils.resize(template, width=width)
    hsv_template = cv2.cvtColor(template, cv2.COLOR_BGR2HSV)
    (tH, tW) = template.shape[:2]
    # cv2.imshow("Template", template)
    # cv2.imshow("Template HSV", hsv_template)
    return tH, tW, hsv_template


def multiScaleTemplateMatching(display_image, image, template, scale_range=(0.05, 1.0, 20), threshold="minVal"):
    """[summary]
    Template matching at multiple scales within 'scale range'

    Arguments:
        display_image {ndarray} -- image to display results on
        image {ndarray} -- image to search
        template {ndarray} -- template used to search image

    Keyword Arguments:
        scale_range {tuple} -- Range of scales to resize image to.
                                Uses numpy.linspace
                                (default: {(0.05, 1.0, 20)})

    Returns:
        [accumulator] -- class for storing the accumulator to estimate the modal
                            ball position
        [ball_locations] -- class for storing potential ball locations and the
                                metadata for each point
        [found] -- the position of single best match in the image
    """
    def resizeImage(image, scale):
        """[summary]
        resize the image according to the scale, and keep track
        of the ratio of the resizing

        Arguments:
            image {ndarray} -- image to be resized
            scale {float} -- scale factor

        Returns:
            [resized] -- the output (resized) image
            [r] -- the inverse scale factor
        """

        resized = imutils.resize(image, width=int(image.shape[1] * scale))
        r = image.shape[1] / float(resized.shape[1])

        return resized, r

    def resizeError(resized, template_size):
        """[summary]
        Checks if the resized image is smaller than the template.

        Arguments:
            resized {ndarray} -- resized image
            template_size {tuple} -- contains the template height and width
                                        (tH, tW)

        Returns:
            [bool] -- true when the template is larger than the image
        """

        resize_error = False
        if resized.shape[0] < template_size[0] or resized.shape[1] < template_size[1]:
            resize_error = True

        return resize_error

    def findPotentialLocations(display_image, resized_image, template, locations, scale_ratio, threshold="minVal"):
        """[summary]
        Applies OpenCV template matching to find possible matches in the
        image. Uses 'cv2.TM_SQDIFF_NORMED' which means a lower value is
        a better match. For results below a threshold, add their data to
        a 'ballLocations' class

        Arguments:
            display_image {ndarray} -- image to display results on
            resized_image {ndarray} -- image to search
            template {ndarray} -- template used to search image
            locations {ballLocations} -- class for storing potential ball locations and the
                                    metadata for each point
            scale_ratio {float} -- inverse scale applied to image for match
                                        (effectively the scale needed to be applied
                                        to the ball)

        Keyword Arguments:
            threshold_scale {float} -- a multiplication factor that controls the threshold
                                        of the template matching results (default: {1.0})

        Returns:
            [results] {ndarray} -- template matching result where each element is the
                                    match value for that point in the image
            [(minVal, minLoc)] -- the value and location of the best match in the image
        """
        
        result = cv2.matchTemplate(
            resized_image, template, cv2.TM_SQDIFF_NORMED)
        (minVal, _, minLoc, _) = cv2.minMaxLoc(result)

        (tH, tW) = template.shape[:2]

        if minVal/eval(threshold) < 0.8:
            # this is for speed
            threshold = minVal
        else:
            threshold = eval(threshold)*0.99
        
        loc = np.where(result <= threshold)
        if any(map(len, loc)):
            for pt in zip(*loc[::-1]):
                locations.location.append((int(pt[0]*scale_ratio), int(pt[1]*scale_ratio), int(
                    (pt[0]+tW) * scale_ratio), int((pt[1]+tH) * scale_ratio)))

                locations.centre.append(
                    (int((pt[0] + tW/2) * scale_ratio), int((pt[1] + tH/2) * scale_ratio)))
                locations.radius.append(int(tW/2 * scale_ratio))
                
                locations.scale.append(scale_ratio)
                locations.value.append(result[pt[1]][pt[0]])
                
                # cv2.rectangle(display_image, (int(pt[0]*scale_ratio), int(pt[1]*scale_ratio)), (int(
                #     (pt[0]+tW) * scale_ratio), int((pt[1]+tH) * scale_ratio)), (255, 0, 0), 1)

        return result, (minVal, minLoc)

    def createAccumulator(display_image, image, ball_locations):
        """[summary]
        Creates an accumulator that can be used to estimate the most likely ball
        position by gathering evidence using each potential match.

        Arguments:
            display_image {[type]} -- image to display results on
            ball_locations {[type]} -- class storing ball locations

        Returns:
            Accumulator -- Class storing accumulator
        """
        # create accumulator for later
        accumulator = Accumulator()
        accumulator.createAccum((image.shape[:2]))

        if any(map(len, ball_locations.location)):
            # rect_list, weights = cv2.groupRectangles(ball_locations.location, 1)
            # for rect in rect_list:
            #     cv2.rectangle(image, (rect[0], rect[1]), (rect[2], rect[3]), (0,0,255), 5)5

            for i, centre in enumerate(ball_locations.centre):
                accumulator.vote(
                    centre[::-1], ball_locations.radius[i], 1/ball_locations.value[i])

            accumulator.normaliseAccum()
            # cv2.imshow("accumulator", accumulator.accum)

            # Find & display 'best' match
            accumulator.findMax()
            # cv2.circle(display_image, accumulator.maxLoc, 10, (0, 0, 255), 4)
            
            accumulator.findBlob()
            
        return accumulator

    def updateMinLocation(found, minVal, minLoc, r):
        """[summary]
        Updates best match variables

        Arguments:
            found {[type]} -- current best match
            minVal {[type]} -- query match value
            minLoc {[type]} -- query match location
            r {[type]} -- query match scale ratio

        Returns:
            {tuple} -- new best match
        """
        # if we have found a new min correlation value, then update
        # the bookkeeping variable
        if found == (None, None, None) or minVal < found[0]:
            found = (minVal, minLoc, r)
        return found

    def drawMinLoc(display_image, found):
        """[summary]
        Draws a circle around best match

        Arguments:
            display_image {[type]} -- image to display results
            found {[type]} -- stores best match info (minVal, minLoc, scale)
        """
        # unpack the bookkeeping variable and compute the (x, y) coordinates
        # of the bounding box based on the resized ratio
        if found != None:
            (minVal, minLoc, r) = found
            centre = (int((minLoc[0] + tW/2) * r), int((minLoc[1] + tH/2) * r))
            radius = int((tW/2) * r)

            # draw a bounding circle around the detected result and display the image
            cv2.circle(display_image, centre, radius, (0, 255, 0), 2)
            cv2.putText(display_image, "minVal %f" %
                        minVal, (50, 20), 5, 1, (0, 255, 0), 1, cv2.LINE_AA)

    def createMask(image, threshold_low, threshold_high):
            """[summary]
            
            Arguments:
                image {[type]} -- [description]
                threshold_low {[type]} -- [description]
                threshold_high {[type]} -- [description]
            """
            hue, _, _ = cv2.split(image)
            mask = cv2.inRange(hue, threshold_low, threshold_high)
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            # cv2.imshow("mask", mask)
            return mask

    """
    Main code for multiScale template matching method
    """
    # create object to store potential ball locations
    ball_locations = ballLocations()
    found = (None, None, None)

    # cv2.imshow("test", image)
    mask = createMask(image, 20, 65)
    res = cv2.bitwise_and(image, image, mask= mask)
    # cv2.imshow("res", res)

    # loop over the scales of the image
    for scale in np.linspace(scale_range[0], scale_range[1], scale_range[2])[::-1]:
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized, r = resizeImage(res, scale)

        (tH, tW) = template.shape[:2]
        if resizeError(resized, (tH, tW)):
            break

        _, (minVal, minLoc) = findPotentialLocations(display_image, resized, template,
                                                     ball_locations,
                                                     scale_ratio=r, threshold=threshold)

        # if we have found a new min correlation value, then update
        # the bookkeeping variable
        found = updateMinLocation(found, minVal, minLoc, r)

    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    drawMinLoc(display_image, found)
    
    accumulator = createAccumulator(display_image, image, ball_locations)

    return accumulator, ball_locations, found


"""
Real time tennis ball detection
"""
if __name__ == "__main__":
    directory = sys.path[0]
    # get HSV template of tennis ball
    filename = directory + '/tennisBallTemplate.jpg'
    tH, tW, hsv_template = getTemplate(filename, width=24)

    # start camera
    cam = cv2.VideoCapture(-1)
    should_run = True
    should_record = False
    threshold_val = "minVal"
    frame_width = 960

    if should_record:
        ret, image = cam.read()
        image = imutils.resize(image, width=960)
        frame_height = image.shape[0]
        out = cv2.VideoWriter(directory + '/videos/output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (frame_width,frame_height))

    # Instructions
    print "(OPTION 1) Press 's' key when there is no tennis ball present"
    print "(OPTION 2) Press the 't' key to set the threshold while a tennis ball is present (& it has the green circle over it"
    print "Press the 'r' key to reset"


    while(should_run == True):
        # retrieve frame from camera
        ret, image = cam.read()

        # wait for q to exit and release camera
        key = cv2.waitKey(1) & 0xFF
        if checkEndKey(key):
            should_run = False
            break

        # check if camera returned a frame
        if (ret == False):
            print "No image taken"
            continue

        # resize image to a known width but retain aspect ratio
        image = imutils.resize(image, width=960)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # cv2.imshow("Image HSV", hsv_image)

        # Do template matching
        accumulator, ball_locations, (minVal, _, _) = multiScaleTemplateMatching(
            display_image=image, image=hsv_image, template=hsv_template,
            scale_range=(0.5,2.5,3), threshold=threshold_val)
        
        if key == ord("s"):
            # press when no ball is present to set upper limit
            threshold_val = str(minVal*0.75)
            print "Tennis ball threshold value = %s" % threshold_val
            print "No ball threshold value = %f" % minVal

        # if key == ord("t"):
        #     # set threshold value while showing a tennis ball to identify when a ball is not present in current lighting
        #     try:
        #         threshold_val = str(minVal * 2)
        #         print "Tennis ball threshold value = %s" % threshold_val
        #     except:
        #         print "Please set no ball threshold"
        elif key == ord("r"):
            #reset threshold value
            threshold_val = "minVal"

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        try:
            cv2.putText(image, "Detected Balls = %d" %
                        len(accumulator.keypoints), (20, 100), 5, 2, (0, 255, 0), 2, cv2.LINE_AA)
            image = cv2.drawKeypoints(image, accumulator.keypoints, np.array(
                []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        except:
            # print "No keypoints"
            
            pass

        # cv2.imshow("Image", image)
        out.write(image)
    
    cv2.destroyAllWindows()
    cam.release()
    out.release()
