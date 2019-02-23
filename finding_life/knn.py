import numpy as np
from sklearn.neighbors import KNeighborsClassifier
import imutils
from imutils import paths
import cv2
import sys

# known samples
minerals = np.array([[0.1, 0.15, 0.4],
                     [0.2, 0.2, 0.1]])

minerals_names = ['chlorite', 'saponite']


def knn_train(features, labels):
    """
    A KNN classifier is trained using a single feature for each image.
    The index of features should relate to the equivalent index of labels.
    neighbors is used to choose how many features the
    """

    model = KNeighborsClassifier(n_neighbors=1,
                                      metric='euclidean',
                                      n_jobs=-1)

    model.fit(features, labels)

    return(model)

def stack_images(directory):

    imagePaths = list(paths.list_images(directory))

    
    for (i, imagePath) in enumerate(imagePaths):
        image = cv2.imread(imagePath)   # read image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert to grayscale
        image = image/255.  # convert to float
        image = cv2.resize(image, (30,20))  # resize to known size
        cv2.imshow(str(i), image)   # display image

        if i == 0:
            stack = image.copy()    # create stack of images
            continue
        
        stack = np.dstack((stack, image))   # add next image (wavelength) to stack
        
    return stack.reshape(-1, stack.shape[-1])


if __name__ == "__main__":

    # create a test case
    unknown = np.array([[0.1, 0.1, 0.15],
                        [0.12, 0.1, 0.12],
                        [0.2, 0.08, 0.17]])

    # create knn model
    model = knn_train(minerals, minerals_names)
    
    # label = model.predict(unknown)
    stack = stack_images(sys.path[0] + "/test_images")

    label = model.predict(stack)

    print(label)

    cv2.waitKey(0)