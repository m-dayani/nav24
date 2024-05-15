import cv2
import numpy as np


def process(image):
    print(image.shape)

    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return "84.34 34.23"


if __name__ == "__main__":
    process(np.zeros((384, 836), np.float32))
