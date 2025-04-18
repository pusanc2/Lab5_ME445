#!/usr/bin/env python3

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = 760.0
tx = 0.20
ty = 0.07

# Function that converts image coord to world coord
def IMG2W(col, row):
    o_r = 240
    o_c = 320
    x_c = (row - o_r) / beta
    y_c = (col - o_c) / beta
    x_c2 = x_c + tx
    y_c2 = y_c + ty
    x_w = x_c2 * np.cos(theta) - y_c2 * np.sin(theta)
    y_w = x_c2 * np.sin(theta) + y_c2 * np.cos(theta)
    
    return (x_w, y_w)

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False
    params.blobColor = 255

    # Filter by Area.
    params.filterByArea = True
    # params.minArea = 300
    params.minArea = 150
    params.maxArea = 2000

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.2

    # Filter by Inerita
    params.filterByInertia = False
    params.minInertiaRatio = 0.8
    params.maxInertiaRatio = 1.0
    
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.75

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # lower = (110,50,50)     # blue lower
    # upper = (130,255,255)   # blue upper
    # if color == "orange":
    #     lower = (5, 100, 100)
    #     upper = (20, 255, 255)
    # # elif color == "green":
    # #     lower = (50, 125, 100)
    # #     upper = (70, 240, 200)
    # elif color == "green":
    #     lower = (40, 40, 40)
    #     upper = (80, 255, 255)
    # elif color == "yellow":
    #     lower = (20, 80, 80)
    #     upper = (35, 255, 255)
    # elif (color == "blue"):    
    #     lower = (0,0,50)     # yellow lower
    #     upper = (130,255,180)   # yellow upper
    
    # else:
    #     raise ValueError("Unsupported color passed")
    if (color == "green"):
        lower = (40,50,50)   # green lower
        upper = (85,255,255)   # green upper
    # elif (color == "blue"):    
    #     lower = (0,0,50)     # yellow lower
    #     upper = (130,255,180)   # yellow upper
    elif color == "yellow":
        lower = (10, 100, 100)
        upper = (35, 255, 255)    
    else:
        lower = (0, 0, 0)
        upper = (0, 0, 0)

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
