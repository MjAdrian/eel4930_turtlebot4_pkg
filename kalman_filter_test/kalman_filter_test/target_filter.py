"""
    Author: Aditya Ramesh
    Detects best fitting ellipse of the target 
    
    Sources:
        https://arxiv.org/pdf/0912.3589.pdf 
"""

import numpy as np
import cv2

def filter_by_brightness2(frame):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray_frame, 150, 200, cv2.THRESH_BINARY_INV)
    connectivity = 8
    analysis = cv2.connectedComponentsWithStats(threshold,
                                                connectivity,
                                                cv2.CV_32S)
    (totalLabels, label_ids, values, centroid) = analysis
    output = np.zeros(gray_frame.shape, dtype="uint8")

    for i in range(1, totalLabels):
        area = values[i, cv2.CC_STAT_AREA]  
    
        if (area > 100) and (area < 10000):
            
            # Labels stores all the IDs of the components on the each pixel
            # It has the same dimension as the threshold
            # So we'll check the component
            # then convert it to 255 value to mark it white
            componentMask = (label_ids == i).astype("uint8") * 255
            
            # Creating the Final output mask
            output = cv2.bitwise_or(output, componentMask)
    
    return output

def filter_by_brightness(frame):
    """Fitlers frame for target's brightness""" 
    # Convert image to grayscale
    # kernel = np.ones((5,5),np.float32)/25
    # blur = cv2.filter2D(frame, cv2.CV_32F, kernel)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Image thresholding
    _, thresh = cv2.threshold(gray_frame, 175, 200, cv2.THRESH_BINARY_INV)
    cv2.imshow('c', thresh)

    # Finds connected regions
    connectivity = 8 # 4 or 8
    analysis = cv2.connectedComponentsWithStats(thresh,
                                                connectivity,
                                                cv2.CV_32S)
    (totalLabels, label_ids, values, centroid) = analysis
    
    # Finds largest connected region
    mask = np.zeros(gray_frame.shape, dtype="uint8")
    for i in range(1, totalLabels):
        area = values[i, cv2.CC_STAT_AREA]  
        if (area > 200) and (area < 80000):
            # Labels stores all the IDs of the components on the each pixel
            # It has the same dimension as the threshold
            # So we'll check the component
            # then convert it to 255 value to mark it white
            componentMask = (label_ids == i).astype("uint8") * 255
            # Creating the Final output mask
            mask = cv2.bitwise_or(mask, componentMask)

    result_w_color = cv2.bitwise_and(frame, frame, mask = mask)
    
    return result_w_color


def filter_by_color(frame):
    """Filters frame for target color"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # lower boundary RED color range values; Hue (0 - 10)
    lower1 = np.array([0,140,0])
    upper1 = np.array([10,255,255])
    
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,140,0])
    upper2 = np.array([180,255,255])
    
    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)
    full_mask = lower_mask + upper_mask

    result_w_color = cv2.bitwise_and(frame, frame, mask = full_mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    result = cv2.dilate(result_w_color, kernel)

    return result

def find_contours_threshold(frame, filtered_frame):
    """Finds contours of frame using image thresholding"""
    filtered_frame_gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(filtered_frame_gray, 100, 255, cv2.THRESH_BINARY)
    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    frame_w_contours = cv2.drawContours(frame, contours, -1, (0,255,0), 1)
    return frame_w_contours, contours, heirarchy

def find_contours_adaptive_threshold(frame, filtered_frame, kernel_size=31, C=25):
    """Finds contours of frame using adaptive image thresholding"""
    filtered_frame_gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(filtered_frame_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, kernel_size, C)
    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    frame_w_contours = cv2.drawContours(frame, contours, -1, (0,255,0), 1, maxLevel=0)
    return frame_w_contours, contours, heirarchy

def find_contours_canny(frame, lower_threshold=50, higher_threshold=100, sobel_size=5, L2_grad=True, kernel_size=13, C=50):
    """Finds contours of frame by finding Canny edges then using adaptive image thresholding"""
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.uint8)
    # get canny edges
    edges = cv2.Canny(image=frame_gray, 
                      threshold1=lower_threshold, 
                      threshold2=higher_threshold, 
                      edges=frame, 
                      apertureSize=sobel_size, 
                      L2gradient=L2_grad)
    # get contours
    thresh = cv2.adaptiveThreshold(edges, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, kernel_size, C)
    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    frame_w_contours = cv2.drawContours(frame, contours, -1, (0,255,0), 1)
    return frame_w_contours, contours, heirarchy

def find_contours_connected_regions(frame):
    """Finds contours of frame using image thresholding then finding the largest connected region"""
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(frame_gray, 50, 150, cv2.THRESH_BINARY_INV)

    connectivity = 8 # 4 or 8
    analysis = cv2.connectedComponentsWithStats(thresh,
                                                connectivity,
                                                cv2.CV_32S)
    (totalLabels, label_ids, values, centroid) = analysis
    
    output = np.zeros(frame_gray.shape, dtype="uint8")
    for i in range(1, totalLabels):
        area = values[i, cv2.CC_STAT_AREA]  
        if (area > 100) and (area > 40000):
            # Labels stores all the IDs of the components on the each pixel
            # It has the same dimension as the threshold
            # So we'll check the component
            # then convert it to 255 value to mark it white
            componentMask = (label_ids == i).astype("uint8") * 255
            # Creating the Final output mask
            output = cv2.bitwise_or(output, componentMask)

    contours, heirarchy = cv2.findContours(output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    frame_w_contours = cv2.drawContours(frame, contours, -1, (0,255,0), 1)
    return frame_w_contours, contours, heirarchy

def find_target(frame, contours, heirarchy):
    "Finds the largest ellipse given a frame and its contours"
    ellipses = np.array([[(0,0), (0,0), 0]]*len(contours), dtype=object)
    image_w_ellipse = frame
    ellipse_w_max_area = None
    max_area = 0

    # Find ellipses
    for i, c in enumerate(contours):
        if c.shape[0] > 5:
            h = heirarchy[0, i, :]
            if h[3] != -1:
                ellipses[i] = cv2.fitEllipse(c)
                # image_w_ellipse = cv2.ellipse(frame, ellipses[i], (0,255,255), 2)
            elif h[2] == -1:
                ellipses[i] = cv2.fitEllipse(c)
                # image_w_ellipse = cv2.ellipse(frame, ellipses[i], (0,255,255), 2)

    # Get largest ellipse          
    if ellipses.size > 3:
        ellipse_axis = ellipses[:, 1]
        ellipse_area = np.empty(ellipse_axis.shape)
        for i, (Ma, mb) in enumerate(ellipse_axis):
            ellipse_area[i] = np.pi * Ma * mb
        
        if np.max(ellipse_area) > 500: #and np.max(ellipse_area) < 2500000:
            ellipse_w_max_area = ellipses[np.argmax(ellipse_area)]
            image_w_ellipse = cv2.ellipse(frame, ellipse_w_max_area, (0,255,0), 2)
            max_area = np.max(ellipse_area)
    
    return image_w_ellipse, ellipse_w_max_area, max_area