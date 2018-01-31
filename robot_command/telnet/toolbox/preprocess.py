import cv2
import numpy as np

def threshold(img_bgr, seuil=None):
    '''
    Much much quicker 
    There are many ways to perform pixel-wise threshold to separate "skin pixels" 
    from "non-skin pixels", and there are papers based on virtually any colorspace (even with RGB). 
    
    They worked with the YCbCr colorspace and got quite nice results, 
    the paper also mentions a threshold that worked well for them
    Face Segmentation Using Skin-Color Map in Videophone Applications by Chai and Ngan
    using http://ieeexplore.ieee.org/document/767122/
    '''
    im_ycrcb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YCR_CB)
    skin_ycrcb_mint = np.array((0, 133, 77))
    skin_ycrcb_maxt = np.array((255, 173, 127))
    mask = cv2.inRange(im_ycrcb, skin_ycrcb_mint, skin_ycrcb_maxt)
    skin_ycrcb = cv2.bitwise_and(image, image, mask = mask)
    
    return skin_ycrcb

def get_contour(img_rgb):
    img_filtered = cv2.medianBlur(img_rgb, 1) ## Median blur to 
    
    blur = cv2.GaussianBlur(img_filtered,(5,5),0) # Gaussian blur to 
    
    _,thresh = cv2.threshold(blur,20,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    ci = None
    for i in range(len(contours)):
        cnt=contours[i]
        area = cv2.contourArea(cnt)
        if(area>max_area):
            max_area=area
            ci=i
    hand_contour=contours[ci]
    
    return hand_contour, thresh
