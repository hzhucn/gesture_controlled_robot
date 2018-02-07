import cv2

def threshold(img_rgb, seuil=None):
    if seuil == None :
        seuil=15
    for i in range(len(img_rgb)):
        for j in range(len(img_rgb[i])):
            r = img_rgb[i][j][0]
            g = img_rgb[i][j][1]
            b = img_rgb[i][j][2]
            if (max(r,g,b)- min(r,g,b) < seuil) or (max(r,g,b)!=r):
                img_rgb[i][j]=[0,0,0]
    return img_rgb

def get_contour(img_rgb):
    img_filtered = cv2.medianBlur(img_rgb, 1) ## Median blur to 
    
    gray = cv2.cvtColor(img_filtered,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0) # Gaussian blur to 
    
    
    _,thresh2 = cv2.threshold(blur,20,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    _, contours, hierarchy = cv2.findContours(thresh2, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    ci = None
    for i in range(len(contours)):
        cnt=contours[i]
        area = cv2.contourArea(cnt)
        if(area>max_area):
            max_area=area
            ci=i
    hand_contour=contours[ci]
    
    return hand_contour