#! /usr/bin/env python
import os
import sys
import csv
import cv2
import glob
import numpy as np
from math import atan2, degrees, pi
from shapely.geometry import LineString

# to calculate edge and mask most of non-road area in the image   
def calcEdgeAndROI(gray):   
    edges = cv2.Canny(gray,100,110)

    # get the rows and cols of image                      
    i,j = edges.shape

    # mask the image and keep only the required ROI
    for x in range(i/2):
        for y in range(j):
            edges[x][y] = 0
           
    for x in range(i*85/100, i):
        for y in range(j):
            edges[x][y] = 0       
           
    for x in range(i*65/100, i):
        for y in range(j*40/100, j*70/100):
            edges[x][y] = 0

    k = j/3
    start = i/2

    while(k>0):
        for x in range(start, i, 1): 
            for y in range(0, k):
                edges[x][y] = 0
            k -= 2
       
    k = j*2/3
    while(k<=j):
        for x in range(start, i, 1):    
            for y in range(k, j):
                edges[x][y] = 0
            k += 2
    return edges   

''' function to select only range of lane angled lines'''
def selectRadians(lines):
    # store all the above constraint passed radians in a list 
    rads = []
    for rho,theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 10*(-b))
        y1 = int(y0 + 10*(a))
        x2 = int(x0 - 10*(-b))
        y2 = int(y0 - 10*(a))

        radian = theta
        theta = degrees(theta)

        # filter out lines with degrees that are outside ROI        
        if (theta >40 and theta < 60 ):                                
            rads.append(radian)                     
        # filter out lines with degrees that are outside ROI
        if (theta > 110 and theta < 130):   
            rads.append(radian)
    return rads

''' calculate x,y from radian'''   
def calcX1Y1X2Y2(rads):                   
    a = np.cos(rads)
    b = np.sin(rads)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 2000*(-b))
    y1 = int(y0 + 2000*(a))
    x2 = int(x0 - 2000*(-b))
    y2 = int(y0 - 2000*(a))   
    return x1,y1,x2,y2

''' Draw and find intersection for RIGHT LANE'''
def forRight(x1,y1,x2,y2, img):
    cv2.line(img,(x1,y1),(x2,y2),(0,255,255),10)
    line1 = LineString([(x1,y1),(x2,y2)])
    line2 = LineString([(1600,0), (1600, 1200)])
                          
    if not line1.intersection(line2):
        line2 = LineString([(0, 1200), (1600, 1200)])
       
    p = (line1.intersection(line2))                                  
    cv2.circle(img,(int(p.x), int(p.y)),10,(0,255,0),10)                        
    return p.x, p.y, img

''' Draw and find intersection for LEFT LANE'''
def forLeft(x1,y1,x2,y2, img):
    cv2.line(img,(x1,y1),(x2,y2), (255, 255,0),10)   
    line1 = LineString([(x1,y1),(x2,y2)])
    line2 = LineString([(0,0), (0, 1200)])
                                                  
    if not line1.intersection(line2):
        line2 = LineString([(0, 1200), (1600, 1200)])
       
    p = (line1.intersection(line2))                                               
    cv2.circle(img,(int(p.x), int(p.y)),10,(0,255,0),10)
    return p.x, p.y, img
   
   
''' check validity of intercepts and convert into integer'''
def checkAndAddIntercepts(left_x, left_y, right_x, right_y ):
    if(not left_x == None):
        left_x = int(left_x)
    if(not right_x == None):
        right_x = int(right_x)
    if(not left_y == None):
        left_y = int(left_y)
    if(not right_y == None):
        right_y = int(right_y)
    return left_x, left_y, right_x, right_y            

''' write intercepts to file '''
def writeIntercepts(intercepts):          
    #CSV output
    with open('intercepts.csv', 'w') as f:
        writer = csv.writer(f)   
        writer.writerows(intercepts)
       
def video_to_frame(video):
	vidcap = cv2.VideoCapture(video)
	imgs = []
	while vidcap.isOpened():
		success, image = vidcap.read()
		imgs.append(image) 
		if (success == 0):
			break
	return imgs               
   
   
if __name__ == "__main__":

    cv2.namedWindow('Lane Markers', cv2.WINDOW_NORMAL )
    imgs = video_to_frame('movie.mp4')
  
    #imgs = sorted(glob.glob("image/*.png"))
    intercepts = []
   
       
    for img in imgs:
        # Load one image at a time 
        #img  = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
       
        # to calculate edge and mask most of non-road area in the image
        edges = calcEdgeAndROI(gray)   
   
        # find Hough lines
        lines = cv2.HoughLines(edges,3,np.pi/180,180)
       
        # store all the above constraint passed radians in a list 
        rads = selectRadians(lines)
            
        # draw only the min and max of lines, instead of all                               
        left_x, right_x = None, None
        left_y, right_y = None, None
        flag1, flag2 = True, True
       
        #find the best line to the left and right of the vehicle
        if(len(rads) > 0 ):
            for rho,theta in lines[0]:
                # for RIGHT LANE
                if max(rads) == theta and flag1 == True:
                    maxr = max(rads)
                    x1,y1,x2,y2 = calcX1Y1X2Y2(max(rads))
                   
                    if(flag2 == False and maxr-minr > 0.5 or flag2 == True or flag1== False and flag2 == False):
                        right_x, right_y, img = forRight(x1,y1,x2,y2, img)
                        flag1 = False
                       
                # for LEFT LANE
                if min(rads) == theta and flag2 == True:                                   
                    minr = min(rads)
                    x1,y1,x2,y2 = calcX1Y1X2Y2(min(rads))
                   
                    if(flag1 == False and maxr-minr > 0.5 or flag1 == True or flag1== False and flag2 == False):
                        left_x, left_y, img = forLeft(x1,y1,x2,y2, img)
                        flag2 = False       
       
        # show image with detected lanes               
        cv2.imshow('Lane Markers', img)
        cv2.waitKey()
		
        # write image to file
        #cv2.imwrite('outputs/'+fname,img)
        #print(fname, "....done!")
       
        # convert intercepts to integer if not None
        left_x, left_y, right_x, right_y = checkAndAddIntercepts(left_x, left_y, right_x, right_y)
       
        # Append intercepts into a list
        #intercepts.append((os.path.basename(fname), left_x,right_x,left_y, right_y))
       
    # write intercepts to a csv file
    writeIntercepts(intercepts)
           
    # close all image windows          
    cv2.destroyAllWindows()

