#!/usr/bin/env python
import cv2
import numpy as np

ix,iy = -1,-1
image1 = cv2.imread('/home/aniketrs/Downloads/PheenoLedArrayTest.jpg')  # np.zeros((512,512,3), np.uint8)
image = cv2.resize(image1, (800, 600))

# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),3,(255,0,0),1)
        ix,iy = x,y
    elif event == cv2.EVENT_RBUTTONDOWN:
        cv_img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # cv2.imshow('hsv image', cv_img_hsv)
        # print type(cv_img_hsv)
        (hue, sat, value, i) = cv2.cv.Get2D(cv2.cv.fromarray(cv_img_hsv), y, x)
        print 'Hue=%d, Saturation:%d, Value:%d'%(hue, sat, value)

        # red_color = img[y, x, 0]
        # green_color = img[y, x , 1]
        # blue_color = img[y, x, 2]
        # print red_color, green_color, blue_color
        # strRGB = str(red_color) + ',' + str(green_color) + ',' + str(blue_color)
        strHSV = str(hue) + ',' + str(sat) + ',' + str(value)
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        cv2.putText(img, strHSV, (x, y), font, 1, (0, 0, 0), 2)
        cv2.imshow('image', img)
        # norm_red = int(red_color/255)
        # norm_green = int(green_color/255)
        # norm_blue = int(blue_color/255)
        # Cmax = max(norm_red, norm_blue, norm_green)
        # Cmin = min(norm_red, norm_blue, norm_green)
        # delta = Cmax - Cmin
        # if delta is 0:
        #     hue = 0
        #     sat = 0
        # elif:
        #     sat = int(delta/Cmax)
        #     delta_red = (((Cmax - norm_red)/6) + (delta/2))/delta
        #     delta_green = (((Cmax - norm_green)/6) + (delta/2))/delta
        #     delta_blue = (((Cmax - norm_blue)/6) + (delta/2))/delta

def nothing(x):
    pass

# def Blurring(imag, min_thresh, max_thresh):
    # Create a black image, a window and bind the function to window

cv2.namedWindow('Image')
cv2.createTrackbar('Minimum Threshold', 'Image', 0, 255, nothing)
cv2.createTrackbar('Maximum Threshold', 'Image', 0, 255, nothing)


while(1):
    cv2.imshow('Image',image)
    k = cv2.waitKey(20) & 0xFF
    min_thresh = cv2.getTrackbarPos('Minimum Threshold', 'Image')
    max_thresh = cv2.getTrackbarPos('Maximum Threshold', 'Image')
    if k == 27:
        break
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('image', img)

        # cv2.setMouseCallback('image',draw_circle)
        cv2.namedWindow('Blur')
        blur = cv2.bilateralFilter(gray, 5, sigmaColor=100, sigmaSpace=100) #GaussianBlur(gray, (1, 1), 0)  #
        thresh = cv2.threshold(blur, min_thresh, max_thresh, cv2.THRESH_BINARY)[1]
        # blur_rgb = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
        cv2.imshow('Blur', thresh)
        shape = "undefined"
        cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            M = cv2.moments(c)
            # print 'M=%.3f /n'%M["m00"]
            Cx = 200#int(M["m10"] / M["m00"])
            Cy = 300#int(M["m01"] / M["m00"])
            peri = cv2.arcLength(c, True)
            apprx = cv2.approxPolyDP(c, 0.05 * peri, True)
            if len(apprx) is 3:
                shape = "Triangle"
                cv2.putText(image, shape, (Cx-100, Cy-100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 0), 4)
            elif len(apprx) >= 28 and len(apprx) <= 45:
                shape = "arrow"
            cv2.putText(image, shape, (Cx+100, Cy+200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 4)
            # elif len(apprx) is 5:
            #     shape = "pentagon"
            # elif len(apprx) is 4:
            #     (x, y, w, h) = cv2.boundingRect(apprx)
            #     ar = w / float(h)
            #     shape = "square" if ar > 0.95 and ar <= 1.05 else "rectangle"
            # else:
            #     shape = "circle"
            cv2.drawContours(image, [c], 0, (0, 255, 0), 2)
            cv2.putText(image, shape, (Cx, Cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 4)

        # print ix,iy
cv2.destroyAllWindows()