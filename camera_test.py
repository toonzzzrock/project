# -*- coding: utf-8 -*-
"""
Created on Sun Mar 13 17:58:01 2022

@author: WIN 10 OS
"""

import cv2
import Lane_detect_2 as ld
a = [[204, 198], [466, 198], [635, 351], [23, 333]]

cam = cv2.VideoCapture(0, cv2.CAP_DSHOW) 
cv2.destroyAllWindows()
cv2.namedWindow("test")

img_counter = 0

def point(img):
    for i,j in a:
        for it in range(-3,3):
            for jt in range(-3,3):
                img[j+jt][i+it] = [255,0,0]
    return img        
i = 0
while True:
    ret, frame = cam.read()
    
    cv2.imshow("test", point(frame.copy()))

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    
    elif k%256 == 32:
        # SPACE pressed
        if len(a) == 0:
            img_name = "opencv_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1
        else:
            try:
                ld.main(frame)
            except TypeError:
                pass
    '''
    if len(a) == 0:
            img_name = "opencv_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1
    else:
        try:
            ld.main(frame)
        except TypeError:
            pass'''
    print(i)
    i+=1
    if i == 100:
        break
    

cam.release()

cv2.destroyAllWindows()