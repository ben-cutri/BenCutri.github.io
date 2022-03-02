###############
##Design the function "calibrate" to  return 
# (1) intrinsic_params: should be a list with four elements: [f_x, f_y, o_x, o_y], where f_x and f_y is focal length, o_x and o_y is offset;
# (2) is_constant: should be bool data type. False if the intrinsic parameters differed from world coordinates. 
#                                            True if the intrinsic parameters are invariable.
#It is ok to add other functions if you need
###############

import numpy as np
import cv2 as cv
from cv2 import imread, cvtColor, COLOR_BGR2GRAY, TERM_CRITERIA_EPS, TERM_CRITERIA_MAX_ITER, \
    findChessboardCorners, cornerSubPix, drawChessboardCorners
from numpy.core.function_base import linspace
from numpy.lib.shape_base import expand_dims

def calibrate(imgname):
    # criteria for stopping
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # arrays for storing image and object points from the images.
    imgpoints = []
    objpoints = [] 

    # preparing the object points and image
    objp = np.zeros((4*9,3), np.float32)
    objp[:,:2] = 10*np.mgrid[0:4,0:9].T.reshape(-1,2)

    img = cv.imread('checkboard.png')
    img1 = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # creating chessboard corners
    ret, corners = cv.findChessboardCorners(img1, (4,9), None)

    # when created, add object and image points
    if ret == True: 
        objpoints.append(objp)

        corners_2 = cv.cornerSubPix(img1,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        img_coor = np.squeeze(corners_2)

        # display the corners
        cv.drawChessboardCorners(img, (4,9), corners_2, ret)
        img = cv.resize(img,(1600,900))
        cv.imshow('img', img)
        cv.waitKey(0)
        cv.destroyAllWindows()

    # define world coordinates
    world = np.array([(40,0,40),(40,0,30),(40,0,20),(40,0,10),(30,0,40),(30,0,30),(30,0,20),(30,0,10),(20,0,40),(20,0,30),(20,0,20),
            (20,0,10),(10,0,40),(10,0,30),(10,0,20),(10,0,10),(0,0,40),(0,0,30),(0,0,20),(0,0,10),(0,10,40),(0,10,30),(0,10,20),
            (0,10,10),(0,20,40),(0,20,30),(0,20,20),(0,20,10),(0,30,40),(0,30,30),(0,30,20),(0,30,10),(0,40,40),(0,40,30),(0,40,20),
            (0,40,10)])

    # create the combinded matrix and solve with singular value decomp (svd)
    X = np.zeros((72,12), np.float32)
    X[::2, 3] = 1
    X[1::2, 7] = 1
    X[:, 11] = 1

    for i in range(len(world)):
        for j in range(0,len(X)):
            if  j%2==0:
                X[j,:3] = world[i]
                X[j,8:11] = world[i]

            else :
                X[j,4:7] = world[i]
                X[j,8:11] = world[i]
                i+=1
        break

    for p in range(len(img_coor)):
        for q in range(0,len(X)):
            if  q%2==0:
                k = img_coor[p][0]
                X[q,8:12] *= -k

            else :
                h = img_coor[p][1]
                X[q,8:12] *= -h
                p+=1
        break

    a,b,c = np.linalg.svd(X)
    d = c[11]
    d = d.reshape(3,4)
    lamda = np.linalg.norm(d[2,:3])
    d = 1/lamda*d
    print(d)

    f1 = d[0,:3]
    f2 = d[1,:3]
    f3 = d[2,:3]

    ox = (f1.T).dot(f3)
    oy = (f2.T).dot(f3)
    fx = np.sqrt((f1.T).dot(f1) - ox*ox) 
    fy = np.sqrt((f2.T).dot(f2) - oy*oy)
    intrinsic_params = []
    intrinsic_params.append(ox)
    intrinsic_params.append(oy)
    intrinsic_params.append(fx)
    intrinsic_params.append(fy)
    return intrinsic_params, True
   
if __name__ == "__main__":
    intrinsic_params, is_constant = calibrate('checkboard.png')
    print(intrinsic_params)
    print(is_constant)