###############
##Design the function "findRotMat" to  return 
# 1) rotMat1: a 2D numpy array which indicates the rotation matrix from xyz to XYZ 
# 2) rotMat2: a 2D numpy array which indicates the rotation matrix from XYZ to xyz 
#It is ok to add other functions if you need
###############

import numpy as np
import cv2

def findRotMat(alpha, beta, gamma):
    alpha = np.radians(alpha)
    beta = np.radians(beta)
    gamma = np.radians(gamma)

    R_1 = np.array([[np.cos(alpha),np.sin(alpha),0],[-np.sin(alpha),np.cos(alpha),0],[0,0,1]])
    R_2 = np.array([[1,0,0],[0,np.cos(beta),np.sin(beta) ],[0,-np.sin(beta),np.cos(beta)]])
    R_3 = np.array([[np.cos(gamma),np.sin(gamma),0],[-np.sin(gamma),np.cos(gamma),0],[0,0,1]])
    rotMat1 = R_3.dot(R_2).dot(R_1)

    R_1 = np.array([[np.cos(-alpha),np.sin(-alpha),0],[np.sin(-alpha),np.cos(-alpha),0],[0,0,1]])
    R_2 = np.array([[1,0,0],[0,np.cos(-beta),np.sin(-beta) ],[0,-np.sin(-beta),np.cos(-beta)]])
    R_3 = np.array([[np.cos(-gamma),np.sin(-gamma),0],[-np.sin(-gamma),np.cos(-gamma),0],[0,0,1]])
    rotMat2 = R_1.dot(R_2).dot(R_3)
    return rotMat1, rotMat2



if __name__ == "__main__":
    alpha = 45
    beta = 30
    gamma = 60
    rotMat1, rotMat2 = findRotMat(alpha, beta, gamma)
    print(rotMat1)
    print(rotMat2)
