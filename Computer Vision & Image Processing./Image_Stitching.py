

"""
Image Stitching Problem
(Due date: Nov. 26, 11:59 P.M., 2021)

The goal of this task is to stitch two images of overlap into one image.
You are given 'left.jpg' and 'right.jpg' for your image stitching code testing. 
Note that different left/right images might be used when grading your code. 

To this end, you need to find keypoints (points of interest) in the given left and right images.
Then, use proper feature descriptors to extract features for these keypoints. 
Next, you should match the keypoints in both images using the feature distance via KNN (k=2); 
cross-checking and ratio test might be helpful for feature matching. 
After this, you need to implement RANSAC algorithm to estimate homography matrix. 
(If you want to make your result reproducible, you can try and set fixed random seed)
At last, you can make a panorama, warp one image and stitch it to another one using the homography transform.
Note that your final panorama image should NOT be cropped or missing any region of left/right image. 

Do NOT modify the code provided to you.
You are allowed use APIs provided by numpy and opencv, except “cv2.findHomography()” and
APIs that have “stitch”, “Stitch”, “match” or “Match” in their names, e.g., “cv2.BFMatcher()” and
“cv2.Stitcher.create()”.
If you intend to use SIFT feature, make sure your OpenCV version is 3.4.2.17, see project2.pdf for details.
"""

import cv2
import numpy as np
# np.random.seed(<int>) # you can use this line to set the fixed random seed if you are using np.random
import random
# random.seed(<int>) # you can use this line to set the fixed random seed if you are using random

left_img = cv2.imread('./left.jpg')
right_img = cv2.imread('./right.jpg')

def findH(SER, DES):
    B = []
    for i in range(4):
        x, y = SER[i][0], SER[i][1]
        p_x, p_y = DES[i][0], DES[i][1]
        B.append([0, 0, 0, x, y, 1, -p_y * x, -p_y * y, -p_y])
        B.append([x, y, 1, 0, 0, 0, -x * p_x, -p_x * y, -p_x])
    B = np.asarray(B)

    U, S, v = np.linalg.svd(B)
    H = v[8]
    H = H.reshape(3,3)
    H = H * 1/H[2,2]

    return H

def generateRandom(SER_Pts, DES_Pts, N):
  r = np.random.choice(len(SER_Pts), N)
  DES = [DES_Pts[i] for i in r]
  SER = [SER_Pts[i] for i in r]
  return np.asarray(SER, dtype=np.float32), np.asarray(DES, dtype=np.float32)


def ransacHomography(SER_Pts, dst_Pts):
    max_LDES = []
    max_LSER = []
    max_I = 0
    finalH = None

    for i in range(6000):
        SERP, DESP = generateRandom(SER_Pts, dst_Pts, 4)
        H = findH(SERP, DESP)
        inlines = 0
        line_DES = []
        line_SER = []

        for p_1, p_2 in zip(SER_Pts, dst_Pts):
            p_1U = (np.append(p_1, 1)).reshape(3, 1)
            p_2e = H.dot(p_1U)
            p_2e = (p_2e / p_2e[2])[:2].reshape(1, 2)[0]

            if np.linalg.norm(p_2 - p_2e) < 5:
                inlines += 1
                line_DES.append(p_2)
                line_SER.append(p_1)

        if inlines > max_I:
            max_I = inlines
            print(max_I)
            max_LDES = line_DES.copy()
            max_LDES = np.asarray(max_LDES, dtype=np.float32)
            max_LSER = line_SER.copy()
            max_LSER = np.asarray(max_LSER, dtype=np.float32)
            H_final = H

        if max_I > inlines:
            break

    return H_final

rows_1, cols_1 = left_img.shape[:2]
rows_2, cols_2 = right_img.shape[:2]

corners_1 = np.float32([[0,0], [0,rows_1], [cols_1, rows_1], [cols_1,0]]).reshape(-1,1,2)
corners_2 = np.float32([[0,0], [0,rows_2], [cols_2, rows_2], [cols_2,0]]).reshape(-1,1,2)

perspective_corners_1 = cv2.perspectiveTransform(corners_1, H)
corners = np.concatenate((corners_2, perspective_corners_1), axis=0)

[x_min, y_min] = np.int32(corners.min(axis=0).ravel())
[x_max, y_max] = np.int32(corners.max(axis=0).ravel())

TY = -y_min
TX = -x_min
T = np.asarray([[1,0,TX],[0,1,TY],[0,0,1]])
H_T = np.dot(T,H)
im_out = cv2.warpPerspective(left_img,H_T, (x_max-x_min,y_max-y_min))

im_out[TY:TY+rows_2,TX:TX+cols_2] = right_img

def solution(left_img, right_img):
  # create SIFT object
  sift = cv2.xfeatures2d.SIFT_create()

  # detect SIFT features in both images
  keypoints_L, descriptors_L = sift.detectAndCompute(left_img, None)
  keypoints_R, descriptors_R = sift.detectAndCompute(right_img, None)

  # calculating and ranking feature distances for each keypoint
  # right matching
  m_R = []
  md_R=[]

  for i in range(len(descriptors_R)):
    y = np.asarray(descriptors_L)
    x = np.asarray(descriptors_R[i])
    m_R.append(np.linalg.norm(y-x,axis=1))
    c1 = c2 = float('inf')

    for j in m_R[i]:
      if j <= c1:
        c1, c2 = j, c1
      elif j < c2:
        c2 = j
    md_R.append((c1, c2))

  # cross checking the match to ensure the match is good (symmetric)
  # left matching
  m_L = []
  md_L=[]

  for i in range(len(descriptors_L)):
    y = np.asarray(descriptors_R)
    x = np.asarray(descriptors_L[i])
    m_L.append(np.linalg.norm(y-x, axis=1))
    c1 = c2 = float('inf')

    for j in m_L[i]:
      if j <= c1:
        c1, c2 = j, c1
      elif j < c2:
        c2 = j
    md_L.append((c1, c2))

  # creating indexes for variables

  # left index
  ratio = 0.75
  index_L=[]

  for i in range(len(md_L)):
    y = md_L[i][0]/md_L[i][1]
    if y < ratio:
      index_L.append(i)

  # right index
  index_R=[]

  for i in range(len(md_R)):
    y = md_R[i][0]/md_R[i][1]
    if y < ratio:
      index_R.append(i)

  # using keypoints
  fkp_L = []
  for i in index_L:
      fkp_L.append(keypoints_L[i])

  fkp_R = []
  for i in index_R:
      fkp_R.append(keypoints_R[i])

  r_pts = cv2.KeyPoint_convert(fkp_R)
  l_pts = cv2.KeyPoint_convert(fkp_L)

  H = ransacHomography(l_pts, r_pts)

  print(H)
  """
  :param left_img:
  :param right_img:
  :return: you need to return the result panorama image which is stitched by left_img and right_img
  """

  # TO DO: implement your solution here
  # raise NotImplementedError
  # return result_img
    

if __name__ == "__main__":
    left_img = cv2.imread('left.jpg')
    right_img = cv2.imread('right.jpg')
    result_img = solution(left_img, right_img)
    cv2.imwrite('results/task1_result.jpg', result_img)