"""
Morphology Image Processing
(Due date: Nov. 26, 11:59 P.M., 2021)

The goal of this task is to experiment with commonly used morphology
binary image processing techniques. Use the proper combination of the four commonly used morphology operations, 
i.e. erosion, dilation, open and close, to remove noises and extract boundary of a binary image. 
Specifically, you are given a binary image with noises for your testing, which is named 'task3.png'.  
Note that different binary image might be used when grading your code. 

You are required to write programs to: 
(i) implement four commonly used morphology operations: erosion, dilation, open and close. 
    The stucturing element (SE) should be a 3x3 square of all 1's for all the operations.
(ii) remove noises in task3.png using proper combination of the above morphology operations. 
(iii) extract the boundaries of the objects in denoised binary image 
      using proper combination of the above morphology operations. 
Hint: 
• Zero-padding is needed before morphology operations. 

Do NOT modify the code provided to you.
You are NOT allowed to use OpenCV library except the functions we already been imported from cv2. 
You are allowed to use Numpy libraries, HOWEVER, 
you are NOT allowed to use any functions or APIs directly related to morphology operations.
Please implement erosion, dilation, open and close operations ON YOUR OWN.
"""

from cv2 import imread, imwrite, imshow, IMREAD_GRAYSCALE, namedWindow, waitKey, destroyAllWindows
import numpy as np


def morph_erode(img):
    img = np.where(img==0,img,1)
    erd_img = np.zeros((img.shape))
    kernel = np.ones((3,3),np.int32)
    padded = np.pad(array=img,pad_width=1,mode='constant')

    for i in range(padded.shape[1]):
        for j in range(padded.shape[0]):
            img_window = padded[i:(i+3),j:(j+3)]
            if np.array_equal(img_window,kernel):
                erd_img[i][j] = 1
    
    return erd_img
    """
    :param img: numpy.ndarray(int or bool), image
    :return erode_img: numpy.ndarray(int or bool), image, same size as the input image

    Apply mophology erosion on input binary image. 
    Use 3x3 squared structuring element of all 1's. 
    """

    # TO DO: implement your solution here
    raise NotImplementedError
    return erode_img


def morph_dilate(img):
    img = np.where(img==0,img,1)
    dil_img = np.zeros((img.shape))
    padded = np.pad(array=img,pad_width=1,mode='constant')

    for i in range(img.shape[1]):
        for j in range(img.shape[0]):
            img_win = img[i:(i+3),j:(j+3)]
            if 1 in img_win:
                dil_img[i][j] = 1

    return dil_img
    """
    :param img: numpy.ndarray(int or bool), image
    :return dilate_img: numpy.ndarray(int or bool), image, same size as the input image

    Apply mophology dilation on input binary image. 
    Use 3x3 squared structuring element of all 1's. 
    """

    # TO DO: implement your solution here
    raise NotImplementedError
    return dilate_img


def morph_open(img):
    erd_img = morph_erode(img)
    img_open = morph_dilate(erd_img)

    return img_open

    """
    :param img: numpy.ndarray(int or bool), image
    :return open_img: numpy.ndarray(int or bool), image, same size as the input image

    Apply mophology opening on input binary image. 
    Use 3x3 squared structuring element of all 1's. 
    You can use the combination of above morph_erode/dilate functions for this. 
    """

    # TO DO: implement your solution here
    raise NotImplementedError
    return open_img


def morph_close(img):
    dil_img = morph_dilate(img)
    img_close = morph_erode(dil_img)
    
    return img_close
    """
    :param img: numpy.ndarray(int or bool), image
    :return close_img: numpy.ndarray(int or bool), image, same size as the input image

    Apply mophology closing on input binary image. 
    Use 3x3 squared structuring element of all 1's. 
    You can use the combination of above morph_erode/dilate functions for this. 
    """

    # TO DO: implement your solution here
    raise NotImplementedError
    return close_img


def denoise(img):
    img_open = morph_open(img)
    denoise = morph_close(img_open)
    denoise = np.where(denoise==0,denoise,255)

    return denoise
    """
    :param img: numpy.ndarray(int), image
    :return denoise_img: numpy.ndarray(int), image, same size as the input image

    Remove noises from binary image using morphology operations. 
    If you convert the dtype of input binary image from int to bool,
    make sure to convert the dtype of returned image back to int.
    """

    # TO DO: implement your solution here
    raise NotImplementedError
    return denoise_img


def boundary(img):
    erd_img = morph_erode(img)
    bound_img = img - erd_img

    return bound_img
    """
    :param img: numpy.ndarray(int), image
    :return denoise_img: numpy.ndarray(int), image, same size as the input image

    Extract boundaries from binary image using morphology operations. 
    If you convert the dtype of input binary image from int to bool,
    make sure to convert the dtype of returned image back to int.
    """

    # TO DO: implement your solution here
    raise NotImplementedError
    return bound_img


if __name__ == "__main__":
    img = imread('task3.png', IMREAD_GRAYSCALE)
    denoise_img = denoise(img)
    imwrite('results/task3_denoise.jpg', denoise_img)
    bound_img = boundary(denoise_img)
    imwrite('results/task3_boundary.jpg', bound_img)





