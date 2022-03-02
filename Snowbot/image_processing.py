import numpy as np
import cv2 as cv2
from matplotlib import pyplot as plt
from math import atan, pi


def main():
    img = read_image('news.jpg')
    show_image(img)
    # try_out_find_angle()


def read_image(img_path):
    """
    :param img_path: Path of the image you want to read
    :return:
    """
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    return img


def show_image(img, display_time=0):
    """
    :param img: The image you want to display
    :param display_time: The amount of time you want to display the image for in ms.
    Note: A value of 0 will display intently until a KEY is pressed.
    """
    cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('image', img)
    cv2.waitKey(display_time)
    cv2.destroyAllWindows()


def write_image(img, img_saving_path):
    """
    :param img: A list or numpy array of image
    :param img_saving_path: The path you want to save image to
    """
    if isinstance(img, list):
        img = np.asarray(img, dtype=np.uint8)
    else:
        raise TypeError("img is neither a list nor a ndarray.")

    cv2.imwrite(img_saving_path, img)


def find_angle_of_pixel(locate_pixel, center_pixel):
    """
    :param locate_pixel: A tuple or list of len 2 with an x and y value representing the position of the pixel in an image
    This is the pixel who's angle relative to the center you'd like to find
    :param center_pixel: A tuple or list of len 2 with an x and y value representing the position of the pixel in an image
    This is the pixel at the center of the image
    :return: The angle of locate_pixel relative to the center_pixel in radians. In the image above the center of the
    circle is north: radian 0, to the right is east: radian pi/2, to the south: radian pi, to the west: radian 3pi/2
    """

    x = float(locate_pixel[0] - center_pixel[0])
    y = float(locate_pixel[1] - center_pixel[1])

    if x == 0 and y < 0:
        return 0 # N
    elif x > 0 and y == 0:
        return pi/2.0 # E
    elif x == 0 and y > 0:
        return pi # S
    elif x < 0 and y == 0:
        return (3*pi)/2.0 # W
    elif x == 0 and y == 0:
        return -1 # center

    angle = atan(x / y)

    if x > 0 and y < 0:
        return -1 * angle # NE Q1
    elif x > 0 and y > 0:
        return (pi/2.0) + angle  # SE Q2
    elif x < 0 and y > 0:
        return pi + (-1 * angle) # SW Q3
    elif x < 0 and y < 0:
        return (2*pi) - angle # NW Q4

    return '???'


def try_out_find_angle():
    print('N:', find_angle_of_pixel((2, 0), (2, 2)))
    print('Q1:', find_angle_of_pixel((3, 1), (2, 2)))
    print('E:', find_angle_of_pixel((4, 2), (2, 2)))
    print('Q2:', find_angle_of_pixel((3, 4), (2, 2)))
    print('S:', find_angle_of_pixel((2, 4), (2, 2)))
    print('Q3:', find_angle_of_pixel((1, 5), (2, 2)))
    print('W:', find_angle_of_pixel((0, 2), (2, 2)))
    print('Q4:', find_angle_of_pixel((1, 1), (2, 2)))


if __name__ == '__main__':
    main()
