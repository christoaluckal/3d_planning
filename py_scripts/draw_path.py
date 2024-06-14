import numpy as np
import matplotlib.pyplot as plt
import cv2

def drawPath(file_str="",img="landscape.ppm"):

    im_copy = cv2.imread(img)


    with open(file_str, "r") as f:
        lines = f.readlines()

    lines = lines[1:]
    for i in range(len(lines)):
        try:
            lines[i] = lines[i].split()
            row = float(lines[i][1][1:])
            col = float(lines[i][2][:-1])
            row = int(row)
            col = int(col)

            im_copy[row][col] = [0,0,255]
        except:
            pass

    # resize image to 3x
    im_copy = cv2.resize(im_copy, (0,0), fx=3, fy=3)

    cv2.imshow("Path", im_copy)
    cv2.waitKey(0)

drawPath("/home/caluckal/Developer/summer2024/3d_planning/cpp/path.txt")