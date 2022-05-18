import numpy as np
import cv2


def is_PtInRectangle(test_pt, clearance, pt1, pt2, pt3, pt4):
    x, y = test_pt
    x_lst = [pt1[0]-clearance, pt2[0]+clearance, pt3[0]+clearance, pt4[0]-clearance]
    y_lst = [pt1[1]+clearance, pt2[1]+clearance, pt3[1]-clearance, pt4[1]-clearance]
    max_x, min_x = max(x_lst), min(x_lst)
    max_y, min_y = max(y_lst), min(y_lst)
    if min_x <= x <= max_x and min_y <= y <= max_y:
        return True
    else:
        return False

def is_ObstacleSpace(u, v, clearance):
    psn = (u, v)
    if (0 + clearance <= u <= 400 - clearance) and (0 + clearance <= v <= 200 - clearance) and \
        (is_PtInRectangle(psn, clearance, (142.5, 107.5), (157.5, 107.5), (157.5, 92.5), (142.5, 92.5)) == False)  and \
        (is_PtInRectangle(psn, clearance, (192.5, 130), (207.5, 130), (207.5, 115), (192.5, 115)) == False) and \
        (is_PtInRectangle(psn, clearance, (192.5, 85), (207.5, 85), (207.5, 70), (192.5, 70)) == False) and \
        (is_PtInRectangle(psn, clearance, (242.5, 107.5), (257.7, 107.5), (257.7, 92.5), (242.5, 92.5)) == False):
        return False
    else:
        return True
    
# Creates the obstacle space visualization
def get_playground(playground, clearance = 0):
    breadth, length, _ = playground.shape
    for b in range(breadth):
        for l in range(length):
            psn = (l, b)
            if not ((is_PtInRectangle(psn, clearance, (142.5, 107.5), (157.5, 107.5), (157.5, 92.5), (142.5, 92.5)) == False)  and \
                (is_PtInRectangle(psn, clearance, (192.5, 130), (207.5, 130), (207.5, 115), (192.5, 115)) == False) and \
                (is_PtInRectangle(psn, clearance, (192.5, 85), (207.5, 85), (207.5, 70), (192.5, 70)) == False) and \
                (is_PtInRectangle(psn, clearance, (242.5, 107.5), (257.7, 107.5), (257.7, 92.5), (242.5, 92.5)) == False)):
                playground[breadth - b, l] = [255, 0, 0]
    return playground