import cv2
import numpy as np
from collections import deque

img=cv2.imread("map_task1.png")
img_copy=img.copy()

#colors
black=(0,0,0)
white=(255,255,255)
red=(0,0,255)
green=(0,255,0)
orange=(0,128,255)
pink=(255,0,255)
blue=(255,0,0)

#start and end points
start=(145,880)  #here, x and y coordinates are interchanged since the system is interchanging them
end=(563,359)

# This function checks if the given point is inside the image
def in_bound(img, x, y):
    return 0 <= x < img.shape[0] and 0 <= y < img.shape[1]

# This function displays the final path
def show_path(img, end, start, parent):
    path = []
    new = np.copy(img_copy)

    # This stores the parent elements of all the points starting from end until it reaches start
    while end != start:
        var = int(parent[end][0]), int(parent[end][1])
        path.append(var)
        end = (var[0], var[1])

    path.reverse()

    for i in path:
        new[i] = orange
    cv2.namedWindow('final image', cv2.WINDOW_NORMAL)
    cv2.imshow('final image', new)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#This function helps to check if a point is black or not(black represents obstacle)
def obstacle(position):
    x, y = position
    if img[y][x][0] == 0 and img[y][x][1] == 0 and img[y][x][2] == 0:
        return True
    return False

def bfs(img, start, end):
    width, height = img.shape[0],img.shape[1]

    new = np.copy(img)

    # parent[x,y] stores the parent node of the point (x,y) and visited[i] tells whether point'i' has been visited or not(visited[i]=0 means not visited)
    parent = np.zeros((width, height, 2))
    visited = np.zeros((width, height))
    parent[start] = None
    visited[start] = 1

    # for bfs we need to use a queue to store neighbours of a pixel
    neighbour = deque([])
    neighbour.append(start)

    while len(neighbour):
        current = neighbour.popleft()

        if current == end:
            show_path(new, current, start, parent)

        for i in ((0, -1), (1, 0), (0, 1), (-1, 0), (-1, -1), (-1, 1), (1, 1), (1, -1)):
            point = (current[0] + i[0], current[1] + i[1])
            if not in_bound(img, point[0], point[1]):
                continue
            if (img[point] == 0).all():
                continue
            if visited[point]:
                continue
            neighbour.append(point)
            visited[point] = 1
            parent[point] = (current[0], current[1])
            new[point] = orange

            if point == end:
                break

bfs(img,start,end)