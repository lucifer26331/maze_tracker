import cv2 as cv
import numpy as np
import random
from collections import deque

# Image generation
mat = np.full((10, 10), 255, dtype=np.uint8)
width, length = mat.shape

# Using random we can assign black pixels randomly througout the maze
for i in range(int(0.2 * width * length)):
    x = random.randint(0, width - 1)
    y = random.randint(0, length - 1)
    mat[x, y] = 0

# Fixing the start and end point
mat[2, 2] = 127
mat[8, 8] = 127

# Resizing the maze
mat1 = np.full((100, 100), 0, dtype=np.uint8)
img = cv.resize(mat, (100, 100), mat1, interpolation=cv.INTER_AREA)

# Setting two points at a reasonable distance to be start and end
start = (20, 20)
end = (80, 80)

# We have done this to obtain a single grey start and end pixel
for i in range(20, 30):
    for j in range(20, 30):
        img[i][j] = 255

for i in range(80, 90):
    for j in range(80, 90):
        img[i][j] = 255

img[start] = 127
img[end] = 127

# COLORS
green = (0, 255, 0)
orange = (0, 215, 255)
pink = (255, 0, 255)

# Displaying the maze
cv.namedWindow("Maze", cv.WINDOW_NORMAL)
cv.imshow("Maze", img)
cv.waitKey(0)


# Auxiliary functions

# Using class makes it easier to implement a-star algorithm
class Node:
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        self.g = np.inf
        self.h = np.inf
        self.f = np.inf


# This function helps in finding the minimum f value required in a-star algorithm

def get_min_dist_node(open_list):
    min_dist = np.inf
    min_node = None
    for node in open_list:
        if open_list[node].f < min_dist:
            min_dist = open_list[node].f
            min_node = open_list[node]
    return min_node


# Used for calculating the euclidean distance
def calc_dist(point, current):
    return (point[0] - current[0]) ** 2 + (point[1] - current[1]) ** 2


# This function checks if the given point is inside the maze
def in_bound(img, x, y):
    return 0 <= x < img.shape[0] and 0 <= y < img.shape[1]


# This function displays the final path as well as keeps track of number of steps required by the algorithm
def show_path(img, end, start, parent):
    path = []
    new = np.copy(img)

    # This helps in displaying the path from start to end and also helps in storing number of steps
    while end != start:
        var = int(parent[end][0]), int(parent[end][1])
        path.append(var)
        end = (var[0], var[1])

    path.reverse()

    for i in path:
        new[i] = green
        cv.namedWindow('final image', cv.WINDOW_NORMAL)
        cv.imshow('final image', new)
        cv.waitKey(1)
    cv.destroyAllWindows()
    return len(path)


# This function checks if the given position is an obstacle i.e. is it a black pixel
def obstacle(position):
    x, y = position
    if img[y][x][0] == 0 and img[y][x][1] == 0 and img[y][x][2] == 0:
        return True
    return False


# Breadth first search

def bfs(img, start, end):
    width, height = img.shape

    # The maze is grayscale and for more attractive path display we have converted it to color
    new = np.copy(img)
    new = cv.cvtColor(new, cv.COLOR_GRAY2BGR)

    # parent stores the parent node and visited stores the information about a pixel being visited before
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
            steps = show_path(new, current, start, parent)
            return steps

        for i in ((0, -1), (1, 0), (0, 1), (-1, 0), (-1, -1), (-1, 1), (1, 1), (1, -1)):
            point = (current[0] + i[0], current[1] + i[1])
            if not in_bound(img, point[0], point[1]):
                continue
            if img[point] == 0:
                continue
            if visited[point]:
                continue
            neighbour.append(point)
            visited[point] = 1
            parent[point] = (current[0], current[1])
            new[point] = orange

            if point == end:
                break

        cv.namedWindow('BFS', cv.WINDOW_NORMAL)
        cv.imshow('BFS', new)
        cv.waitKey(1)


# Depth first search

def dfs(img, start, end):
    width, height = img.shape

    # The maze is grayscale and for more attractive path display we have converted it to color
    new = np.copy(img)
    new = cv.cvtColor(new, cv.COLOR_GRAY2BGR)

    # parent stores the parent node and visited stores the information about a pixel being visited before
    parent = np.zeros((width, height, 2))
    visited = np.zeros((width, height))
    parent[start] = None
    visited[start] = 1

    # for dfs we need to use a stack to store neighbours of a pixel
    s = deque([])
    s.append(start)

    while len(s):
        current = s.pop()
        if current == end:
            steps = show_path(new, current, start, parent)
            return steps
        for i in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            point = (current[0] + i[0], current[1] + i[1])
            if not in_bound(img, point[0], point[1]):
                continue
            if img[point] == 0:
                continue
            if visited[point]:
                continue
            s.append(point)
            visited[point] = 1
            parent[point] = (current[0], current[1])
            new[point] = orange
            if point == end:
                break
        cv.namedWindow('DFS', cv.WINDOW_NORMAL)
        cv.imshow('DFS', new)
        cv.waitKey(1)


# Random Walk

# Random walks are present all around us in nature in form of brownian motion, stock market fluctuation, genetic drift
# this algorithm gives control to the computer to decide it's own path and is bounded by simple rules
# these rules include: not going out of maze, not going into blocked pixels and giving equal probability to all
# neighbouring pixels

def random_walk(img, start, end):
    length, width = img.shape

    # The maze is grayscale and for more attractive path display we have converted it to color
    new = np.copy(img)
    new = cv.cvtColor(new, cv.COLOR_GRAY2BGR)

    # parent stores the parent node and visited stores the information about a pixel being visited before
    parent = np.zeros((length, width, 2))
    visited = np.zeros((length, width))
    current = start
    visited[start] = 1

    while current != end:
        visited[current] = 1
        # This list contains all the neigbours machine can travel to for the current pixel
        viable_neighbours = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                point = (current[0] + i, current[1] + j)
                if in_bound(img, point[0], point[1]) and not visited[point] and img[point] != 0:
                    viable_neighbours.append(point)

        # if there are no available neighbour pixel due to some reason the machine retracts its path till it finds one
        if bool(viable_neighbours):
            next_node = random.choices(viable_neighbours, k=1)[0]
            parent[next_node[0], next_node[1]] = (current[0], current[1])
            current = next_node
            viable_neighbours.clear()
        else:
            next_node = int(parent[current][0]), int(parent[current][1])
            current = next_node
            viable_neighbours.clear()

    steps = show_path(new, current, start, parent)
    return steps


# Greedy best first search


def greedy_best_first(img, start, end):
    width, height = img.shape

    # The maze is grayscale and for more attractive path display we have converted it to color
    new = np.copy(img)
    new = cv.cvtColor(new, cv.COLOR_GRAY2BGR)

    # parent stores the parent node and visited stores the information about a pixel being visited before
    parent = np.zeros((width, height, 2))
    visited = np.zeros((width, height))
    current = start
    visited[start] = 1

    # this list stores the distance of neighbour nodes from end pixel
    neighbour_dist = []
    next_node = None

    while current != end:
        visited[current] = 1
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                point = (current[0] + i, current[1] + j)
                if in_bound(new, point[0], point[1]) and not visited[point] and img[point] != 0:
                    neighbour_dist.append((point, calc_dist(point, end)))  # same as g of a star

        min_dist = neighbour_dist[0][1]
        for points in neighbour_dist:
            if points[1] < min_dist:
                min_dist = points[1]
                next_node = points[0]

        parent[next_node[0], next_node[1]] = (current[0], current[1])
        current = next_node
        neighbour_dist.clear()

    steps = show_path(new, current, start, parent)
    return steps


# Dijkstra algorithm

def dijkstra(img, start, end):
    h, w = img.shape

    # The maze is grayscale and for more attractive path display we have converted it to color
    new = np.copy(img)
    new = cv.cvtColor(new, cv.COLOR_GRAY2BGR)

    # this list stores the distance of all visited pixels from the start pixel
    dist = np.full((h, w), fill_value=np.inf)
    dist[start] = 0

    # parent stores the parent node and visited stores the information about a pixel being visited before
    parent = np.zeros((h, w, 2))
    visited = np.zeros((h, w))
    current = start
    visited[start] = 1

    while current != end:
        visited[current] = 1
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                point = (current[0] + i, current[1] + j)
                if in_bound(new, point[0], point[1]) and not visited[point] and img[point] != 0:
                    if calc_dist(point, current) + dist[current] < dist[point]:
                        dist[point] = calc_dist(point, current) + dist[current]
                        parent[point[0], point[1]] = [current[0], current[1]]
                        new[current] = orange
        min = np.inf
        for i in range(h):
            for j in range(w):
                if min > dist[i, j] and visited[i, j] != 1:
                    min = dist[i, j]
                    current = (i, j)

                    cv.namedWindow('Dijkstra', cv.WINDOW_NORMAL)
                    cv.imshow('Dijkstra', new)
                    cv.waitKey(1)
    steps = show_path(new, current, start, parent)
    return steps


def astar_algorithm(start, end):
    # The maze is grayscale and for more attractive path display we have converted it to color
    new = np.copy(img)
    new = cv.cvtColor(new, cv.COLOR_GRAY2BGR)

    # Open list stores all the pixel to be visited and closed list stores all the pixels which have been visited
    open_list = {}
    closed_list = []
    start_node = Node(None, start)
    start_node.g = 0
    start_node.h = start_node.f = calc_dist(start, end)
    open_list[start] = start_node

    # parent stores the parent node and visited stores the information about a pixel being visited before
    parent = np.zeros((width * 10, length * 10, 2))

    while len(open_list) > 0:
        current_node = get_min_dist_node(open_list)
        new[current_node.position[0]][current_node.position[1]] = orange
        open_list.pop(current_node.position)

        if current_node.position == end:
            steps = show_path(new, current_node.position, start, parent)
            return steps

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if not in_bound(img, node_position[0], node_position[1]):
                continue
            if node_position in closed_list:
                continue
            if img[node_position[0], node_position[1]] == 0:
                continue

            new[node_position[0]][node_position[1]] = pink
            new_node = Node(current_node, node_position)

            parent[int(node_position[0]), int(node_position[1])] = current_node.position

            new_node.g = current_node.g + calc_dist(current_node.position, new_node.position)
            new_node.h = calc_dist(new_node.position, end)
            new_node.f = new_node.g + new_node.h

            if new_node.position in open_list:
                if new_node.g < open_list[new_node.position].g:
                    open_list[new_node.position] = new_node
            else:
                open_list[new_node.position] = new_node

        if current_node.position not in closed_list:
            closed_list.append(current_node.position)

        cv.namedWindow('path_finding', cv.WINDOW_NORMAL)
        cv.imshow("path_finding", new)
        cv.waitKey(1)


def main():
    # Get tick count method helps get the number of tick required to carry out the algorithm
    tick1 = cv.getTickCount()
    bfs_steps = bfs(img, start, end)
    tick2 = cv.getTickCount()

    dfs_steps = dfs(img, start, end)
    tick3 = cv.getTickCount()

    dijkstra_steps = dijkstra(img, start, end)
    tick4 = cv.getTickCount()

    greedy_best_steps = greedy_best_first(img, start, end)
    tick5 = cv.getTickCount()

    random_walk_steps = random_walk(img, start, end)
    tick6 = cv.getTickCount()

    astar_algorithm_steps = astar_algorithm(start, end)
    tick7 = cv.getTickCount()

    # Get tick frequency helps calculate the number of seconds required to carry out the algorithm
    print("Time for bfs is " + str((tick2 - tick1) / cv.getTickFrequency()) + "s")
    print("Steps for bfs is " + str(bfs_steps))
    print("Time for dfs is " + str((tick3 - tick2) / cv.getTickFrequency()) + "s")
    print("Steps for dfs is " + str(dfs_steps))
    print("Time for dijkstra is " + str((tick4 - tick3) / cv.getTickFrequency()) + "s")
    print("Steps for dijkstra is " + str(dijkstra_steps))
    print("Time for greedy best first search is " + str((tick5 - tick4) / cv.getTickFrequency()) + "s")
    print("Steps for greedy best first search is " + str(greedy_best_steps))
    print("Time for random walk is " + str((tick6 - tick5) / cv.getTickFrequency()) + "s")
    print("Steps for random walk is " + str(random_walk_steps))
    print("Time for a-star algorithm is " + str((tick7 - tick6) / cv.getTickFrequency()) + "s")
    print("Steps for a-star algorithm is " + str(astar_algorithm_steps))

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

