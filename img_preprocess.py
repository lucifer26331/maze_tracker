import cv2 
import numpy as np
from collections import deque

#reading and processing of map1
frame1 = cv2.imread('map1.png')
gframe1=cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
n1,l1=gframe1.shape
blurg1=cv2.GaussianBlur(gframe1,(3,3),0)
for i in range(n1):
    for j in range(l1):
        if blurg1[i,j] !=255:
            blurg1[i,j] = 0
img1=cv2.cvtColor(blurg1,cv2.COLOR_GRAY2BGR)


#reading and processing of map2
frame2 = cv2.imread('map2.png')
gframe2=cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
n2,l2=gframe2.shape
blurg2=cv2.GaussianBlur(gframe2,(3,3),0)
for i in range(n2):
    for j in range(l2):
        if blurg2[i,j] !=255:
            blurg2[i,j] = 0
img2=cv2.cvtColor(blurg2,cv2.COLOR_GRAY2BGR)

#reading and processing of map3
frame3 = cv2.imread('map3.png')
gframe3=cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)
n3,l3=gframe3.shape
blurg3=cv2.GaussianBlur(gframe3,(3,3),0)
for i in range(n3):
    for j in range(l3):
        if blurg3[i,j] !=255:
            blurg3[i,j] = 0
img3=cv2.cvtColor(blurg3,cv2.COLOR_GRAY2BGR)

#colors
black=(0,0,0)
white=(255,255,255)
red=(0,0,255)
green=(0,255,0)
orange=(0,128,255)
pink=(255,0,255)
blue=(255,0,0)
grey=(127,127,127)

#start and end 
start1=(4,1037)
end1=(476,168)
start2=(109,1194)
end2=(557,412)
start3=(75,1156)
end3=(427,33)
img1[4,1037]= red       #start=red
img1[476,168]= green    #end=green
img2[109,1194]=red
img2[557,412]= green
img3[75,1156]= red
img3[427,33]= green

#Multiple copies of original images are made to avoid corruption of original image
#Copies of img1
img1_copy=img1.copy()
img1_copy2=img1.copy()
img1_copy3=img1.copy()
img1_copy4=img1.copy()
img1_copy5=img1.copy()
img1_copy6=img1.copy()

#Copies of img2
img2_copy=img2.copy()
img2_copy2=img2.copy()
img2_copy3=img2.copy()
img2_copy4=img2.copy()
img2_copy5=img2.copy()
img2_copy6=img2.copy()

#Copies of img3
img3_copy=img3.copy()
img3_copy2=img3.copy()
img3_copy3=img3.copy()
img3_copy4=img3.copy()
img3_copy5=img3.copy()
img3_copy6=img3.copy()

#Helping Functions
class Node():
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        self.g = np.inf
        self.h = np.inf
        self.f = np.inf

#This function checks if a point is black or not(black represents obstacle)
def isObstacle(p,img1):
    x,y=p
    if img1[x,y][0]==black[0] and img1[x,y][1]==black[1] and img1[x,y][2]==black[2]:
        return True
    return False

#This function is used to show the final path
def show_path(node,img1_copy):
    print('show path')
    current_node = node
    path = []
    while current_node is not None:
        path.append((current_node.position[1],current_node.position[0]))
        current_node = current_node.parent
    path.reverse()
    for i in range(len(path)-1):
        cv2.line(img1_copy,path[i], path[i+1], blue, 2)
    cv2.namedWindow('final', cv2.WINDOW_NORMAL)
    cv2.imshow("final", img1_copy)
    cv2.waitKey(0)
    print("Final image shown")

#This function helps to find the node with minimum value of f
#Used in astar
def get_min_dist_node(open_list):
    min_dist = np.inf
    min_node = None
    for node in open_list:
        if open_list[node].f < min_dist:
            min_dist = open_list[node].f
            min_node = open_list[node]
    return min_node

#This function is used to find the eucledian distance between two points
def calcDist(point,current):
    return (point[0] - current[0])**2 + (point[1] - current[1])**2

#Breadth First Search
#Breadth First Search finds the shortest path
#It uses queue
#basic algo
#while(queue not empty):
#   pop
#   visited
#   traverse
def bfs(start,img1,img1_copy):
    print("bfs start")  
    q=deque() 
    q.append(start)     
    while len(q): 
        current=q.popleft() 
        i,j=current.position[0], current.position[1]

        if (i+1)<img1.shape[0]:
            if not isObstacle((i+1,j),img1) and (img1[i+1,j]!=grey).any():
                if (img1[i+1,j]==green).all():
                    break                    #reached destination
                img1[i+1,j]=grey             #visited node
                n=Node(current,(i+1,j))      #current node becomes the parent node
                q.append(n)
        if (j+1)<img1.shape[1]:
            if not isObstacle((i,j+1),img1) and (img1[i,j+1]!=grey).any():
                if (img1[i,j+1]==green).all():
                    break
                img1[i,j+1]=grey
                n=Node(current,(i,j+1))
                q.append(n)
        if (i-1)>0:
            if not isObstacle((i-1,j),img1) and (img1[i-1,j]!=grey).any():
                if (img1[i-1,j]==green).all():
                    break
                img1[i-1,j]=grey
                n=Node(current,(i-1,j))
                q.append(n)
        if (j-1)>0:
            if not isObstacle((i,j-1),img1) and (img1[i,j-1]!=grey).any():
                if (img1[i,j-1]==green).all():
                    break
                img1[i,j-1]=grey
                n=Node(current,(i,j-1))
                q.append(n)
        
    show_path(current,img1_copy)

#Implementing bfs
print("BFS for img1")
bfs(Node(None,(start1[0],start1[1])),img1_copy,img1_copy2)
print("BFS for img2")
bfs(Node(None,(start2[0],start2[1])),img2_copy,img2_copy2)
print("BFS for img3")
bfs(Node(None,(start3[0],start3[1])),img3_copy,img3_copy2)

#Astar
#Breadth first Search guarantees shortest path but astar does not guarantee shortest path
def astar(img1_copy,img1,start,end):
    print('astar called')

    #open_list stores all the nodes that we want to visit
    open_list = {}

    #closed_list stores all the nodes that we have visited
    closed_list = []

    start_node =  Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    open_list[start] = start_node
    while len(open_list)!=0:
        print("dict size = ", len(open_list))
        current_node = get_min_dist_node(open_list)
        img1[current_node.position[0]][current_node.position[1]] = orange
        open_list.pop(current_node.position)

        if current_node.position == end:
            print("Goal Reached")
            show_path(current_node,img1_copy)
            return

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] > (img1.shape[0] - 1) or node_position[0] < 0 or node_position[1] > (img1.shape[1] - 1) or node_position[1] < 0:
                print("Out of shape")
                continue
            if node_position in closed_list:
                print("Visited")
                continue
            if isObstacle(node_position,img1):
                print("Obstacle found")
                continue
            
            img1[node_position[0]][node_position[1]] = pink
            new_node = Node(current_node, node_position)
            print(new_node)

            new_node.g = current_node.g + calcDist(current_node.position, new_node.position)
            new_node.h = calcDist(new_node.position, end)
            new_node.f = new_node.g + new_node.h

            if new_node.position in open_list:
                if new_node.g < open_list[new_node.position].g:
                    open_list[new_node.position] = new_node
            else:
                open_list[new_node.position] = new_node
        
        if current_node.position not in closed_list:
            closed_list.append(current_node.position)

#Implementing Astar            
print("Astar for img1")
astar(img1_copy3,img1_copy4,start1,end1)
print("Astar for img2")
astar(img2_copy3,img2_copy4,start2,end2)
print("Astar for img3")
astar(img3_copy3,img3_copy4,start3,end3)

#Depth First Search
#Depth First Search does not guarantee shortest path
#It uses stack
def dfs(start,img1,img1_copy):
    q=deque()
    q.append(start)
    while(len(q)):
        current=q.pop()
        i,j=current.position[0],current.position[1]
        if (i+1)<img1.shape[0]:
            if (img1[i+1,j]!=black).any() and (img1[i+1,j]!=grey).any():
                if (img1[i+1,j]==green).all():
                    break
                img1[i+1,j]=grey
                n=Node(current,(i+1,j))
                q.append(n)
        if (j+1)<img1.shape[1]:
            if (img1[i,j+1]!=black).any() and (img1[i,j+1]!=grey).any():
                if (img1[i,j+1]==green).all():
                    break
                img1[i,j+1]=grey
                n=Node(current,(i,j+1))
                q.append(n)
        if (i-1)>0:
            if (img1[i-1,j]!=black).any() and (img1[i-1,j]!=grey).any():
                if (img1[i-1,j]==green).all():
                    break
                img1[i-1,j]=grey
                n=Node(current,(i-1,j))
                q.append(n)
        if (j-1)>0:
            if (img1[i,j-1]!=black).any() and (img1[i,j-1]!=grey).any():
                if (img1[i,j-1]==green).all():
                    break
                img1[i,j-1]=grey
                n=Node(current,(i,j-1))
                q.append(n)
    show_path(current,img1_copy)

#Implementing dfs
print("DFS for img1")
dfs(Node(None,(start1[0],start1[1])),img1_copy5,img1_copy6)
print("DFS for img2")
dfs(Node(None,(start2[0],start2[1])),img2_copy5,img2_copy6)
print("DFS for img3")
dfs(Node(None,(start3[0],start3[1])),img3_copy5,img3_copy6)