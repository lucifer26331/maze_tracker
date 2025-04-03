import random
import cv2 as cv
import glob
import time

imdir = 'traffic_signs/'  # this is the directory in which all the traffic sign images are stored
ext = ['png', 'jpg', 'gif']  # all possible extension of traffic sign images
files = []  # contains unread images
templates = []  # contains gray and cropped templates
frames = []  # contains read images
cv.namedWindow('Video', cv.WINDOW_NORMAL)  # window used to display the video
video = cv.VideoWriter('Traffic_signals.mp4', cv.VideoWriter_fourcc(*'mp4v'), 60, (400, 400))  # stores the video

red_light_index = 0  # stores the position of red light as it is special case
green_light_index = 0  # when red light is displayed we need this to be displayed later
stop_index = 0  # stores the position of stop board as it is special case
go_index = 0  # when stop board is displayed we need this to be displayed later

message = ""  # stores the message to be written in the file
curr_message = ""  # this helps in avoiding repeat messages
total_time = 0  # stores the approximate total time of the video


def get_template(img):
    threshold = 0.9  # if match template finds a 90% match it will give positive result
    temp = cv.cvtColor(img, cv.COLOR_BGR2GRAY)  # as match template uses grayscale images
    index = -1  # this helps to keep track of the return message
    for template in templates:
        res = cv.matchTemplate(temp, template, cv.TM_CCOEFF_NORMED)
        index += 1
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        if max_val >= threshold:
            return index


def get_message(index):
    messages = {0: "30 30", 1: "75 75", 2: "30 -30", 3: "20 20", 4: "35 35", 5: "0 30", 6: "0 0", 7: "30 0", 8: "20 20",
                9: "40 40", 10: "60 60", 11: "0 0", 12: "error"}  # new messages can be added to the dictionary
    return messages[index]


# this loop stores all the images in unread form in files
for i in ext:
    files.extend(glob.glob(imdir + '*.' + i))

# this loop stores all the images in read form in frames
for file in files:
    frames.append(cv.imread(file))

# this loop collects all the special case indices
for index in range(len(files)):
    if files[index] == 'traffic_signs\\redlight.jpg':
        red_light_index = index
    if files[index] == 'traffic_signs\\greenlight.jpg':
        green_light_index = index
    if files[index] == 'traffic_signs\\go.png':
        go_index = index
    if files[index] == 'traffic_signs\\stop.jpg':
        stop_index = index

# this loop resizes the images and stores templates in templates list
for i in range(len(frames)):
    frames[i] = cv.resize(frames[i], (400, 400))
    templates.append(cv.cvtColor(frames[i][70:340, 70:340], cv.COLOR_BGR2GRAY))

# Making a video
rand_index = 0  # this will contain a random index from the frames list
while 1:
    if rand_index == red_light_index:  # special case when red light is displayed
        rand_index = green_light_index
    elif rand_index == stop_index:  # special case when stop sign is displayed
        rand_index = go_index
    else:
        rand_index = random.randint(0, len(frames) - 1)

    image = frames[rand_index]
    time_frame = random.randint(4, 6)  # randomizes the amount of time a sign is displayed
    time_video = time.time() + time_frame  # keeps track of total video length
    total_time += time_frame
    while time.time() < time_video:
        cv.imshow('Video', image)
        video.write(image)  # saving the video
        cv.waitKey(1)

    if cv.waitKey(1) & 0xFF == ord('d'):
        break
    if total_time > 75:  # we can change the video length from here
        break
cv.destroyAllWindows()
video.release()

# Matching templates

vid = cv.VideoCapture('Traffic_signals.mp4')  # reading the stored video
file = open('motor_speeds.txt', 'w')  # this file will contain the speed data
if vid is None:
    print("Check video location")  # if video is not found in the same directory

time1 = time.time()  # helps in avoiding multiple template checks and makes program run faster
while True:
    time2 = time.time()
    is_true, frame = vid.read()
    if is_true is False:
        break
    if time2 - time1 > 0.04:  # checks template once in a while to make program faster
        index = get_template(frame)
        message = get_message(index)
        time1 = time.time()
    if curr_message != message:
        curr_message = message
        file.write(curr_message + "\n")

file.close()
cv.destroyAllWindows()