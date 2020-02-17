import math
import cv2
import numpy as np
from networktables import NetworkTables

NetworkTables.initialize(server='10.30.24.2')
print('NT connected: ', NetworkTables.isConnected())
netTable = NetworkTables.getTable('PracticeBot20')

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 180)
cap.set(5, 15)
cv2.namedWindow('Camera')

def nothing(x):
    pass

# globals
Errors = {0:'No error', 1:'NoneType error in HoughLines', 2:'Math domain error in trajectory'}
ERROR = 0
IMAGE = 0

# constants
px_w = 0
px_h = 0
aov_x = 0 * math.pi/180  # angle of view of camera, pi/180 to convert degrees to radians
aov_y = 0 * math.pi/180
real_h = 0  # t target height - camera height on bot
aoc = 0 * math.pi/180  # angle of camera on bot
v_init = 0  # initial velocity of ball
g = 386  # acceleration of gravity in inches per second squared

#set inrange stuff
cv2.createTrackbar('lowH','Camera',25,179, nothing)
cv2.createTrackbar('highH','Camera',102,179, nothing)
cv2.createTrackbar('lowS','Camera',220,255, nothing)
cv2.createTrackbar('highS','Camera',255,255, nothing)
cv2.createTrackbar('lowV','Camera',48,155, nothing)
cv2.createTrackbar('highV','Camera',138,255, nothing)


def calc_img():
    global ERROR
    global IMAGE

    line_endpts = []
    
    ret, frame = cap.read()
    cv2.imshow('Camera', frame)
    cv2.waitKey(200)

    #color inrange stuff
    ilowH = cv2.getTrackbarPos('lowH', 'Camera')
    ilowS = cv2.getTrackbarPos('lowS', 'Camera')
    ilowV = cv2.getTrackbarPos('lowV', 'Camera')
    ihighH = cv2.getTrackbarPos('highH', 'Camera')
    ihighS = cv2.getTrackbarPos('highS', 'Camera')
    ihighV = cv2.getTrackbarPos('highV', 'Camera')
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #more inrange
    low_hsv = np.array([ilowH, ilowS, ilowV])
    high_hsv = np.array([ihighH, ihighS, ihighV])
    mask = cv2.inRange(gray, low_hsv, high_hsv)
    cv2.waitKey(1)
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('mask', mask)
    
    #cv2.imshow('Camera', gray)
    #cv2.waitKey(1000)
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    cv2.imshow('Edges', edges)
    cv2.waitKey(1)
    min_length = 50
    max_gap = 10
    line_data = cv2.HoughLines(edges, 1, math.pi/180, 30)  # edges, 1, math.pi/180, 100, min_length, max_gap
    print(line_data)

    if line_data is None:
        ERROR = 1
        print('line_data none')
        pass
    else:
        print('line_data not none')
        #for x1, y1, x2, y2 in line_data[0]:
                #cv2.line(frame, (x1, y1), (x2, y2), (0, 250, 0), 2)

    for line[0] in line_data:
        [rho, theta] = line
        aA = np.cos(theta)
        bA = np.sin(theta)
        x0A = aA*rho
        y0A = bA*rho
        x1 = int(x0A + 1000 * (-bA))
        y1 = int(y0A + 1000 * (aA))
        x2 = int(x0A - 1000 * (-bA))
        y2 = int(y0A - 1000 * (aA))
        line_endpts.append([x1, y1, x2, y2])
        cv2.line(frame, (x1,y1), (x2,y2), (255, 255, 255), 1, 1)

    IMAGE = frame.copy
    cv2.imshow('Camera', frame)
    cv2.waitKey(1000)
    cv2.imwrite('houghlines5.jpg', frame)

    return line_endpts

def find_corners(line_endpts):
    global ERROR
    global IMAGE

    if ERROR != 0:
        return [(0, 0), (0, 0), (0, 0), (0, 0)]
    
    [x1, y1, x2, y2] = map(list, zip(*line_endpts))
    x = x1+x2
    y = y1+y2
    ax = min(x)
    ay = y[x.index(ax)]
    a = (ax, ay)
    dx = max(x)
    dy = y[x.index(dx)]
    d = (dx, dy)
    y_sorted = y
    y_sorted.sort()
    bc_y1 = y_sorted[0]
    bc_y2 = y_sorted[1]
    bc_x1 = x[y.index(bc_y1)]
    bc_x2 = x[y.index(bc_y2)]
    if bc_x1 < bc_x2:
        b = (bc_x1, bc_y1)
        c = (bc_x2, bc_y2)
    else:
        b = (bc_x2, bc_y2)
        c = (bc_x1, bc_y1)
    return [a, b, c, d]
    
def draw_lines(coordinates):
    global IMAGE
    
    ret, frame = cap.read()
    #gray = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
    #print('converted to gray')

    [a, b, c, d] = coordinates
    
    window_name = 'Camera'
    start_point = (a, b)
    end_point = (c, d)
    color = (0, 0, 255)
    thickness = 3
    IMAGE = cv2.line(frame, start_point, end_point, color, thickness)  #line from a to b
    print('got image 1')
    
    cv2.imshow(window_name, IMAGE)     #show lines n camera image
    cv2.waitKey(200)
    #print('showed images')


def calc_pos(corner_coords):
    global ERROR

    if ERROR != 0:
        return [0, 0]

    # camera constants
    px_w = 640
    px_h = 480
    aov_x = 60 * math.pi / 180  # angle of view of camera, pi/180 to convert degrees to radians
    aov_y = 30 * math.pi / 180

    # field/bot constants, all inches
    target_h = 98
    camera_h = 20
    real_h = target_h - camera_h  # t target height - camera height on bot
    aoc = math.atan(real_h / 120)  # assuming avg. distance 120 in
    print('Set camera angle to', int(aoc * 180 / math.pi), 'degrees')

    # physics constants, all in inches and seconds
    v_init = 500  # min velocity to hit at 120 in away
    g = 386

    # placeholder from array of info from camera recognition code
    [a, b, c, d] = corner_coords

    # scale ad to find real midpoint t in image
    x_scale = (b[0]-a[0])/(d[0]-c[0])
    ad = math.sqrt(((d[0]-a[0])**2 + (d[1]-a[1])**2))
    t_x = x_scale*.5*ad  # x-coord of real midpoint of target t
    # use percent of AoV to shift to center of screen for angle of rotation
    rota_percent = (t_x - .5*px_w)/px_w
    rota_theta = rota_percent*aov_x

    # find y-coord of t in image
    m = (d[1]-a[1])/(d[0]-a[0])  # slope for linear px-to-dist conversion
    t_y = m*(t_x - a[0]) + a[1]  # y=mx+b format
    # find distance dist to target wall at y-val of t
    h_percent = t_y/px_h
    t_angle = aoc - .5*aov_y + h_percent*aov_y
    try:
        assert t_angle == math.pi/2 or t_angle == math.pi or t_angle == 0
        dist = real_h/math.tan(t_angle)

    # calc trajectory for ball w/ physics equation
        traj_root = v_init**4 - g*(g*dist**2 + 2*real_h*v_init**2)
        assert traj_root < 0
        assert dist == 0
        yaw_angle = math.atan((v_init**2 - math.sqrt(traj_root))/(g*dist))
        
    except AssertionError:
        yaw_angle = 0
        ERROR = 2


    return [rota_theta, yaw_angle*180/math.pi]

count = 0


while True:
    ERROR = 0

    LINES = calc_img()
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break
    for x in LINES:
        draw_lines(x)
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break
    CORNERS = find_corners(LINES)

    DATA = calc_pos(CORNERS)

    DATA.append(ERROR)

    netTable.putNumberArray('VisionData', DATA)
    if count % 15 == 0:  # paces print statements to be readable
        print()
        print('Corner coordinate data: ', CORNERS)
        print('Rotation angle: ', DATA[0], '    Yaw angle: ', DATA[1])
        print(Errors[ERROR])
    count += 1

cap.release()
cv2.destroyAllWindows()

    
    
