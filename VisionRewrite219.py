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
image1 = 0

#set inrange stuff
cv2.createTrackbar('lowH','Camera',25,179, nothing)
cv2.createTrackbar('highH','Camera',102,179, nothing)
cv2.createTrackbar('lowS','Camera',220,255, nothing)
cv2.createTrackbar('highS','Camera',255,255, nothing)
cv2.createTrackbar('lowV','Camera',48,155, nothing)
cv2.createTrackbar('highV','Camera',138,255, nothing)
cv2.createTrackbar('threshold','Camera',40,80, nothing)


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] *b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        #print('got none')
        return None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return int(x), int(y)


def calc_img():
    global ERROR
    global image1
    
    ret, frame = cap.read()
    #cv2.imshow('Camera', frame)

    #color inrange stuff
    ilowH = cv2.getTrackbarPos('lowH', 'Camera')
    ilowS = cv2.getTrackbarPos('lowS', 'Camera')
    ilowV = cv2.getTrackbarPos('lowV', 'Camera')
    ihighH = cv2.getTrackbarPos('highH', 'Camera')
    ihighS = cv2.getTrackbarPos('highS', 'Camera')
    ihighV = cv2.getTrackbarPos('highV', 'Camera')
    ithreshold = cv2.getTrackbarPos('threshold', 'Camera')
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #more inrange
    low_hsv = np.array([ilowH, ilowS, ilowV])
    high_hsv = np.array([ihighH, ihighS, ihighV])
    mask = cv2.inRange(gray, low_hsv, high_hsv)
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('mask', mask)
    
    #cv2.imshow('Camera', gray)
    #cv2.waitKey(1000)
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    cv2.imshow('Edges', edges)
    cv2.waitKey(1)
    min_length = 50
    max_gap = 10
    line_data = cv2.HoughLines(edges, 1, math.pi/180, ithreshold)  # edges, 1, math.pi/180, 100, min_length, max_gap

    
    intersect_array = []
    theta_array = []

    if line_data == None:
        pass
    else:
        i = 0
        line_data_len = len(line_data)
        lineLimit = 100
        if (line_data_len > lineLimit):
            print('Reducing lines from ', line_data_len, ' to ', lineLimit)
            line_data_len = lineLimit

        numPlus60 = 0;
        numMinus60 = 0
        numZero = 0;

        while i < line_data_len:
            for rho, theta in line_data[i]:
                commonTheta = theta * 57.2958 - 90;
                if (commonTheta < -45):
                    numMinus60 += 1
                else:
                    if (commonTheta < 45):
                        numZero += 1
                    else:
                        if (commonTheta > 45):
                            numPlus60 += 1
                        else:
                            print('Illegal commonTheta ', commonTheta)
                #print('commonTheta ', commonTheta)
                aA = np.cos(theta)
                bA = np.sin(theta)
                x0A = aA*rho
                y0A = bA*rho
                x1 = int(x0A + 1000 * (-bA))
                y1 = int(y0A + 1000 * (aA))
                x2 = int(x0A - 1000 * (-bA))
                y2 = int(y0A - 1000 * (aA))
                cv2.line(frame, (x1,y1), (x2,y2), (255, 255, 255), 1, 1)
                j = 0
                while j < line_data_len:
                    if i == j:
                        j += 1
                    else:
                        for rhob, thetab in line_data[j]:
                            aB = np.cos(thetab)
                            bB = np.sin(thetab)
                            x0B = aB*rhob
                            y0B = bB*rhob
                            x3 = int(x0B + 1000 * (-bB))
                            y3 = int(y0B + 1000 * (aB))
                            x4 = int(x0B - 1000 * (-bB))
                            y4 = int(y0B - 1000 * (aB))
                            line1 = ((x1, y1), (x2, y2))
                            line2 = ((x3, y3), (x4, y4))
                            intersect = line_intersection(line1, line2)
                            if intersect == None:
                                x=1
                                #print('got none')
                            else:
                                intersect_array.append(intersect)
                                theta_array.append((theta, thetab))
                                for x, y in [intersect]:
                                    cv2.line(frame, (x - 2, y), (x + 2, y), (255, 60, 0), 2, 1)
                                    cv2.line(frame, (x, y - 2), (x, y + 2), (255, 60, 0), 2, 1)
                                    
                            j += 1
                i += 1

    if line_data == None:
        ERROR = 1
        print('line_data none')
        pass
    else:
        x=1
        #print('line_data not none')
        #for x1, y1, x2, y2 in line_data[0]:
                #cv2.line(frame, (x1, y1), (x2, y2), (0, 250, 0), 2)

    image1 = frame.copy
    #cv2.imshow('Camera', frame)
    #cv2.waitKey(1000)
    #cv2.imwrite('houghlines5.jpg', frame)
    
    line_endpts = []
    if line_data == None:
        pass
    else:
        for i in line_data:
            line_endpts.append(i[0])


    return intersect_array, theta_array, frame

def organize_points(intersect_array, theta_array, frame):
    useful_points = []
    leftmostX = 1000;
    leftmostY = 1000;
    rightmostX = -1000;
    rightmostY = -1000;
    lowestX = 0;
    lowestY = 0;
    for i in range(len(intersect_array)):
        for theta1, theta2 in [theta_array[i]]:
            t = (abs(theta1 - theta2)) * 57.2958
            if t > 10:
                for x, y in [intersect_array[i]]:
                    if (x < leftmostX):
                        leftmostX = x
                        leftmostY = y;
                        #print('left thetas ', theta1 * 57.2958 - 90, theta2 * 57.2958 - 90)
                    if (x > rightmostX):
                        rightmostX = x;
                        rightmostY = y;
                        #print('right thetas ', theta1 * 57.2958 - 90, theta2 * 57.2958 - 90)
                    if (y > lowestY):
                        lowestX = x;
                        lowestY = y;
                        #print('low thetas ', theta1 * 57.2958 - 90, theta2 * 57.2958 - 90)
                    cv2.line(frame, (x - 2, y), (x + 2, y), (0, 255, 255), 2, 1)
                    cv2.line(frame, (x, y - 2), (x, y + 2), (0, 255, 255), 2, 1)

    useful_points.append((leftmostX, 480-leftmostY))
    useful_points.append((rightmostX, 480-rightmostY))
    useful_points.append((lowestX, 480-lowestY))
                    
    cv2.line(frame, (leftmostX - 2, leftmostY), (leftmostX + 2, leftmostY), (0, 0, 255), 2, 1)
    cv2.line(frame, (leftmostX, leftmostY - 2), (leftmostX, leftmostY + 2), (0, 0, 255), 2, 1)
    cv2.line(frame, (rightmostX - 2, rightmostY), (rightmostX + 2, rightmostY), (0, 0, 255), 2, 1)
    cv2.line(frame, (rightmostX, rightmostY - 2), (rightmostX, rightmostY + 2), (0, 0, 255), 2, 1)
    cv2.line(frame, (lowestX - 2, lowestY), (lowestX + 2, lowestY), (0, 0, 255), 2, 1)
    cv2.line(frame, (lowestX, lowestY - 2), (lowestX, lowestY + 2), (0, 0, 255), 2, 1)
    cv2.imshow('Camera', frame)
    cv2.waitKey(1)
    
    return useful_points


#def find_corners(line_endpts, intersect_array):
#    global ERROR
#    global image1

 #   if ERROR != 0:
 #       return [(0, 0), (0, 0), (0, 0), (0, 0)]
    
 #   [x1, y1, x2, y2] = map(list, zip(*line_endpts))
 #   x = x1+x2
 #   y = y1+y2
 #   ax = min(x)
 #   ay = y[x.index(ax)]
 #   a = (ax, ay)
 #   dx = max(x)
 #   dy = y[x.index(dx)]
 #   d = (dx, dy)
 #   y_sorted = y
 #   y_sorted.sort()
 #   bc_y1 = y_sorted[0]
 #   bc_y2 = y_sorted[1]
 #   bc_x1 = x[y.index(bc_y1)]
 #  bc_x2 = x[y.index(bc_y2)]
 #   if bc_x1 < bc_x2:
 #       b = (bc_x1, bc_y1)
 #       c = (bc_x2, bc_y2)
 #   else:
 #       b = (bc_x2, bc_y2)
 #       c = (bc_x1, bc_y1)
 #   return [a, b, c, d]


def calc_pos(corner_coords):
    global ERROR

    if ERROR != 0:
        return [0, 0]

    # camera/bot constants
    px_w = 640
    px_h = 480
    aov_x = 60 * math.pi / 180  # angle of view of camera, pi/180 to convert degrees to radians
    aov_y = 30 * math.pi / 180
    ao_cam = 0

    # physics/field constants
    cam_h = 20
    shot_h = 30
    targ_h = 98
    v_init = 565  # min velocity to hit at 120 in away
    g = 386

    # camera array info
    [a, b, c] = corner_coords

    # camera trig calculations
    mx = b[0]
    my = (c[1]-a[1])/(c[0]-a[0])*(mx - a[0]) + a[1]  # point slope form equation of line AC

    theta_z = aov_x*(.5*px_w - mx)/px_w  # percent of x pixels B is away from center of frame times angle of the frame
    theta_d = aov_y*my/px_h - .5*aov_y + ao_cam
    d = (targ_h-cam_h)/math.tan(theta_d)  # adjacent = opposite/tangent

    # calc trajectory for ball w/ physics equation
    t_root = v_init**4- g*(g*d**2 + 2*(targ_h-shot_h)*v_init**2)
    print(t_root)
    if t_root >= 0 and d > 0:
        t = math.atan((v_init**2 - math.sqrt(t_root))/(g*d))
    else:
        t = 0
        ERROR = 2
    return [theta_z, t*180/math.pi]

count = 0


while True:
    ERROR = 0

    POINTS, THETA, FRAME = calc_img()
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break
    USEFUL = organize_points(POINTS, THETA, FRAME)

    DATA = calc_pos(USEFUL)

    DATA.append(ERROR)

    netTable.putNumberArray('VisionData', DATA)
    if count % 15 == 0:  # paces print statements to be readable
        print()
        print('Corner coordinate data: ', USEFUL)
        print('Rotation angle: ', DATA[0], '    Yaw angle: ', DATA[1])
        print(Errors[ERROR])
    count += 1

cap.release()
cv2.destroyAllWindows()

    
    
