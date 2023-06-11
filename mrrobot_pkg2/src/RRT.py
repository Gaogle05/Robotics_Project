# RRT Algo
# Salmaan

import numpy as np
import matplotlib.pyplot as plt
import random
from PIL import Image, ImageOps  # Python pillow

from pid_controller import PID
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Point
import rospy

from rrt_class import RRTAlgorithm

# Function converts the image into a numpy array of 0s and 1s

# instance of the class


def RRT(values, printer):
    start, goal, numIterations, grid, stepSize = values
    rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)

    for i in range(rrt.iterations):
        # Reset nearest values
        rrt.resetNearestValues()
        # print("iteration: ", i)

        # algo
        point = rrt.sampleAPoint()
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.steerToPoint(rrt.nearestNode, point)
        boolObstacle = rrt.isInObstacle(rrt.nearestNode, new)
        if (boolObstacle == False):
            rrt.addChild(new[0], new[1])
            # plt.pause(0.10)
            # plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="--")
            plt.plot([rrt.nearestNode.locationX, new[0]], [
                     rrt.nearestNode.locationY, new[1]], 'go', linestyle="--")
            # if goal found, append to path
            if (rrt.goalFound(new)):
                rrt.addChild(goal[0], goal[1])
                print("Goal found!")
                break

    # trace back the path returned then adds start to waypoints
    rrt.retraceRRTPath(rrt.goal)
    rrt.Waypoints.insert(0, start)

    if (printer):
        print("Iterations: ", rrt.iterations)
        print("Num Waypoints", rrt.numWaypoints)
        print("Path distance (m): ", rrt.path_distance)
        #print("Waypoints: ", rrt.Waypoints)

    return rrt.Waypoints


def convert_image(show):
    # NOTE: the grid is (y,x) not (x,y)
    img = Image.open('C:/Users/salma/Desktop/MrRobot/Algo/map.png')
    img = ImageOps.grayscale(img)
    np_img = np.array(img)
    np_img = ~np_img  # inverts black and white
    np_img[np_img > 0] = 1
    plt.set_cmap('binary')
    plt.imshow(np_img)
    np.save('map.npy', np_img)
    grid = np.load('map.npy')
    plt.imshow(grid)
    plt.tight_layout()
    if (show == 1):
        plt.show()


def plotStartGoal(values):
    start, goal, numIterations, grid, stepSize = values

    goalRegion = plt.Circle(
        (goal[0], goal[1]), stepSize, color='b', fill=False)
    fig = plt.figure("RRT Algo")
    # plotting
    plt.imshow(grid, cmap='binary')
    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'bo')
    ax = fig.gca()
    ax.add_patch(goalRegion)
    plt.xlabel('X-axis $(m)$')
    plt.ylabel('Y-xis $(m)$')
    plt.title('RRT Algo by MrRobot')
    # plt.show()


def plotWaypoints(Waypoints):
    for i in range(len(Waypoints)-1):
        plt.plot([Waypoints[i][0], Waypoints[i+1][0]],
                 [Waypoints[i][1], Waypoints[i+1][1]],
                 'ro', linestyle="--")
        plt.pause(0.10)


def plotRRT(Waypoints):
    # plotStartGoal(values)
    plotWaypoints(Waypoints)
    plt.show()


def getValues(s_x, s_y, g_x, g_y):
    grid = np.load('map.npy')
    start = np.array([s_x+0.0, s_y+0.0])
    goal = np.array([g_x+0.0, g_y+0.0])
    numIterations = 200
    stepSize = 50
    return [start, goal, numIterations, grid, stepSize]


def printArray(arr, n):
    for i in range(n):
        print("x: ", round(arr[i][0], 2), "\ty :", round(arr[i][1], 2))


def remapCoords(x1, y1, x2, y2):
    # Here we perform a linear translation of the map basis to change the origin co-ords: top left corner -> turtlebot origin
    # origin of turtlebot in array: (325, 170)
    s_x = x1 + 325
    s_y = -y1 + 170

    g_x = x1 + 325
    g_y = -y1 + 170

    return s_x, s_y, g_x, g_y


def coords_map_to_plt(x, y):
    #(16,20) to (400,500)
    #x = (x-13)*25
    #y = -(y+7)*25
    x = 325 + 25*x 
    y = -(175 + 25*y)
    return x, y


def coords_plt_to_map(x, y):
    #(400,500) to (16,20)
    x = x/25 - 13
    y = -(y/25) - 7
    return x, y


def arr_coords_plt_to_map(path):
    # change to map coords as array
    for i in range(path.shape[0]):
        path[i][0], path[i][1] = coords_plt_to_map(
            round(path[i][0]), round(path[i][1]))
    return path


def to_main(startx, starty, goalx, goaly):
    # convert_image(1)  # 0 to leave map, 1 to show it
    s_x, s_y, g_x, g_y = startx, starty, goalx, goaly
    values = getValues(s_x, s_y, g_x, g_y)

    plotStartGoal(values)
    recalculations = 0  # sometimes it doesn't find the goal
    #To-do optimise using RRT* or RRT#
    num_waypoints = 2
    while num_waypoints == 2:  # while loop becomes sometimes RRTs random nodes doesn't reach the goal node
        # 1 means print details, 0 means leave it out
        Waypoints = RRT(values, 1)
        #plotRRT(values, Waypoints)
        # print(Waypoints)
        num_waypoints = len(Waypoints)
        print("Number of waypoints: ", num_waypoints)
        if (num_waypoints > 2):
            plotWaypoints(Waypoints)
            plt.show()
            printArray(Waypoints, num_waypoints)
        else:
            print("Recalculating...")
            recalculations += 1
            if recalculations > 5:
                recalculated_too_many_times = True
                assert recalculated_too_many_times, "Out of map error: select co-ordinates in the range Sir"
                break

    print("Times recalculated: ", recalculations)
    #print("End of file")


def main():
    live = 1
    
    startx = 325
    starty = 170
    goalx = 326
    goaly = 171
    
    if (live):
        pid = PID()
        state = pid.get_state()
        startx = state.pose.position.x
        starty = state.pose.position.y
    

    print("\nHello Mr. Top Secret Government Agent - i.e. Pravesh/Benji :)\nI will look for suspicious objects on my way there.\nInstruct me on where to go in (x,y) co-ordinates.\n")

    goalx = float(input('x (goal): '))
    goaly = float(input('y (goal): '))

    #s_x, s_y, g_x, g_y = remapCoords(startx, starty, goalx, goaly)
    
    s_x, s_y = coords_map_to_plt(startx, starty)
    g_x, g_y = coords_map_to_plt(goalx, goaly)
    

    x_max = 400
    y_max = 500

    x_min = 0
    y_min = 0

    while (goalx >= x_max or goaly >= y_max or goalx <= x_min or goaly <= y_min):
        print("\nSir, I am unauthorised to access that location. Please provide me with (x,y) co-ords in the range of\nx:0 to 399\ny:0 to 499\n")
        goalx = float(input('x1 (goal): '))
        goaly = float(input('x2 (goal): '))

    print("\nMission Accepted Sir.\nI am just calculating the path, then we're ready to go!\nPath: calculating...\n")

    print(startx, starty, goalx, goaly)
    # MAIN ALGO
    X, Y = to_main(startx, starty, goalx, goaly)

    path = np.column_stack((X, Y))
    path = arr_coords_plt_to_map(path)
    print("\nConfidential co-ordinates:")
    print(path)
    #print("Okay, Mission Accomplished.\nSuccessfully reached the destination.")
    return (path)


path = main()
