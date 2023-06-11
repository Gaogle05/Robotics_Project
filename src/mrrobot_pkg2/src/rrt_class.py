# RRT Algo class
# Salmaan
# Following the tutorial @ https://www.youtube.com/watch?v=OXikozpLFGo
# To-do: optimise rrt to rrt* then to rrt#

import numpy as np
import matplotlib.pyplot as plt
import random

# treeNode class


class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

# RRT Algo Class


class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])  # root
        self.goal = treeNode(goal[0], goal[1])  # goal position
        self.nearestNode = None  # nearest node
        self.iterations = min(numIterations, 200)  # number of iterations
        self.grid = grid  # map
        self.rho = stepSize  # this is the length of each branch
        self.path_distance = 0  # total path distance
        self.nearestDist = 10000  # distance to nearest node
        self.numWaypoints = 0  # number of waypoints
        self.Waypoints = []  # the waypoints themselves

    # adds point to nearest node and adds goal when reached
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            # adds goal node to children of nearest node
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)
            # adds tempNode to children of nearest node
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

    # sample a random point within grid limits
    def sampleAPoint(self):
        # (x,y) here means (y,x) in grid, that's why its 1 not 0
        x = random.randint(1, self.grid.shape[1]-1)
        y = random.randint(1, self.grid.shape[0]-1)
        point = np.array([x, y])
        return point

    # steer a distance stepsize from start to end
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0],
                         locationStart.locationY + offset[1]])
        if (point[0] >= self.grid.shape[1]):
            point[0] = self.grid.shape[1] - 1
        if (point[1] >= self.grid.shape[0]):
            point[1] = self.grid.shape[0] - 1
        return point

    # checks if obstacle is in between start node and end point of edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            # check if testPoint lies wihtin obstacle
            testPoint[0] = min(testPoint[0], self.grid.shape[1] - 1)
            testPoint[1] = min(testPoint[1], self.grid.shape[0] - 1)

            # checking if testPoint lies within obstacle
            if (self.grid[int(round(testPoint[1])), int(round(testPoint[0]))] == 1):
                return True

        return False

    # find unit vector btwn a node and an end point which forms a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX,
                     locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat

    # find nearest node from a give unconnected point (using Euclidean norm)
    def findNearest(self, root, point):
        # return condition if root is NULL
        # find distance btwn root and point
        # if this is lower than nearest distance, set as nearest node + update distance
        if not root:
            return
        dist = self.distance(root, point)
        if (dist <= self.nearestDist):
            self.nearestNode = root
            self.nearestDist = dist
        # recursively call by iterating through children
        for child in root.children:
            # do something
            self.findNearest(child, point)
            pass

    # find euclidean distance bween node and (x,y) point
    def distance(self, node1, point):
        dist = np.sqrt(
            (node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist

    # check if goal has been reached within step size
    def goalFound(self, point):
        # hint: check is point is within goal by using distance
        if (self.distance(self.goal, point) <= self.rho):
            return True
        return False

    # reset nearestNode and nearest distance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    # trace path from goal to start
    # HINT: it goes backwards through parent nodes, maybe use recursion here
    def retraceRRTPath(self, goal):
        # end recursion when goal node reaches start
        if goal is None:
            print("Goal is out of map: Choose a different goal.")
            return
        if (goal.locationX == self.randomTree.locationX):
            return
        # adds 1 to number of waypoints
        self.numWaypoints += 1
        # insert currentPoint to waypoints array from beginning
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0, currentPoint)
        # adds step size to path distance
        self.path_distance += self.rho
        # recursion
        self.retraceRRTPath(goal.parent)

# end of class-------------------------------------------------------------
