#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from enum import Enum


class Tree:
    def __init__(self,start):
        self.verticies = []
        self.edges = []
        self.init(start)
        self.max_state =start
        self.neighbours  = {}
        self.goal = []
        self.cost = {}

        self.cost[tuple(start)] = 1


    def get_closest_goal(self):
        # ax = plt.axes()
        # self.print_tree(ax)
        # plt.show()
        if len(self.goal)==0:
            t ="##########################################\n"
            one ="# Path not found with current iterations #\n"
            two ="# Please vary the thresholds to find the #\n"
            g = "# goal, or rerun.                        #\n"


            print(f"{t}{one}{two}{g}{t}")
            exit(0)


        return self.goal[0]


    def init(self,x_new):
        self.verticies.append(x_new)

    def add_vertex(self,x_new):
        self.verticies.append(x_new)
    def make_neighbours(self):
        #for i in range(len(self.edges)):
            #Dprint(self.edges[i][0].reshape(2,))
            #print(self.edges[i])
        self.neighbours = { tuple(x1.reshape(2,)):tuple(x2.reshape(2,)) for (x1,x2) in self.edges }

    def add_edge(self,x1,x2):
        self.edges.append((x1,x2))
        if tuple(x1.reshape(2,)) not in self.neighbours:
            self.neighbours[tuple(x1.reshape(2,))]=[]
        else:
            self.neighbours[tuple(x1.reshape(2,))].append(tuple(x2.reshape(2,)))
        if tuple(x2.reshape(2,)) not in self.neighbours:
            self.neighbours[tuple(x2.reshape(2,))]=[]
        else:
            self.neighbours[tuple(x2.reshape(2,))].append(tuple(x1.reshape(2,)))
        self.cost[tuple(x2)] = self.cost[tuple(x1)] +1
    def get_cost(self,a,b)->float:
        #print(self.cost[b])
        return self.cost[b]
    def neighbours_at(self,curr):
        #print(f"The curr is {curr}")
        return self.neighbours[curr]
    def print_tree(self,main_axis):
        main_axis.scatter([a[0] for a in self.verticies], [a[1] for a in self.verticies])
        for edge in self.edges:
            #print(edge)
            x,y = list(zip(edge[0],edge[1]))
            #main_axis.plot([edge[0][0],edge[1][0]] , [edge[0][1],edge[1][1]],color='green')
            main_axis.plot(x,y,color='green')

        main_axis.scatter([self.verticies[0][0]], [self.verticies[0][1]],marker="*",s=200,color="orange",zorder=10)
        main_axis.scatter([self.goal[0][0]], [self.goal[0][1]],marker="X",s=200,color="orange",zorder=10)


def euclid_dist(x1,x2):
    return  sum([ ((x1[i])-x2[i])**2 for i in range(len(x1))  ])**0.5

class CollisionType(Enum):
    GOAL_FOUND = 0
    COLLISION = 1
    NO_COLLISION =2