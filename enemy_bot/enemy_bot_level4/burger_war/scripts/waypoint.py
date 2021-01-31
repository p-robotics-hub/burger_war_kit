#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math

class Waypoints:

    def __init__(self, path):
        self.points = []
        self.number = 0
        self._load_waypoints(path)
        print ('number of waypoints: '+str(len(self.points)))

    def _load_waypoints(self, path):
        with open(path) as f:
            lines = csv.reader(f)
            for l in lines:
                point = [float(n) for n in l]
                point[2] = point[2]*math.pi/180.0
                self.points.append(point)

    def get_next_waypoint(self):
        self.number = self.number+1
        if self.number == len(self.points):
            self.number = 0          
        return self.points[self.number][:]

    def get_current_waypoint(self):
        return self.points[self.number]

    def get_any_waypoint(self, n):
        return self.points[n]

    def set_number(self, n):
        self.number = n


        
                


if __name__ == "__main__":
    Waypoints('waypoints.csv')