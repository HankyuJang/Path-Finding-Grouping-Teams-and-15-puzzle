# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 17:31:29 2017

@author: PulkitMaloo
"""
# Assumption speed = 45 for missing or 0 values since majority of the
# road segments have speed 45

from __future__ import division
import sys
import numpy as np


# ==============================================================================
# The format of data is:
# data =
#   {
#   city:
#       {
#           latitude: value
#           longitude: value
#           to_city:
#               {
#                   name: {dist: value, speed: value, highway: value},
#                   name: {..},
#                   ...
#               }
#       }
#   }
#
# PS: The missing values are filled with None
# ==============================================================================


def reading_files():
    data = dict()
    f_city = open('city-gps.txt', 'r')
    f_road = open('road-segments.txt', 'r')

    for city in f_city:
        city = city.split()
        data[city[0]] = {'latitude': city[1], 'longitude': city[2]}

    for seg in f_road:
        seg = seg.split()

        # One city has a route to itself
        if seg[0] == seg[1]:
            continue

        # Some road segment are not in city-gps, they don't have lat or lon
        if seg[0] not in data:
            data[seg[0]] = {'latitude': None, 'longitude': None}

        # Some road segment didn't have speed
        if len(seg) != 5:
             seg = seg[:3] + [None] + seg[3:]

        # from_city to to_city
        if 'to_city' not in data[seg[0]]:
            data[seg[0]]['to_city'] = dict()
        data[seg[0]]['to_city'][seg[1]] = {'dist': seg[2], 'speed': seg[3], 'highway': seg[4]}

        # to_city to from_city
        if seg[1] not in data:
            data[seg[1]] = {'latitude': None, 'longitude': None}

        if len(seg) != 5:
             seg = seg[:3] + [None] + seg[3:]

        if 'to_city' not in data[seg[1]]:
            data[seg[1]]['to_city'] = dict()
        data[seg[1]]['to_city'][seg[0]] = {'dist': seg[2], 'speed': seg[3], 'highway': seg[4]}

    f_city.close()
    f_road.close()
    return data

def lat_lon_distance(from_city, to_city):
#### This function is copied from ######################################
# https://www.w3resource.com/python-exercises/math/python-math-exercise-27.php
    from math import radians, sin, cos, acos
    slat = radians(float(data[from_city]['latitude']))
    slon = radians(float(data[from_city]['longitude']))
    elat = radians(float(data[to_city]['latitude']))
    elon = radians(float(data[to_city]['longitude']))
    dist = 6371.01 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon - elon))
    return round(dist, 2)

def distance(from_city, to_city):
    # Handle distance = 0
    dist = int(data[from_city]['to_city'][to_city]['dist'])
    return dist if dist > 0 else lat_lon_distance(from_city, to_city)

def speed(from_city, to_city):
    # Handling speed = 0 or missing values by returning 45
    try:
        speed = int(data[from_city]['to_city'][to_city]['speed'])
        return speed if speed > 0 else 45
    except TypeError:
        return 45

def time(from_city, to_city):
    return distance(from_city, to_city)/speed(from_city, to_city)

def cost(current_city, next_city):
    if cost_function == 'distance':
        return distance(current_city, next_city)
    elif cost_function == 'time':
        return time(current_city, next_city)
    else:
        return 1

def is_goal(city):
    return city == end_city

def successors(city):
    return data[city]['to_city'].keys()

def solve(start_city):
    paths = [[start_city]]
    visited = []
    while paths:
        # use pop() for DFS
        path = paths.pop(0)
        curr = path[-1]
        for next_city in successors(curr):
            if next_city in visited:
                continue
            if is_goal(next_city):
                return path + [next_city]
            visited.append(next_city)
            new_path = path + [next_city]
            paths.append(new_path)

    # What to output when no path is found
    return False


start_city = 'Bloomington,_Indiana' #sys.argv[0]
end_city = 'Indianapolis,_Indiana' #sys.argv[1]
routing_algorithm = 'bfs' #sys.argv[2]
cost_function = 'segments' #sys.argv[3]

data = reading_files()
print(' '.join(solve(start_city)))
