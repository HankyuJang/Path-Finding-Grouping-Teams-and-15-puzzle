# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 17:31:29 2017

@author: PulkitMaloo
"""
# Assumption 1
# speed = 45 for missing or 0 values
# since majority of the road segments seem to have speed 45

# Assumption 2
# When speed=0 and distance=0 there is no route possible
# Because the highway name is ferry, there's no road there

from __future__ import division
import sys
import numpy as np


# ==============================================================================
#   The format of data is:
#
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

        # Ferry cases where speed=0 and distance=0
        if int(seg[2])==0 and int(seg[3])==0:
            continue

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

        #### Delete this ########
#        if int(data[seg[1]]['to_city'][seg[0]]['dist'])==0:
#            import pandas as pd
#            print(data[seg[1]]['latitude'], data[seg[1]]['longitude'])

    f_city.close()
    f_road.close()
    return data

def lat_lon_distance(from_city, to_city):
# This function is copied from the following website
# https://www.w3resource.com/python-exercises/math/python-math-exercise-27.php
    from math import radians, sin, cos, acos
    try:
        slat = radians(float(data[from_city]['latitude']))
        slon = radians(float(data[from_city]['longitude']))
        elat = radians(float(data[to_city]['latitude']))
        elon = radians(float(data[to_city]['longitude']))
        dist = 6371.01 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon - elon))
        return dist
    except:
        #### What to return here? #########
        # If shortest path infinity would work
        # If longest path
        return float('inf')

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
    # Paths is is a list of path where each path can explored further
    paths = [[start_city]]
    distance_of_paths = [0]
    time_of_paths = [0]
    segments_of_paths = [0]
    costs_of_paths = [0]
    # List of the visited cities
    visited = []
    # While there are still paths to be explored
    while paths:
        # Retrieve a path according to BFS or DFS
        # use pop(0) for BFS
        # use pop() for DFS
        astar = 0
        i = 0 if routing_algorithm in ['bfs','uniform'] else -1 if routing_algorithm=='dfs' else astar
        path = paths.pop(i)
        curr_dist = distance_of_paths.pop(i)
        curr_time = time_of_paths.pop(i)
        curr_segments = segments_of_paths.pop(i)
        curr_cost = costs_of_paths.pop(i)
        # Retreive the last city from the path to be expanded
        curr_city = path[-1]
        # For all cities that we can go from the current city
        for next_city in successors(curr_city):
#==============================================================================
#            print(path, curr_dist, curr_time, curr_cost)
#==============================================================================
            # And the next_city is already visited, evaluate the next successor
            if next_city in visited:
                continue
            # Updating path, distance, time, and segments
            new_path = path + [next_city]
            new_cost = curr_cost + cost(curr_city, next_city)
            new_dist = curr_dist + distance(curr_city, next_city)
            new_time = curr_time + time(curr_city, next_city)
            new_segments = curr_segments + 1
            # Check if it's our goal state then return the path
            if is_goal(next_city):
                return [str(new_dist), str(new_time)] + new_path
            # Add this current city to our list of visited cities
            visited.append(next_city)
            # Add the new expanded path to our paths list

            paths.append(new_path)
            costs_of_paths.append(new_cost)
            distance_of_paths.append(new_dist)
            time_of_paths.append(new_time)
            segments_of_paths.append(new_segments)

    # What to output when no path is found?
    return False


start_city = 'Bloomington,_Indiana' #sys.argv[0]
end_city = 'Indianapolis,_Indiana' #sys.argv[1]
routing_algorithm = 'bfs' #sys.argv[2]
cost_function = 'time' #sys.argv[3]

data = reading_files()
try:
    print(' '.join(solve(start_city)))
except TypeError:
    print("No route found!")