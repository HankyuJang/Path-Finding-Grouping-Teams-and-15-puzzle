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
#           "latitude": value
#           "longitude": value
#           "to_city":
#               {
#                   city: {"dist": value, "speed": value, "highway": value},
#                   city: {..},
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

    f_city.close()
    f_road.close()
    return data

def dist_nearest_city(city):
    k = successors(city)
    v = [distance(city, i) for i in k]
    return k[v.index(min(v))]

def dist_farthest_city(city):
    k = successors(city)
    v = [distance(city, i) for i in k]
    return k[v.index(max(v))]

def cost_nearest_city(city):
    k = successors(city)
    v = [cost(city, i) for i in k]
    return k[v.index(min(v))]

def latitude(city):
    try:
        return float(data[city]['latitude'])
    except:
        return float(data[dist_nearest_city(city)]['latitude'])

def longitude(city):
    try:
        return float(data[city]['longitude'])
    except:
        return float(data[dist_nearest_city(city)]['longitude'])

def lat_lon_distance(from_city, to_city):
# This function is copied from the following website
# https://www.w3resource.com/python-exercises/math/python-math-exercise-27.php
    from math import radians, sin, cos, acos
    slat = radians(latitude(from_city))
    slon = radians(longitude(from_city))
    elat = radians(latitude(to_city))
    elon = radians(longitude(to_city))
    dist = 6371.01 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon - elon))
    return dist

def distance(from_city, to_city):
    return int(data[from_city]['to_city'][to_city]['dist'])

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

def distance_of_path(path):
    total_dist = 0
    for i in range(len(path)-1):
        c1 = path[i]
        c2 = path[i+1]
        total_dist += distance(c1, c2)
    return total_dist

def time_of_path(path):
    total_time = 0
    for i in range(len(path)-1):
        c1 = path[i]
        c2 = path[i+1]
        total_time += time(c1, c2)
    return round(total_time, 4)

def segments_of_path(path):
    return len(path)-1

def is_goal(city):
    return city == end_city

def successors(city):
    return data[city]['to_city'].keys()

def solve(start_city):
    # fringe is is a list of path where each path can explored further
    fringe = [[start_city]]
    # List of the visited cities
    visited = []
    # While there are still paths to be explored
    while fringe:
        # For switching between bfs dfs, use pop(0) for BFS, pop() for DFS
        if routing_algorithm == 'bfs':
            i = 0
        if routing_algorithm == 'dfs':
            i = -1
        curr_path = fringe.pop(i)
        # Retreive the last city from the path to be expanded
        curr_city = curr_path[-1]
        # For all cities that we can go from the current city
        for next_city in successors(curr_city):
            # And if the next_city is already visited, discard
            if next_city in visited:
                continue
            # Updating path
            new_path = curr_path + [next_city]
            # Check if it's our goal state then return the path
            if is_goal(next_city):
                return new_path
            # Add this current city to our list of visited cities
            visited.append(next_city)
            # Add the new expanded path to our paths list
            fringe.append(new_path)

    # No route found
    return False


#==============================================================================
# 1. If GOAL?(initial-state) then return initial-state
# 2. INSERT(initial-node, FRINGE)
# 3. Repeat:
# 4.   If empty(FRINGE) then return failure
# 5.   s  REMOVE(FRINGE)
# 6.   INSERT(s, CLOSED)
# 7.   If GOAL?(s) then return s and/or path
# 8.   For every state s’ in SUCC(s):
# 9.       If s’ in CLOSED, discard s’
# 10.     If s’ in FRINGE with larger s’, remove from FRINGE
# 11.	    If s’ not in FRINGE, INSERT(s’, FRINGE)
#==============================================================================

def solve3(start_city):
    import heapq
    # fringe = [ ( cost, [path], distance, time ), (...), ... ]
    fringe = []
    heapq.heappush(fringe, [0, [start_city]])
    closed = []
    while fringe:
        print("heap", fringe)
#        s = REMOVE(FRINGE)
        s = heapq.heappop(fringe)
#        print("1", s)
        curr_path, curr_cost = s[1],  s[0]
        curr_city = curr_path[-1]
#        INSERT(s, CLOSED)
        closed.append(curr_city)
#        If GOAL?(s) then return s and/or path
        if is_goal(curr_city):
            return curr_path

        for next_city in successors(curr_city):
#            If s’ in CLOSED, discard s’
            if next_city in closed:
                continue

            new_path = curr_path + [next_city]
#            print("p",curr_path, new_path)

            # There is problem here curr_cost = g+h and we don't want h
            g_next_city = curr_cost + cost(curr_city, next_city)
            h_next_city = 0
            if routing_algorithm == 'astar':
                h_next_city += lat_lon_distance(next_city, end_city)

            f_next_city = g_next_city + h_next_city

#            If s’ in FRINGE with larger s’, remove from FRINGE
            flag = False
            for e in fringe:
                if e[1] == new_path and e[0] > f_next_city:
                    flag = True
                    fringe.remove(e)
                    heapq.heappush(fringe, [f_next_city, new_path])
                    break

#            If s’ not in FRINGE, INSERT(s’, FRINGE)
            if flag == True:
                continue
            heapq.heappush(fringe, [f_next_city, new_path])

    # No route found
    return False


## check for start city == end city?

start_city = 'Bloomington,_Indiana' #sys.argv[0]
end_city = 'Indianapolis,_Indiana' #sys.argv[1]
routing_algorithm = 'bfs' #sys.argv[2]
cost_function = 'distance' #sys.argv[3]

data = reading_files()

try:
    if routing_algorithm in ['bfs', 'dfs']:
        solution = solve(start_city)

    elif routing_algorithm in ['uniform', 'astar']:
        solution = solve(start_city)
    else:
        print("Need extra credits")
    distance_of_solution = distance_of_path(solution)
    time_of_solution = time_of_path(solution)
    solution_str = ' '.join(solve(start_city))
    print(' '.join([str(distance_of_solution), str(time_of_solution), solution_str]))
except TypeError:
    print("No route found!")

