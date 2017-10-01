# put your routing program here!
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 17:31:29 2017

@author: PulkitMaloo
"""
from __future__ import division
import sys
#import numpy as np
import heapq
radius_of_earth = 3959
# Assumption 1
# speed = 45 for missing or 0 values
# since majority of the road segments seem to have speed 45
speed_limit = 30
max_speed = 65
max_dist = 923
heuristic_factor = 1
# Assumption 2
# When speed=0 and distance=0 there is no route possible
# Because the highway name is ferry, there's no road there
# =============================================================
#   The format of data is:
# data =
#   {
#   city:
#       {
#           "latitude": value(float) / None
#           "longitude": value(float) / None
#           "visited": False(bool)
#           "parent": city(str)
#           "cost": value(int)
#           "to_city":
#               {
#                   city: {"distance": value(int), "speed": value(int), "time":value(int), segments:1(int) "highway": value(str)},
#                   city: {..},
#                   ...
#               }
#       }
#   }
# =============================================================
def reading_files():
    data = dict()
    f_city = open('city-gps.txt', 'r')
    f_road = open('road-segments.txt', 'r')
    for city in f_city:
        city = city.split()
        data[city[0]] = {'latitude': float(city[1]), 'longitude': float(city[2]), 'visited': False, 'parent': None, 'cost': 0}
    for seg in f_road:
        seg = seg.split()
        # One city has a route to itself
        if seg[0] == seg[1]:
            continue
        # Updating missing speed values by a constant speed_limit
        if len(seg) != 5:
            seg = seg[:3] + [speed_limit] + seg[3:]
        # Ferry cases where distance = 0 and speed = 0
        if int(seg[2])==0:
            continue
        # Updating speed = 0
        if int(seg[3]) == 0:
            seg[3] = speed_limit
        # Some road segment are not in city-gps, they don't have lat or lon
        if seg[0] not in data:
            data[seg[0]] = {'latitude': None, 'longitude': None, 'visited': False, 'parent': None, 'cost': 0}
        # from_city to to_city
        if 'to_city' not in data[seg[0]]:
            data[seg[0]]['to_city'] = dict()
        data[seg[0]]['to_city'][seg[1]] = {'distance':int(seg[2]), 'speed':int(seg[3]), 'time':int(seg[2])/int(seg[3]), 'segments':1, 'highway':seg[4]}
        # to_city to from_city
        if seg[1] not in data:
            data[seg[1]] = {'latitude': None, 'longitude': None, 'visited': False,  'parent': None, 'cost': 0}
        if 'to_city' not in data[seg[1]]:
            data[seg[1]]['to_city'] = dict()
        data[seg[1]]['to_city'][seg[0]] = {'distance':int(seg[2]), 'speed':int(seg[3]), 'time':int(seg[2])/int(seg[3]), 'segments':1, 'highway':seg[4]}
    f_city.close()
    f_road.close()
    return data

## Returns the farthest city from the current city
#def dist_farthest_city(city):
#    k = data[city]['to_city'].keys()
#    v = [distance(city, i) for i in k]
#    return k[v.index(max(v))]
#
## Returns the nearest city costwise from the current city
#def cost_nearest_city(city):
#    k = data[city]['to_city'].keys()
#    v = [cost(city, i) for i in k]
#    return k[v.index(min(v))]

# Returns the nearest city and its distance from the current city which has latitude and longitude
def dist_nearest_city(city):
    nearest_cities = successors(city)
    d = []
    for c in nearest_cities:
        if data[c]['latitude'] == None:
            continue
        heapq.heappush(d, (distance(city, c), c))
    return heapq.heappop(d)

def lat_lon(city):
    return data[city]['latitude'], data[city]['longitude']

# Heuristic function: calculates manhattan distance between two cities
def euclidean_distance(from_city, to_city):
# This function is copied from the following website
# https://www.w3resource.com/python-exercises/math/python-math-exercise-27.php
    from math import radians, sin, cos, acos
    slat, slon = map(radians, lat_lon(from_city))
    elat, elon = map(radians, lat_lon(to_city))
    dist = radius_of_earth*acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon - elon))
    return dist

def heuristic(city):
    dist = 0
    try:
        if data[city]['latitude']:
            dist = euclidean_distance(city, end_city)
        else:
            d, nearest_city = dist_nearest_city(city)
            dist = heuristic(nearest_city) - d
            dist = dist if dist > 0 else 0
    except:
        return 0
    dist = dist*heuristic_factor
    if cost_function == 'distance':
        return dist
    elif cost_function == 'time':
        return dist/max_speed
    else:
        return dist/max_dist

def distance(from_city, to_city):
    return data[from_city]['to_city'][to_city]['distance']

def speed(from_city, to_city):
    return data[from_city]['to_city'][to_city]['speed']

def time(from_city, to_city):
    return data[from_city]['to_city'][to_city]['time']

def cost(from_city, to_city):
        return data[from_city]['to_city'][to_city][cost_function]

def distance_of_path(path):
    return sum([distance(path[i], path[i+1]) for i in range(len(path)-1)])

def time_of_path(path):
    return sum([time(path[i], path[i+1]) for i in range(len(path)-1)])

def segments_of_path(path):
    return len(path)-1

def cost_of_path(path):
    return eval(cost_function+"_of_path(path)")

def path(city):
    current = city
    current_path = [city]
    while current != start_city:
        current_path.append(data[current]['parent'])
        current = data[current]['parent']
    return distance_of_path(current_path), time_of_path(current_path), current_path[::-1]

def successors(city):
    return data[city]['to_city'].keys()

#===  SA #1   =================================================
# 1. If GOAL?(initial-state) then return initial-state
# 2. INSERT(initial-node, FRINGE)
# 3. Repeat:
# 4.  	If empty(FRINGE) then return failure
# 5.		s  REMOVE(FRINGE)
# 6.		For every state s’ in SUCC(s):
# 7.			If GOAL?(s’) then return s’ and/or path
# 8.       INSERT(s’, FRINGE)
#==============================================================
def solve1(start_city, end_city):
    # For switching between bfs dfs, use pop(0) for BFS, pop() for DFS
    i = {'bfs': 0, 'dfs': -1}[routing_algorithm]
    # fringe is a list of cities which can explored further
    fringe = [start_city]
    data[start_city]['visited'] = True
    # While there are still cities to be explored
    while fringe:
        # Retreive the city from the fringe to be expanded
        curr_city = fringe.pop(i)
        # For all cities that we can go from the current city
        for next_city in data[curr_city]['to_city']:
            # And if the next_city is already visited, discard
            if data[next_city]['visited']:
                continue
            data[next_city]['parent'] = curr_city
            # Check if it's our goal state then return the path
            if next_city == end_city:
                return path(next_city)
            # Mark this city visited
            data[next_city]['visited'] = True
            # Add the new city to our fringe
            fringe.append(next_city)
    # No route found
    return False

#===  SA #2   =================================================
# 1. If GOAL?(initial-state) then return initial-state
# 2. INSERT(initial-node, FRINGE)
# 3. Repeat:
# 4. 	If empty(FRINGE) then return failure
# 5.		s  REMOVE(FRINGE)
# 6.		If GOAL?(s) then return s and/or path
# 7.    For every state s’ in SUCC(s):
# 8.		    INSERT(s’, FRINGE)
#==============================================================
def solve2(start_city, end_city):
    fringe = [[heuristic(start_city), start_city]]
    while fringe:
        curr_city = heapq.heappop(fringe)[1]
        g_curr_cost = data[curr_city]['cost']
        if curr_city == end_city:
            return path(curr_city)
        for next_city in data[curr_city]['to_city']:
            g_next_city = g_curr_cost + cost(curr_city, next_city)
            h_next_city = heuristic(next_city)
            f_next_city = g_next_city + h_next_city
            if data[next_city]['parent']:
                if g_next_city >= data[next_city]['cost']:
                    continue
                else:
                    try:
                        fringe.remove([data[next_city]['cost'] + h_next_city, next_city])
                    except:
                        pass
                    heapq.heapify(fringe)
            data[next_city]['parent'] = curr_city
            data[next_city]['cost'] = g_next_city
            heapq.heappush(fringe, [f_next_city, next_city])
    return False

#==== SA #3   =================================================
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
#==============================================================
def solve3(start_city, end_city):
    fringe = [[0, start_city]]
    while fringe:
        curr_city = heapq.heappop(fringe)[1]
        curr_cost = data[curr_city]['cost']
        data[curr_city]['visited'] = True
        if curr_city == end_city:
            return path(curr_city)
        for next_city in data[curr_city]['to_city']:
            if data[next_city]['visited']:
                continue
            next_cost = curr_cost + cost(curr_city, next_city)
            if data[next_city]['parent']:
                if next_cost > data[next_city]['cost']:
                    continue
                else:
                    fringe.remove([data[next_city]['cost'], next_city])
                    heapq.heapify(fringe)
            data[next_city]['parent'] = curr_city
            data[next_city]['cost'] = next_cost
            heapq.heappush(fringe, [next_cost, next_city])
    return False

def solve4(start_city, end_city):
    fringe = [[-heuristic(start_city), start_city]]
    while fringe:
        curr_city = heapq.heappop(fringe)[1]
        g_curr_cost = data[curr_city]['cost']
        if curr_city == end_city:
            return path(curr_city)
        for next_city in data[curr_city]['to_city']:
            g_next_city = g_curr_cost - cost(curr_city, next_city)
            try:
                h_next_city = -heuristic(next_city)
            except:
                h_next_city = 0
            f_next_city = g_next_city + h_next_city
            if data[next_city]['parent']:
                if g_next_city < data[next_city]['cost']:
                    continue
                else:
                    try:
                        fringe.remove([data[next_city]['cost'] + h_next_city, next_city])
                    except:
                        pass
                    heapq.heapify(fringe)
            data[next_city]['parent'] = curr_city
            data[next_city]['cost'] = g_next_city
            heapq.heappush(fringe, [f_next_city, next_city])
    return False

start_city = 'Bloomington,_Indiana' #sys.argv[0]
end_city = 'Seattle,_Washington' #sys.argv[1]
routing_algorithm = 'longtour' #sys.argv[2]
cost_function = 'distance' #sys.argv[3]

data = reading_files()
try:
    if routing_algorithm in ['bfs', 'dfs']:
        solution = solve1(start_city, end_city)
    elif routing_algorithm == 'uniform':
        solution = solve3(start_city, end_city)
    elif routing_algorithm == 'astar':
        solution = solve2(start_city, end_city)
    elif routing_algorithm == 'longtour':
        solution = solve4(start_city, end_city)
    else:
        print("Need extra credits")
    print solution[0], round(solution[1], 4), ' '.join(solution[2])
except TypeError:
    print("No route found!")


