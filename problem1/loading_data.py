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
import heapq
#import matplotlib.pyplot as plt
# ==============================================================================
#   The format of data is:
#
# data =
#   {
#   city:
#       {
#           "latitude": value(float) / None
#           "longitude": value(float) / None
#           "visited": False(bool)
#           "to_city":
#               {
#                   city: {"distance": value(int), "speed": value(int), "time":value(int), "highway": value(str)},
#                   city: {..},
#                   ...
#               }
#       }
#   }
# ==============================================================================

def reading_files():
    data = dict()
    f_city = open('city-gps.txt', 'r')
    f_road = open('road-segments.txt', 'r')

    for city in f_city:
        city = city.split()
        data[city[0]] = {'latitude': float(city[1]), 'longitude': float(city[2]), 'visited': False}

    for seg in f_road:
        seg = seg.split()

        # One city has a route to itself
        if seg[0] == seg[1]:
            continue

        # Handling Missing speed values by returning 45
        if len(seg) != 5:
            seg = seg[:3] + [45] + seg[3:]

        # Handling speed = 0
        if int(seg[3]) == 0:
            seg[3] = 45

        # Ferry cases where distance=0 and speed=0 (already fixed)
        if int(seg[2])==0:
            continue

        # Some road segment are not in city-gps, they don't have lat or lon
        if seg[0] not in data:
            data[seg[0]] = {'latitude': None, 'longitude': None, 'visited': False}

        # from_city to to_city
        if 'to_city' not in data[seg[0]]:
            data[seg[0]]['to_city'] = dict()
        data[seg[0]]['to_city'][seg[1]] = {'distance': int(seg[2]), 'speed': int(seg[3]), 'time': int(seg[2])/int(seg[3]),'highway': seg[4]}

        # to_city to from_city
        if seg[1] not in data:
            data[seg[1]] = {'latitude': None, 'longitude': None, 'visited': False}

        if 'to_city' not in data[seg[1]]:
            data[seg[1]]['to_city'] = dict()
        data[seg[1]]['to_city'][seg[0]] = {'distance': int(seg[2]), 'speed': int(seg[3]), 'time': int(seg[2])/int(seg[3]), 'highway': seg[4]}


    f_city.close()
    f_road.close()
    return data

def latitude(city):
    if data[city]['latitude'] == None:
        data[city]['latitude'] = data[dist_nearest_city(city)]['latitude']
    return data[city]['latitude']

def longitude(city):
    if data[city]['longitude'] == None:
        data[city]['longitude'] = data[dist_nearest_city(city)]['longitude']
    return data[city]['longitude']

# Returns the nearest city from the current city
def dist_nearest_city(city):
    k = successors(city)
    v = [distance(city, i) for i in k]
    return k[v.index(min(v))]

# Returns the farthest city from the current city
def dist_farthest_city(city):
    k = successors(city)
    v = [distance(city, i) for i in k]
    return k[v.index(max(v))]

# Returns the nearest city costwise from the current city
def cost_nearest_city(city):
    k = successors(city)
    v = [cost(city, i) for i in k]
    return k[v.index(min(v))]

# Heuristic function: calculates manhattan distance between two cities
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

# Returns distance between two cities
def distance(from_city, to_city):
    return data[from_city]['to_city'][to_city]['distance']

# Returns speed between two cities
def speed(from_city, to_city):
    return data[from_city]['to_city'][to_city]['speed']

# Returns tiem between two cities
def time(from_city, to_city):
    return data[from_city]['to_city'][to_city]['time']

#Returns cost between two cities according to the cost_function
def cost(from_city, to_city):
    try:
        return data[from_city]['to_city'][to_city][cost_function]
    except:
        return 1

# Returns distance of a path
def distance_of_path(path):
    total_dist = 0
    for i in range(len(path)-1):
        c1 = path[i]
        c2 = path[i+1]
        total_dist += distance(c1, c2)
    return total_dist

# Returns time of a path
def time_of_path(path):
    total_time = 0
    for i in range(len(path)-1):
        c1 = path[i]
        c2 = path[i+1]
        total_time += time(c1, c2)
    return round(total_time, 4)

# Returns segments of a path
def segments_of_path(path):
    return len(path)-1

# Returns cost of a path according to the cost_function
def cost_of_path(path):
    return eval(cost_function+"_of_path(path)")

def is_visited(city):
    return data[city]['visited']

def is_goal(city):
    return city == end_city

def successors(city):
    return data[city]['to_city'].keys()

def solve(start_city):
    # fringe is is a list of path where each path can explored further
    fringe = [[start_city]]
    # While there are still paths to be explored
    while fringe:
        # For switching between bfs dfs, use pop(0) for BFS, pop() for DFS
        i = {'bfs':0, 'dfs':-1}[routing_algorithm]
        curr_path = fringe.pop(i)
        # Retreive the last city from the path to be expanded
        curr_city = curr_path[-1]
        # For all cities that we can go from the current city
        for next_city in successors(curr_city):
            # And if the next_city is already visited, discard
            if is_visited(next_city):
                continue
            # Updating path
            new_path = curr_path + [next_city]
            # Check if it's our goal state then return the path
            if is_goal(next_city):
                return new_path
            # Add this current city to our list of visited cities
            data[next_city]['visited'] = True
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
    # fringe = [ ( cost, [path], distance, time, segments ), (...), ... ]
    fringe = []
    heapq.heappush(fringe, [0, [start_city], 0, 0, 0])

    while fringe:
#        print "heap", fringe, "\n"
#        s = REMOVE(FRINGE)
        s = heapq.heappop(fringe)
#        print("1", s)
        curr_path, curr_cost, curr_dist, curr_time, curr_segments = s[1], s[0], s[2], s[3], s[4]

        if routing_algorithm == 'astar':
            if cost_function == 'distance':
                curr_cost = curr_dist
            elif cost_function == 'time':
                curr_cost = curr_time
            else:
                curr_cost = curr_segments

        curr_city = curr_path[-1]
#        INSERT(s, CLOSED)
        data[curr_city]['visited'] = True

#        color=iter(plt.cm.rainbow(np.linspace(0,1,len(solution))))

#        If GOAL?(s) then return s and/or path
        if is_goal(curr_city):
#            print(heapq.nsmallest(10,fringe))
            return [str(curr_dist), str(curr_time)] + curr_path

        for next_city in successors(curr_city):
#            If s’ in CLOSED, discard s’
            if is_visited(next_city):
                continue

            d_next_city = distance(curr_city, next_city)
            t_next_city = time(curr_city, next_city)

            new_path = curr_path + [next_city]
            new_dist = curr_dist + d_next_city
            new_time = curr_time + t_next_city
            new_segments = curr_segments + 1
#            print("p",curr_path, new_path)

            # There is problem here curr_cost = g+h and we don't want h
            g_next_city = curr_cost + cost(curr_city, next_city)
#            print(g_next_city)
            h_next_city = 0
            if routing_algorithm == 'astar':
                h_next_city += lat_lon_distance(next_city, end_city)

            f_next_city = g_next_city + h_next_city

#            If s’ in FRINGE with larger s’, remove from FRINGE
#            flag = False
#            for e in fringe:
#                if e[1] == new_path and e[0] > f_next_city:
#                    flag = True
#                    fringe.remove(e)
#                    heapq.heappush(fringe, [f_next_city, new_path, new_dist, new_time, new_segments])
#                    break
##
###            If s’ not in FRINGE, INSERT(s’, FRINGE)
#            if flag == True:
#                continue

#            ax1.scatter(latitude(curr_city), longitude(curr_city), color=next(color))
#            ax1.plot([latitude(curr_city), latitude(next_city)], [longitude(curr_city), longitude(next_city)], color=next(color))
#            print([f_next_city, new_path])
            heapq.heappush(fringe, [f_next_city, new_path, new_dist, new_time, new_segments])
    # No route found
    return False


## check for start city == end city?

start_city = 'Bloomington,_Indiana' #'Abbot_Village,_Maine' #sys.argv[0]
end_city = 'Seattle,_Washington' #'Abbotsford,_Wisconsin' #sys.argv[1]
routing_algorithm = 'uniform' #sys.argv[2]
cost_function = 'distance' #sys.argv[3]

data = reading_files()

#fig = plt.figure()
#ax = fig.gca()
#fig1 = plt.figure()
#ax1 = fig1.gca()
#ax1.scatter(latitude(start_city), longitude(start_city), color='y', s = 100)
#ax1.scatter(latitude(end_city), longitude(end_city), color='g', s = 100)

try:
    if routing_algorithm in ['bfs', 'dfs']:
        print(' '.join(solve(start_city)))
    elif routing_algorithm in ['uniform', 'astar']:
        print(' '.join(solve3(start_city)))
    else:
        print("Need extra credits")

#    color1=iter(plt.cm.rainbow(np.linspace(0,1,len(solution))))
#    color2=iter(plt.cm.rainbow(np.linspace(0,1,len(solution))))
#    for i in range(len(solution)):
#        city1 = solution[i]
#        city2 = solution[i+1]
#        ax.plot([latitude(city1), latitude(city2)], [longitude(city1), longitude(city2)], color=next(color1))
#        ax.scatter(latitude(city1), longitude(city1), color=next(color2))
#    ax.scatter(latitude(start_city), longitude(start_city), color='y', s = 100)
#    ax.scatter(latitude(end_city), longitude(end_city), color='g', s = 100)

except TypeError:
    print("No route found!")


