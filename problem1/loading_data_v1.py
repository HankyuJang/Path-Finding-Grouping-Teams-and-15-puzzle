# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 17:31:29 2017

@author: PulkitMaloo
"""
from __future__ import division
#import sys
#import numpy as np
import heapq

# Assumption 1
# speed = 45 for missing or 0 values
# since majority of the road segments seem to have speed 45
speed_limit = 40
radius_of_earth = 3949
# Assumption 2
# When speed=0 and distance=0 there is no route possible
# Because the highway name is ferry, there's no road there

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
#           "parent": city-name(str)
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
        data[city[0]] = {'latitude': float(city[1]), 'longitude': float(city[2]), 'visited': False, 'parent': None, 'cost': 0}

    for seg in f_road:
        seg = seg.split()

        # One city has a route to itself
        if seg[0] == seg[1]:
            continue

        # Handling Missing speed values by returning 45
        if len(seg) != 5:
            seg = seg[:3] + [speed_limit] + seg[3:]

        # Handling speed = 0
        if int(seg[3]) == 0:
            seg[3] = speed_limit

        # Ferry cases where distance=0 and speed=0 (already fixed)
        if int(seg[2])==0:
            continue

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

def lat_lon(city):
    if data[city]['latitude'] == None:
        # If city does not has a latitude longitude
#        data[city]['latitude'], data[city]['longitude'] = lat_lon(dist_nearest_city(city)[1])
        return lat_lon(dist_nearest_city(city)[1])
    return data[city]['latitude'], data[city]['longitude']

# Returns the nearest city and its distance from the current city which has latitude and longitude
def dist_nearest_city(city):
    nearest_cities = data[city]['to_city'].keys()
    d = []
    for c in nearest_cities:
        if data[c]['latitude'] == None:
            continue
        heapq.heappush(d, (distance(city, c), c))
    return heapq.heappop(d)

# Returns the farthest city from the current city
def dist_farthest_city(city):
    k = data[city]['to_city'].keys()
    v = [distance(city, i) for i in k]
    return k[v.index(max(v))]

# Returns the nearest city costwise from the current city
def cost_nearest_city(city):
    k = data[city]['to_city'].keys()
    v = [cost(city, i) for i in k]
    return k[v.index(min(v))]

# Heuristic function: calculates manhattan distance between two cities
def manhattan_distance(from_city, to_city):
# This function is copied from the following website
# https://www.w3resource.com/python-exercises/math/python-math-exercise-27.php
    from math import radians, sin, cos, acos
    slat, slon = map(radians, lat_lon(from_city))
    elat, elon = map(radians, lat_lon(to_city))
    dist = radius_of_earth*acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon - elon))
    return dist

def heuristic(city):
	if cost_function == 'distance':
		return manhattan_distance(city, end_city)
	elif cost_function == 'time':
		# time = dist/speed
		# could take the const min speed cause it will never overestimate
		return "time heuristic"
	else:
		# segment could be written as some heuristic of dist
		return "Segment heuristic"


def distance(from_city, to_city):
    return data[from_city]['to_city'][to_city]['distance']

def speed(from_city, to_city):
    return data[from_city]['to_city'][to_city]['speed']

def time(from_city, to_city):
    return data[from_city]['to_city'][to_city]['time']

#Returns cost between two cities according to the cost_function
def cost(from_city, to_city, cost = cost_function):
        return data[from_city]['to_city'][to_city][cost]

def distance_of_path(path):
    return sum([distance(path[i], path[i+1]) for i in range(len(path)-1)])

def time_of_path(path):
    return sum([time(path[i], path[i+1]) for i in range(len(path)-1)])

def segments_of_path(path):
    return len(path)-1

def cost_of_path(path):
    return eval(cost_function+"_of_path(path)")

def path(city = end_city):
    current = city
    current_path = [city]
    while current != start_city:
#        print(current_path)
        current_path.append(data[current]['parent'])
        current = data[current]['parent']
#    print len(set(current_path)), len(current_path)
    return distance_of_path(current_path[::-1]), time_of_path(current_path[::-1]), current_path[::-1]

def solve(start_city):
    # For switching between bfs dfs, use pop(0) for BFS, pop() for DFS
    i = {'bfs':0, 'dfs':-1}[routing_algorithm]
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


#===  SA #2   =================================================================
# 1. If GOAL?(initial-state) then return initial-state
# 2. INSERT(initial-node, FRINGE)
# 3. Repeat:
# 4. 	If empty(FRINGE) then return failure
# 5.		s  REMOVE(FRINGE)
# 6.		If GOAL?(s) then return s and/or path
# 7.    For every state s’ in SUCC(s):
# 8.		    INSERT(s’, FRINGE)
#==== SA #3   =================================================================
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
    fringe = []
    heapq.heappush(fringe, [0, start_city])
    while fringe:
#        print fringe, "\n"
        curr_city = heapq.heappop(fringe)[1]
        curr_cost = data[curr_city]['cost']
        data[curr_city]['visited'] = True
        if curr_city == end_city:
            print fringe, '\n'
            print "Goal Reached in", cost_function, curr_cost
            return path(curr_city)
        for next_city in data[curr_city]['to_city']:
            if data[next_city]['visited']:
                continue
            next_cost = curr_cost + cost(curr_city, next_city)
            if data[next_city]['parent']:
                p = data[next_city]['parent']
                if next_cost > data[p]['cost'] + cost(p, next_city):
                    continue
                else:
                    data[next_city]['parent'] = curr_city
            else:
                data[next_city]['parent'] = curr_city
            data[next_city]['cost'] = next_cost
            heapq.heappush(fringe, [next_cost, next_city])
    return False

## check for start city == end city?

start_city = 'Bloomington,_Indiana' #sys.argv[0]
end_city = 'Chicago,_Illinois' #sys.argv[1]
routing_algorithm = 'uniform' #sys.argv[2]
cost_function = 'distance' #sys.argv[3]

data = reading_files()

#import matplotlib.pyplot as plt
#fig = plt.figure()
#ax = fig.gca()
#fig1 = plt.figure()
#ax1 = fig1.gca()
#ax1.scatter(latitude(start_city), longitude(start_city), color='y', s = 100)
#ax1.scatter(latitude(end_city), longitude(end_city), color='g', s = 100)

try:
    if routing_algorithm in ['bfs', 'dfs']:
        solution = solve(start_city)
    elif routing_algorithm in ['uniform', 'astar']:
        solution = solve3(start_city)
    else:
        print("Need extra credits")

    print solution[0], round(solution[1], 3), ' '.join(solution[2])

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


