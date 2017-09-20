# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 17:31:29 2017

@author: PulkitMaloo
"""

import sys
import numpy as np


def reading_files():
    data = dict()
    f_city = open('city-gps.txt', 'r')
    f_road = open('road-segments.txt', 'r')

    for city in f_city:
        city = city.split()
        data[city[0]] = {'latitude': city[1], 'longitude': city[2]}

    for seg in f_road:
        seg = seg.split()

        # Some road segment are not in city-gps, they don't have lat or lon
        if seg[0] not in data:
            data[seg[0]] = {'latitude': None, 'longitude': None}

        # Some road segment didn't have speed
        if len(seg) != 5:
             seg = seg[:3] + [None] + seg[3:]

        if 'to_city' not in data[seg[0]]:
            data[seg[0]]['to_city'] = []

        to_city = {'name': seg[1], 'dist': seg[2], 'speed': seg[3], 'highway':seg[4]}
        data[seg[0]]['to_city'].append(to_city)

    f_city.close()
    f_road.close()
    return data

#def solve(data):
#    solution = []
#    fringe = []
#    return solution

data = reading_files()

